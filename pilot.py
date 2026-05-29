"""Fly the Tello with an Xbox controller (Mode 2 RC), via Apple's GameController.

    uv run python pilot.py list                 # show controller connection
    uv run python pilot.py monitor              # live named values (sign check)
    uv run python pilot.py fly                  # arm + fly (drone must be on Tello WiFi)
    uv run python pilot.py fly --photos         # same, plus camera + photo button

Controller input comes from the compiled Swift helper `gamepad-reader`
(GameController framework), which streams JSON state on stdout. SDL/pygame can't
read the pad on macOS (raw-HID is Input-Monitoring-gated); GameController can, but
ONLY delivers input to a FOCUSED, foreground app. So `fly`/`monitor` must run in a
Terminal window that stays frontmost on loki's display. If the window loses focus,
the stream goes to zeros -> the drone HOVERS (neutral RC), it does not run away.

Mode 2 layout:
    Left stick   vertical   -> throttle (up/down)
    Left stick   horizontal -> yaw (rotate left/right)
    Right stick  vertical   -> pitch (forward/back)
    Right stick  horizontal -> roll (strafe left/right)

Buttons:
    A      takeoff (from ARMED-on-ground, throttle centered, not just-landed)
    B      land (graceful) -- also the one-button panic stop
    X      photo (with --photos)
    Y      flip forward (flying, battery > 50%)
    Start  arm/disarm on the ground; while FLYING it lands (safe disarm)
    LB     hold = precision speed     RB hold = boost speed
    Back + Start held (~0.3s) = EMERGENCY motor cut (drone drops -- crash only)

Safety: sticks inert unless FLYING; takeoff gated on ARMED + centered throttle +
not-settling; first ~1.2s after takeoff are throttle-ramped; maneuvers run in a
thread so the loop never blocks and EMERGENCY stays reachable; stream stall or
controller disconnect while FLYING -> hover, then auto-land; battery warn <15%,
force-land <10%; Ctrl-C / kill -> graceful land.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
READER_BIN = Path(os.environ.get("TELLO_GAMEPAD_BIN", str(HERE / "gamepad-reader")))
TELEM_LOG = Path(os.environ.get("TELLO_TELEM_LOG", "/tmp/tello-pilot.log"))

# ---- tunables -------------------------------------------------------------
LOOP_HZ = 30
DEADZONE = 0.12
EXPO = 0.45
MAX_RC = 40                  # default roll/pitch cap (gentle indoor)
MAX_UD = 45                  # throttle cap
MAX_YAW = 60                 # yaw-rate cap
BOOST_RC = 80                # roll/pitch cap while RB held
PRECISION_RC = 20            # roll/pitch cap while LB held
TAKEOFF_THROTTLE_GUARD = 0.25
SOFT_START_SEC = 1.2
LAND_SETTLE_SEC = 2.5
STREAM_STALE_SEC = 0.5       # no fresh line in this long -> treat as disconnected
FAILSAFE_LAND_AFTER = 1.5
EMERGENCY_HOLD_SEC = 0.3
RECONNECT_HOLDOFF = 0.25
BATT_WARN = 15
BATT_LAND = 10
BATT_POLL_SEC = 3.0
FINAL_LAND_TIMEOUT = 10.0

BUTTONS = ("a", "b", "x", "y", "lb", "rb", "start", "back")


# ---- shaping --------------------------------------------------------------
def deadzone(v: float, dz: float = DEADZONE) -> float:
    if -dz < v < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def expo(v: float, e: float = EXPO) -> float:
    return (1.0 - e) * v + e * (v ** 3)


def to_rc(v: float, cap: int, ramp: float = 1.0) -> int:
    v = expo(deadzone(v)) * ramp
    return int(max(-cap, min(cap, round(v * cap))))


# ---- maneuver runner ------------------------------------------------------
class Maneuver:
    """Runs one blocking Tello command (takeoff/land/flip) in a daemon thread so
    the control loop never stalls and the EMERGENCY combo stays reachable."""

    def __init__(self):
        self._thread: threading.Thread | None = None
        self.name: str | None = None
        self.done_at: float = 0.0

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, name: str, fn) -> bool:
        if self.active:
            return False
        self.name = name

        def _run():
            try:
                fn()
            except Exception as e:
                sys.stdout.write(f"\n  {name} error: {e}\n")
                sys.stdout.flush()
            finally:
                self.done_at = time.time()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        return True

    def join(self, timeout: float) -> None:
        if self._thread is not None:
            self._thread.join(timeout)


# ---- controller (GameController via the Swift helper subprocess) ----------
class GamepadReader:
    """Reads controller state from the `gamepad-reader` subprocess (one JSON line
    per ~20ms). GC convention: stick up=+1, right=+1. Names, not indices."""

    def __init__(self, binary: Path = READER_BIN):
        self.binary = binary
        self.proc: subprocess.Popen | None = None
        self._state: dict = {}
        self._lock = threading.Lock()
        self._last_update = 0.0
        self._gc_connected = False
        self._prev_buttons: dict[str, bool] = {}
        self._thread: threading.Thread | None = None
        self._stop = False
        self.reconnect_holdoff_until = 0.0

    def open(self) -> bool:
        if not self.binary.exists():
            print(f"gamepad-reader not found at {self.binary}. Build it:")
            print("  swiftc -O gamepad_reader.swift -o gamepad-reader "
                  "-framework GameController -framework AppKit")
            return False
        self.proc = subprocess.Popen(
            [str(self.binary)], stdout=subprocess.PIPE, text=True, bufsize=1
        )
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self.connected:
                return True
            time.sleep(0.05)
        return self.connected

    def _pump(self) -> None:
        assert self.proc and self.proc.stdout
        for line in self.proc.stdout:
            if self._stop:
                break
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            ev = d.get("event")
            if ev == "connect":
                continue
            if ev == "disconnect":
                with self._lock:
                    self._gc_connected = False
                continue
            with self._lock:
                self._state = d
                self._gc_connected = bool(d.get("connected", False))
                prev = self._last_update
                self._last_update = time.time()
            # rising edge of a (re)connection -> brief button holdoff
            if self._gc_connected and (time.time() - prev) > STREAM_STALE_SEC:
                self.reconnect_holdoff_until = time.time() + RECONNECT_HOLDOFF

    @property
    def connected(self) -> bool:
        """Connected AND receiving fresh frames (a stalled stream = disconnected)."""
        with self._lock:
            fresh = (time.time() - self._last_update) < STREAM_STALE_SEC
            return self._gc_connected and fresh

    @property
    def buttons_ready(self) -> bool:
        return time.time() >= self.reconnect_holdoff_until

    def _get(self, key: str, default: float = 0.0) -> float:
        with self._lock:
            return float(self._state.get(key, default))

    def sticks(self) -> tuple[float, float, float, float]:
        return (self._get("lx"), self._get("ly"), self._get("rx"), self._get("ry"))

    def button(self, name: str) -> bool:
        with self._lock:
            return bool(self._state.get(name, 0))

    def pressed(self, name: str) -> bool:
        now = self.button(name)
        was = self._prev_buttons.get(name, False)
        self._prev_buttons[name] = now
        return now and not was

    def close(self) -> None:
        self._stop = True
        if self.proc:
            try:
                self.proc.terminate()
            except Exception:
                pass


# ---- commands -------------------------------------------------------------
def cmd_list() -> int:
    gp = GamepadReader()
    if not gp.open():
        print("No controller stream. Is the pad connected and is this a FOCUSED,")
        print("foreground Terminal on loki's display?")
        return 1
    print("Controller connected and streaming.")
    gp.close()
    return 0


def cmd_monitor(seconds: str = "30") -> int:
    """Live named values -- use to confirm stick directions before flying."""
    gp = GamepadReader()
    if not gp.open():
        print("No controller stream. Focus this window on loki and retry.")
        return 1
    secs = float(seconds)
    end = time.time() + secs
    mon_log = Path("/tmp/tello-monitor.log")
    last_log = 0.0
    print(f"Monitoring {secs:.0f}s. Push LEFT stick UP -> ly should go POSITIVE;")
    print("push RIGHT stick UP -> ry POSITIVE; sticks RIGHT -> lx/rx POSITIVE.\n")
    try:
        while time.time() < end:
            lx, ly, rx, ry = gp.sticks()
            held = [b for b in BUTTONS if gp.button(b)]
            conn = "OK " if gp.connected else "STALE"
            sys.stdout.write(
                f"\r[{conn}] lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} ry={ry:+.2f} held={held}      "
            )
            sys.stdout.flush()
            now = time.time()
            if now - last_log >= 0.3:
                last_log = now
                try:
                    mon_log.write_text(
                        f"{conn} lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} ry={ry:+.2f} held={held}\n"
                    )
                except Exception:
                    pass
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        gp.close()
    print("\n(monitor done)")
    return 0


def _trigger_land(maneuver: Maneuver, t, why: str) -> None:
    try:
        t.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass
    print(f"\n[{why}] landing...")
    maneuver.start("land", t.land)


def _telem(line: str) -> None:
    try:
        with open(TELEM_LOG, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass


def cmd_fly(photos: bool = False) -> int:
    from djitellopy import Tello

    gp = GamepadReader()
    if not gp.open():
        print("No controller stream -- cannot fly. Focus this window on loki.")
        return 1
    print("Controller streaming.")

    t = Tello()
    try:
        t.connect()
    except Exception as e:
        print(f"Cannot reach the Tello (192.168.10.1): {e}")
        print("Join the TELLO-XXXXXX WiFi first (drone.py join-wifi).")
        gp.close()
        return 1
    batt = t.get_battery()
    print(f"Connected. battery={batt}%")
    if batt < BATT_LAND:
        print("Battery too low to fly. Charge first.")
        gp.close()
        return 1
    if photos:
        try:
            t.streamon()
            time.sleep(2.0)
        except Exception as e:
            print(f"  stream on failed (photos disabled): {e}")
            photos = False

    cap_dir = Path.home() / "captures" / time.strftime("%Y-%m-%d") / f"pilot-{time.strftime('%H%M%S')}"

    state = "DISARMED"   # DISARMED -> ARMED -> FLYING -> LANDING -> ARMED
    maneuver = Maneuver()
    stop = {"flag": False}

    def _sig(_signum, _frame):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    print()
    print("READY. START to ARM, A to take off, B to land. Ctrl-C to bail.")
    print("EMERGENCY motor-cut = hold BACK+START. Keep THIS window focused while flying.")
    print()
    _telem(f"--- fly session {time.strftime('%H:%M:%S')} batt={batt}% ---")

    period = 1.0 / LOOP_HZ
    last_batt_check = 0.0
    last_telem = 0.0
    emergency_held_since: float | None = None
    fly_since = 0.0
    land_done_at = 0.0
    prev_active = False
    stale_since: float | None = None

    def status(rc, extra=""):
        lr, fb, ud, yaw = rc
        conn = "OK " if gp.connected else "STALE"
        sys.stdout.write(
            f"\r[{state:8s}] batt={batt:3d}% ctl={conn} rc=(lr={lr:+4d} fb={fb:+4d} ud={ud:+4d} yaw={yaw:+4d}) {extra}      "
        )
        sys.stdout.flush()

    try:
        while not stop["flag"]:
            t0 = time.time()
            active = maneuver.active

            if prev_active and not active:
                if maneuver.name == "takeoff":
                    fly_since = maneuver.done_at
                elif maneuver.name == "land":
                    land_done_at = maneuver.done_at
            prev_active = active

            # --- EMERGENCY combo: every loop, even mid-maneuver ---
            if gp.button("back") and gp.button("start"):
                if emergency_held_since is None:
                    emergency_held_since = t0
                elif t0 - emergency_held_since >= EMERGENCY_HOLD_SEC:
                    print("\n[EMERGENCY] cutting motors!")
                    try:
                        t.emergency()
                    except Exception:
                        pass
                    state = "DISARMED"
                    break
            else:
                emergency_held_since = None

            # --- failsafe: stream stalled / controller gone (true disconnect;
            #     a focus-loss instead streams zeros -> hover via the FLYING path) ---
            if not gp.connected:
                if stale_since is None:
                    stale_since = t0
                if state == "FLYING" and not active:
                    t.send_rc_control(0, 0, 0, 0)  # hover (neutral)
                    if (t0 - stale_since) >= FAILSAFE_LAND_AFTER:
                        _trigger_land(maneuver, t, "FAILSAFE controller lost")
                        state = "LANDING"
                status((0, 0, 0, 0), extra="(controller stale)")
                _sleep_rest(t0, period)
                continue
            stale_since = None

            # --- LANDING -> ARMED once settled ---
            if state == "LANDING" and not active and (t0 - land_done_at) >= LAND_SETTLE_SEC:
                state = "ARMED"
                print("\n  -> ARMED (settled)")

            # --- battery watchdog ---
            if t0 - last_batt_check >= BATT_POLL_SEC:
                try:
                    batt = t.get_battery()
                except Exception:
                    pass
                last_batt_check = t0
                if state == "FLYING" and not active and batt <= BATT_LAND:
                    _trigger_land(maneuver, t, f"LOW BATTERY {batt}%")
                    state = "LANDING"

            # --- buttons (edge-triggered; suppressed briefly after reconnect) ---
            if gp.buttons_ready and not active:
                if gp.pressed("start"):
                    if state == "FLYING":
                        _trigger_land(maneuver, t, "disarm (START)")
                        state = "LANDING"
                    elif state == "DISARMED":
                        state = "ARMED"
                        print("\n  -> ARMED")
                    elif state == "ARMED":
                        state = "DISARMED"
                        print("\n  -> DISARMED")

                if gp.pressed("a") and state == "ARMED":
                    _, ly, _, _ = gp.sticks()
                    if (t0 - land_done_at) < LAND_SETTLE_SEC:
                        print("\n  takeoff blocked: still settling from last landing")
                    elif abs(ly) >= TAKEOFF_THROTTLE_GUARD:
                        print("\n  takeoff blocked: center the throttle (left stick) first")
                    else:
                        print("\n  takeoff...")
                        state = "FLYING"
                        fly_since = 0.0
                        maneuver.start("takeoff", t.takeoff)

                if gp.pressed("b") and state == "FLYING":
                    _trigger_land(maneuver, t, "land (B)")
                    state = "LANDING"

                if photos and gp.pressed("x") and state == "FLYING":
                    _snap(t, cap_dir)

                if gp.pressed("y") and state == "FLYING" and batt > 50:
                    maneuver.start("flip", t.flip_forward)

            # --- output: sticks only while FLYING and not mid-maneuver ---
            if not active:
                if state == "FLYING":
                    cap = MAX_RC
                    if gp.button("rb"):
                        cap = BOOST_RC
                    elif gp.button("lb"):
                        cap = PRECISION_RC
                    ramp = 1.0
                    if fly_since and (t0 - fly_since) < SOFT_START_SEC:
                        ramp = max(0.0, (t0 - fly_since) / SOFT_START_SEC)
                    lx, ly, rx, ry = gp.sticks()   # GC: up=+1, right=+1
                    lr = to_rc(rx, cap, ramp)        # right stick X -> roll
                    fb = to_rc(ry, cap, ramp)        # right stick Y -> pitch (up=fwd)
                    ud = to_rc(ly, MAX_UD, ramp)     # left stick Y  -> throttle (up=climb)
                    yaw = to_rc(lx, MAX_YAW, ramp)   # left stick X  -> yaw (right=cw)
                    t.send_rc_control(lr, fb, ud, yaw)
                    status((lr, fb, ud, yaw), extra="(ramp)" if ramp < 1.0 else "")
                else:
                    t.send_rc_control(0, 0, 0, 0)
                    status((0, 0, 0, 0))
            else:
                status((0, 0, 0, 0), extra=f"({maneuver.name}...)")

            if t0 - last_telem >= 1.0:
                last_telem = t0
                _telem(f"{time.strftime('%H:%M:%S')} state={state} batt={batt}% conn={gp.connected}")

            _sleep_rest(t0, period)

    finally:
        if state in ("FLYING", "LANDING"):
            try:
                t.send_rc_control(0, 0, 0, 0)
            except Exception:
                pass
            if not maneuver.active:
                maneuver.start("land", t.land)
            print("\n[shutdown] landing (bounded)...")
            maneuver.join(FINAL_LAND_TIMEOUT)
            if maneuver.active:
                print("  land link unresponsive -- relying on the Tello's onboard "
                      "command-timeout failsafe to auto-land.")
        try:
            if photos:
                t.streamoff()
        except Exception:
            pass
        try:
            print(f"\nfinal battery={t.get_battery()}%")
        except Exception:
            pass
        gp.close()
    return 0


def _sleep_rest(t0: float, period: float) -> None:
    dt = time.time() - t0
    if dt < period:
        time.sleep(period - dt)


def _snap(t, cap_dir: Path) -> None:
    import cv2
    cap_dir.mkdir(parents=True, exist_ok=True)
    try:
        frame = t.get_frame_read().frame
        if frame is not None and getattr(frame, "size", 0) > 0:
            p = cap_dir / f"shot-{time.strftime('%H%M%S')}.jpg"
            cv2.imwrite(str(p), frame)
            print(f"\n  photo -> {p}")
    except Exception as e:
        print(f"\n  photo error: {e}")


def main() -> int:
    args = sys.argv[1:]
    cmd = args[0] if args else "list"
    if cmd == "list":
        return cmd_list()
    if cmd == "monitor":
        return cmd_monitor(args[1] if len(args) > 1 else "30")
    if cmd == "fly":
        return cmd_fly(photos="--photos" in args)
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
