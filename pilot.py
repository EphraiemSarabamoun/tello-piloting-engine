"""Fly the Tello with an Xbox controller (Mode 2 RC).

    uv run python pilot.py list                 # show detected controllers
    uv run python pilot.py calibrate            # learn/confirm axis+button indices
    uv run python pilot.py fly                  # arm + fly (drone must be on Tello WiFi)
    uv run python pilot.py fly --photos         # same, but enable the camera + photo button

Mode 2 layout (the RC-pilot standard):
    Left stick   vertical   -> throttle (up/down)
    Left stick   horizontal -> yaw (rotate left/right)
    Right stick  vertical   -> pitch (forward/back)
    Right stick  horizontal -> roll (strafe left/right)

Buttons:
    A      takeoff (only from ARMED-on-ground, throttle near center, not settling)
    B      land (graceful) -- this is also the one-button panic stop
    X      photo (only with --photos)
    Y      flip forward (only while flying, battery > 50%)
    Start  arm / disarm toggle on the ground; while FLYING it lands (safe disarm)
    LB     hold = precision speed   RB hold = boost speed
    Back + Start held together (~0.3s) = EMERGENCY motor cut (drone drops -- crash only)

Safety:
    Sticks are inert unless state == FLYING. Takeoff is gated on ARMED + centered
    throttle + not-just-landed. The first ~1.2s after takeoff are throttle-ramped.
    Maneuvers (takeoff/land/flip) run in a background thread so the 30Hz loop NEVER
    blocks -- the EMERGENCY combo stays reachable at every instant, including during
    takeoff and landing.
    Controller disconnect while FLYING -> immediate hover; if it stays gone, auto-land.
    (On the ground a disconnect is safely ignored -- no motors to hover.)
    Battery: warn < 15%, force-land < 10%. Ctrl-C / kill -> graceful land then exit.

This module only reads the controller via pygame/SDL. On macOS that needs a GUI
(Aqua) session: launched over a bare SSH shell, SDL sees zero joysticks. On loki,
run it through `tcc-run` (re-parents into Terminal.app) or in a real Terminal on
loki's display. `calibrate`/`list` will say so loudly if no pad is found.
"""

from __future__ import annotations

import json
import os
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

# Headless-friendly: we never open a window, only the joystick subsystem.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
# Keep reading the pad even when our (dummy) window isn't focused.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import pygame  # noqa: E402

MAP_PATH = Path(os.environ.get("TELLO_PILOT_MAP", str(Path.home() / ".tello_pilot_map.json")))

# ---- tunables -------------------------------------------------------------
LOOP_HZ = 30                 # control + send_rc_control rate
DEADZONE = 0.12              # stick deadzone (fraction)
EXPO = 0.45                  # 0=linear .. 1=very soft center
MAX_RC = 40                  # default roll/pitch cap (gentle indoor)
MAX_UD = 45                  # throttle cap
MAX_YAW = 60                 # yaw-rate cap
BOOST_RC = 80                # roll/pitch cap while RB held
PRECISION_RC = 20            # roll/pitch cap while LB held
TAKEOFF_THROTTLE_GUARD = 0.25  # |left_y| must be under this to allow takeoff
SOFT_START_SEC = 1.2         # ramp RC authority 0->full over this window after takeoff
LAND_SETTLE_SEC = 2.5        # block re-takeoff for this long after a landing completes
FAILSAFE_LAND_AFTER = 1.5    # s of continuous disconnect (while FLYING) -> auto-land
EMERGENCY_HOLD_SEC = 0.3     # Back+Start held this long -> motor cut
RECONNECT_HOLDOFF = 0.25     # ignore button actions for this long after a reconnect
BATT_WARN = 15
BATT_LAND = 10
BATT_POLL_SEC = 3.0
FINAL_LAND_TIMEOUT = 10.0    # bound the shutdown land so we never hang on a dead link


@dataclass
class AxisMap:
    left_x: int = 0
    left_y: int = 1
    right_x: int = 2
    right_y: int = 3
    # signs so that: stick up -> +1, stick right -> +1 (after applying these)
    left_x_sign: float = 1.0
    left_y_sign: float = -1.0
    right_x_sign: float = 1.0
    right_y_sign: float = -1.0


@dataclass
class ButtonMap:
    a: int = 0
    b: int = 1
    x: int = 2
    y: int = 3
    lb: int = 4
    rb: int = 5
    back: int = 6
    start: int = 7


@dataclass
class ControllerMap:
    name: str = "xbox-default"
    axis: AxisMap = field(default_factory=AxisMap)
    button: ButtonMap = field(default_factory=ButtonMap)

    @classmethod
    def load(cls) -> "ControllerMap":
        if MAP_PATH.exists():
            try:
                d = json.loads(MAP_PATH.read_text())
                return cls(
                    name=d.get("name", "learned"),
                    axis=AxisMap(**d["axis"]),
                    button=ButtonMap(**d["button"]),
                )
            except Exception as e:
                print(f"  (ignoring unreadable {MAP_PATH}: {e})")
        return cls()

    def save(self) -> None:
        MAP_PATH.write_text(json.dumps(
            {"name": self.name, "axis": asdict(self.axis), "button": asdict(self.button)},
            indent=2,
        ))
        print(f"  saved mapping -> {MAP_PATH}")


# ---- shaping --------------------------------------------------------------
def deadzone(v: float, dz: float = DEADZONE) -> float:
    """Zero small inputs, then rescale so motion starts at 0 just past dz."""
    if -dz < v < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def expo(v: float, e: float = EXPO) -> float:
    return (1.0 - e) * v + e * (v ** 3)


def to_rc(v: float, cap: int, ramp: float = 1.0) -> int:
    """Shaped stick value (-1..1) -> clamped int RC, scaled by an optional ramp."""
    v = expo(deadzone(v)) * ramp
    return int(max(-cap, min(cap, round(v * cap))))


# ---- maneuver runner ------------------------------------------------------
class Maneuver:
    """Runs one blocking Tello command (takeoff/land/flip) in a daemon thread so
    the 30Hz control loop never stalls and the EMERGENCY combo stays reachable."""

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


# ---- controller -----------------------------------------------------------
class Controller:
    """Thin pygame joystick wrapper with disconnect detection + edge tracking."""

    def __init__(self, cmap: ControllerMap):
        self.cmap = cmap
        self.js: pygame.joystick.JoystickType | None = None
        self.connected = False
        self.disconnected_since: float | None = None
        self.reconnect_holdoff_until = 0.0
        self._prev_buttons: dict[int, bool] = {}

    def open(self) -> bool:
        pygame.event.pump()
        if pygame.joystick.get_count() == 0:
            self.connected = False
            return False
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        was_disconnected = not self.connected
        self.connected = True
        self.disconnected_since = None
        if was_disconnected:
            # Suppress button actions briefly so stabilizing state can't fake a press.
            self.reconnect_holdoff_until = time.time() + RECONNECT_HOLDOFF
            self._prev_buttons.clear()
        return True

    def pump(self) -> None:
        """Drain SDL events; track add/remove so failsafe can react."""
        for ev in pygame.event.get():
            if ev.type == pygame.JOYDEVICEREMOVED:
                self.connected = False
                self.js = None
            elif ev.type == pygame.JOYDEVICEADDED:
                if self.js is None:
                    self.open()
        if not self.connected and self.disconnected_since is None:
            self.disconnected_since = time.time()

    @property
    def buttons_ready(self) -> bool:
        return time.time() >= self.reconnect_holdoff_until

    def axis(self, idx: int, sign: float = 1.0) -> float:
        if not self.connected or self.js is None:
            return 0.0
        try:
            return sign * float(self.js.get_axis(idx))
        except Exception:
            self.connected = False
            self.js = None
            return 0.0

    def button(self, idx: int) -> bool:
        if not self.connected or self.js is None:
            return False
        try:
            return bool(self.js.get_button(idx))
        except Exception:
            self.connected = False
            self.js = None
            return False

    def pressed(self, idx: int) -> bool:
        """True only on the rising edge of a button (debounced press)."""
        now = self.button(idx)
        was = self._prev_buttons.get(idx, False)
        self._prev_buttons[idx] = now
        return now and not was

    def sticks(self) -> tuple[float, float, float, float]:
        a = self.cmap.axis
        return (
            self.axis(a.left_x, a.left_x_sign),
            self.axis(a.left_y, a.left_y_sign),
            self.axis(a.right_x, a.right_x_sign),
            self.axis(a.right_y, a.right_y_sign),
        )


# ---- commands -------------------------------------------------------------
def _init_pygame() -> None:
    pygame.init()
    pygame.joystick.init()


def cmd_list() -> int:
    _init_pygame()
    pygame.event.pump()
    n = pygame.joystick.get_count()
    if n == 0:
        print("No controllers detected.")
        print("On macOS over SSH, SDL sees nothing -- run inside a GUI session")
        print("(loki: via tcc-run or a Terminal on its display). Is the Xbox pad")
        print("paired/connected and powered on?")
        return 1
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"[{i}] {js.get_name()}  axes={js.get_numaxes()} buttons={js.get_numbuttons()} hats={js.get_numhats()}")
    return 0


def cmd_calibrate() -> int:
    """Live-print axis/button changes so we lock the exact indices for this pad."""
    _init_pygame()
    cmap = ControllerMap.load()
    ctl = Controller(cmap)
    if not ctl.open():
        print("No controller found. See `list` notes above.")
        return 1
    print(f"Controller: {ctl.js.get_name()}  axes={ctl.js.get_numaxes()} buttons={ctl.js.get_numbuttons()}")
    print()
    print("Move ONE control at a time. I'll report which index changed.")
    print("Confirm the Mode-2 map: LEFT stick = throttle+yaw, RIGHT stick = pitch+roll.")
    print("Ctrl-C when done (edit ~/.tello_pilot_map.json to override defaults).")
    print()
    last_axis = [0.0] * ctl.js.get_numaxes()
    try:
        while True:
            ctl.pump()
            for i in range(ctl.js.get_numaxes()):
                v = ctl.axis(i)
                if abs(v - last_axis[i]) > 0.18:
                    print(f"  axis {i:2d} = {v:+.2f}")
                    last_axis[i] = v
            for i in range(ctl.js.get_numbuttons()):
                if ctl.pressed(i):
                    print(f"  button {i:2d} pressed")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n  (calibrate done)")
        return 0


def _trigger_land(maneuver: Maneuver, t, why: str) -> None:
    """Neutralize sticks (fire-and-forget) then land in the background thread.

    Landing in a thread means a hung/dead link can't freeze the control loop --
    the loop keeps checking the EMERGENCY combo while the land command runs."""
    try:
        t.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass
    print(f"\n[{why}] landing...")
    if not maneuver.start("land", t.land):
        # A maneuver is already running; nothing else to do.
        pass


def cmd_fly(photos: bool = False) -> int:
    from djitellopy import Tello

    _init_pygame()
    cmap = ControllerMap.load()
    ctl = Controller(cmap)
    if not ctl.open():
        print("No controller found -- cannot fly. See `list` notes.")
        return 1
    print(f"Controller: {ctl.js.get_name()}")

    t = Tello()
    try:
        t.connect()
    except Exception as e:
        print(f"Cannot reach the Tello (192.168.10.1): {e}")
        print("Join the TELLO-XXXXXX WiFi first (drone.py join-wifi).")
        return 1
    batt = t.get_battery()
    print(f"Connected. battery={batt}%")
    if batt < BATT_LAND:
        print("Battery too low to fly. Charge first.")
        return 1
    if photos:
        # Pre-flight only (drone grounded) -- a brief block here is harmless.
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
    print("READY. Press START to ARM. Then A to take off. B to land. Ctrl-C to bail.")
    print("EMERGENCY motor-cut = hold BACK+START together.")
    print()

    period = 1.0 / LOOP_HZ
    last_batt_check = 0.0
    emergency_held_since: float | None = None
    fly_since = 0.0          # set when the takeoff maneuver completes -> drives soft-start ramp
    land_done_at = 0.0       # set when a land maneuver completes -> drives settle gate
    prev_active = False

    def status(rc, extra=""):
        lr, fb, ud, yaw = rc
        conn = "OK " if ctl.connected else "DROP"
        sys.stdout.write(
            f"\r[{state:8s}] batt={batt:3d}% ctl={conn} rc=(lr={lr:+4d} fb={fb:+4d} ud={ud:+4d} yaw={yaw:+4d}) {extra}      "
        )
        sys.stdout.flush()

    try:
        while not stop["flag"]:
            t0 = time.time()
            ctl.pump()
            active = maneuver.active

            # maneuver-completion edge -> arm the soft-start ramp or the land settle
            if prev_active and not active:
                if maneuver.name == "takeoff":
                    fly_since = maneuver.done_at
                elif maneuver.name == "land":
                    land_done_at = maneuver.done_at
            prev_active = active

            # --- EMERGENCY combo: checked EVERY loop, even mid-maneuver ---
            # t.emergency() is fire-and-forget, so it cuts motors even while a
            # takeoff/land thread is still waiting on its 'ok'.
            if ctl.button(cmap.button.back) and ctl.button(cmap.button.start):
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

            # --- failsafe on controller disconnect ---
            if not ctl.connected:
                gone = t0 - (ctl.disconnected_since or t0)
                if state == "FLYING" and not active:
                    t.send_rc_control(0, 0, 0, 0)  # hover
                    if gone >= FAILSAFE_LAND_AFTER:
                        _trigger_land(maneuver, t, "FAILSAFE controller lost")
                        state = "LANDING"
                ctl.open()  # try to re-acquire (sets reconnect holdoff on success)
                status((0, 0, 0, 0), extra="(controller lost)")
                _sleep_rest(t0, period)
                continue

            # --- LANDING -> ARMED once the land finished and the drone settled ---
            if state == "LANDING" and not active and (t0 - land_done_at) >= LAND_SETTLE_SEC:
                state = "ARMED"
                print("\n  -> ARMED (settled)")

            # --- battery watchdog (get_battery is a cheap cached state read) ---
            if t0 - last_batt_check >= BATT_POLL_SEC:
                try:
                    batt = t.get_battery()
                except Exception:
                    pass
                last_batt_check = t0
                if state == "FLYING" and not active and batt <= BATT_LAND:
                    _trigger_land(maneuver, t, f"LOW BATTERY {batt}%")
                    state = "LANDING"

            # --- buttons (edge-triggered; suppressed briefly after a reconnect) ---
            if ctl.buttons_ready and not active:
                if ctl.pressed(cmap.button.start):
                    if state == "FLYING":
                        _trigger_land(maneuver, t, "disarm (START)")
                        state = "LANDING"
                    elif state == "DISARMED":
                        state = "ARMED"
                        print("\n  -> ARMED")
                    elif state == "ARMED":
                        state = "DISARMED"
                        print("\n  -> DISARMED")

                if ctl.pressed(cmap.button.a) and state == "ARMED":
                    _, ly, _, _ = ctl.sticks()
                    if (t0 - land_done_at) < LAND_SETTLE_SEC:
                        print("\n  takeoff blocked: still settling from last landing")
                    elif abs(ly) >= TAKEOFF_THROTTLE_GUARD:
                        print("\n  takeoff blocked: center the throttle (left stick) first")
                    else:
                        print("\n  takeoff...")
                        state = "FLYING"
                        fly_since = 0.0  # ramp starts only once takeoff completes
                        maneuver.start("takeoff", t.takeoff)

                if ctl.pressed(cmap.button.b) and state == "FLYING":
                    _trigger_land(maneuver, t, "land (B)")
                    state = "LANDING"

                if photos and ctl.pressed(cmap.button.x) and state == "FLYING":
                    _snap(t, cap_dir)

                if ctl.pressed(cmap.button.y) and state == "FLYING" and batt > 50:
                    maneuver.start("flip", t.flip_forward)

            # --- output: sticks only while FLYING and not mid-maneuver ---
            if not active:
                if state == "FLYING":
                    cap = MAX_RC
                    if ctl.button(cmap.button.rb):
                        cap = BOOST_RC
                    elif ctl.button(cmap.button.lb):
                        cap = PRECISION_RC
                    # soft-start: ramp authority 0->1 over the first SOFT_START_SEC of flight
                    ramp = 1.0
                    if fly_since and (t0 - fly_since) < SOFT_START_SEC:
                        ramp = max(0.0, (t0 - fly_since) / SOFT_START_SEC)
                    lx, ly, rx, ry = ctl.sticks()
                    lr = to_rc(rx, cap, ramp)        # right stick X -> roll
                    fb = to_rc(ry, cap, ramp)        # right stick Y -> pitch (up=fwd)
                    ud = to_rc(ly, MAX_UD, ramp)     # left stick Y  -> throttle
                    yaw = to_rc(lx, MAX_YAW, ramp)   # left stick X  -> yaw
                    t.send_rc_control(lr, fb, ud, yaw)
                    status((lr, fb, ud, yaw), extra="(ramp)" if ramp < 1.0 else "")
                else:
                    t.send_rc_control(0, 0, 0, 0)    # keep the link alive, neutral
                    status((0, 0, 0, 0))
            else:
                status((0, 0, 0, 0), extra=f"({maneuver.name}...)")

            _sleep_rest(t0, period)

    finally:
        # If still airborne, land -- but bound it so a dead link can't hang us.
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
        pygame.quit()
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
    if cmd == "calibrate":
        return cmd_calibrate()
    if cmd == "fly":
        return cmd_fly(photos="--photos" in args)
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
