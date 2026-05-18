"""Simplified kitchen mission: fly forward to magenta beacon, hover, fly back to cyan, land.

Uses send_rc_control continuously (no SDK auto-hover) with strong forward/back bias.
Snapshots periodically + VLM checks "are we there yet?". No rotation, no goal streaks beyond
two consecutive YES, no ascent state — just relentless forward motion until magenta dominates
the frame, then relentless backward motion until cyan dominates.

Run on loki (joined to Tello AP):
    .venv/bin/python kitchen_simple.py [--home Sirius] [--vlm-endpoint URL]
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import signal
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import requests
from djitellopy import Tello

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from lib.hue_beacon import HueBeacon, LIGHT_IDS, BEDROOM_LIGHTS, KITCHEN_LIGHTS

MAGENTA_XY = (0.41, 0.17)
CYAN_XY = (0.17, 0.30)

NAPOLEON = "http://100.94.176.110:11434/api/generate"
NAPOLEON_MODEL = "moondream:latest"  # 1.7B vision model; ~110ms warm on RTX 5090, 100% correct on kitchen frame at 480x360
ANTHROPIC_API = "https://api.anthropic.com/v1/messages"
ANTHROPIC_MODEL = "claude-haiku-4-5"

LOOP_HZ = 20
PERIOD = 1.0 / LOOP_HZ

CLIMB_SEC = 3.0       # extra height per Viro; ~80cm takeoff + 3.0s * 50 = ~230cm
CLIMB_VZ = 50

OUTBOUND_FB_DEFAULT = 50  # slower forward per Viro; less overshoot risk, more time for color/VLM
OUTBOUND_TIMEOUT_SEC = 15.0   # tight — Viro observed overshoot at 60s; kitchen reached well before

KITCHEN_HOVER_SEC = 3.0

RETURN_FB_DEFAULT = 50    # slower forward to match outbound
RETURN_TIMEOUT_SEC = 20.0

# Rotation at kitchen: turn drone 180° so forward camera faces the home beacon.
TURN_YAW_RATE = 50        # deg/sec, positive = clockwise
TURN_DURATION_SEC = 4.5   # 4.5s * 50 = 225° commanded; with spin-up/spin-down losses lands near 180° actual
TURN_SETTLE_SEC = 1.0

DESCENT_SEC = 1.5
DESCENT_VZ = -45

SNAP_EVERY_SEC = 0.1       # 10Hz with moondream at ~110ms; pool absorbs the latency
GOAL_STREAK = 2            # 2-in-a-row required; with async dispatch, this is single-flight robustness
MIN_OUTBOUND_DURATION_SEC = 3.0   # reached signal ignored before this — drone must fly forward at least this long
MIN_RETURN_DURATION_SEC = 5.0     # same for return — after 180° turn drone still sees bedroom cyan from kitchen

# Velocity-ramp thresholds: color HSV pixel count drives "approach" and "reached" decisions.
APPROACH_HSV_THRESHOLD = 2250   # halfway between the too-soft 3000 and too-aggressive 1500
APPROACH_FB_SCALE = 0.2         # multiply OUTBOUND_FB by this when approaching (80 → 16)
HARD_REACHED_THRESHOLD = 16000  # halfway between too-soft 20000 and too-aggressive 12000
VLM_RESIZE = (480, 360)         # downsample frames sent to moondream; preserves accuracy, cuts payload 3x

MIN_BATTERY = 20
LAND_BATTERY = 15

OUTBOUND_PROMPT = """Drone forward-camera frame. Has the drone ARRIVED at the kitchen?
reached=true ONLY if bright magenta/pink light DOMINATES at least one third of the central region of this frame.
reached=false otherwise. If magenta is only visible as a distant glow, edge tint, or small region: false.
Reply ONLY: {"reached": true|false, "note": "<short>"}"""

RETURN_PROMPT = """Drone forward-camera frame. Has the drone ARRIVED back at the bedroom?
reached=true ONLY if bright cyan light DOMINATES at least one third of the central region of this frame.
reached=false otherwise. If cyan is only visible as a distant glow, edge tint, or small region: false.
Reply ONLY: {"reached": true|false, "note": "<short>"}"""


def _load_anthropic_key() -> str | None:
    k = os.environ.get("ANTHROPIC_API_KEY")
    if k:
        return k
    # Fallback: read from ~/.zshrc.secrets
    try:
        with open(os.path.expanduser("~/.zshrc.secrets")) as f:
            for line in f:
                if "ANTHROPIC_API_KEY" in line and "export" in line:
                    return line.split("=", 1)[1].strip().strip('"').strip("'")
    except Exception:
        pass
    return None


_ANTHROPIC_KEY = _load_anthropic_key()


def ask_vlm_ollama(frame_bgr, prompt: str, timeout: float = 15.0) -> dict:
    # Downsample to VLM_RESIZE before encode — keeps moondream accurate, cuts payload 3x.
    small = cv2.resize(frame_bgr, VLM_RESIZE, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return {"reached": False, "note": "imencode_fail"}
    b64 = base64.b64encode(buf).decode()
    try:
        r = requests.post(NAPOLEON, json={
            "model": NAPOLEON_MODEL,
            "prompt": prompt,
            "images": [b64],
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1, "num_predict": 30},
        }, timeout=timeout)
        r.raise_for_status()
        data = json.loads(r.json()["response"])
        return {"reached": bool(data.get("reached", False)), "note": str(data.get("note", ""))}
    except Exception as e:
        return {"reached": False, "note": f"vlm_err:{e}"}


def ask_vlm_claude(frame_bgr, prompt: str, timeout: float = 15.0) -> dict:
    if _ANTHROPIC_KEY is None:
        return {"reached": False, "note": "no_anthropic_key"}
    ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return {"reached": False, "note": "imencode_fail"}
    b64 = base64.b64encode(buf).decode()
    try:
        r = requests.post(
            ANTHROPIC_API,
            headers={
                "x-api-key": _ANTHROPIC_KEY,
                "anthropic-version": "2023-06-01",
                "content-type": "application/json",
            },
            json={
                "model": ANTHROPIC_MODEL,
                "max_tokens": 150,
                "messages": [{
                    "role": "user",
                    "content": [
                        {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": b64}},
                        {"type": "text", "text": prompt + "\n\nReply ONLY with the JSON object, no markdown fences."},
                    ],
                }],
            },
            timeout=timeout,
        )
        r.raise_for_status()
        text = r.json()["content"][0]["text"].strip()
        if text.startswith("```"):
            text = text.split("```")[1].lstrip("json").strip()
        data = json.loads(text)
        return {"reached": bool(data.get("reached", False)), "note": str(data.get("note", ""))}
    except Exception as e:
        return {"reached": False, "note": f"vlm_err:{e}"}


import numpy as np

# HSV thresholds for 960x720 Tello stream (=691200 px total). OpenCV HSV: H 0-179, S 0-255, V 0-255.
# Magenta/pink: H wraps around 0/180. Use union of [140-180] and [0-10] with sat>80, val>80.
# Cyan: H around 85-100. Same sat/val floor.
#
# Tuned 2026-05-14 against 119 real flight frames in /tmp/flight-frames/ from 9 missions.
# Ground truth from Ephraiem:
#   - tello-ksimple-150407 (overshot): snap2/3 are deep in kitchen. HSV magenta = 226k, 359k.
#   - tello-ksimple-150659 (reached at snap5): HSV magenta = 35k at snap5, 65k at snap7.
#   - tello-ksimple-152813 (never reached): peak HSV magenta = 13.5k across 66 frames.
#   - tello-phase1rc-130617 (yaw sweep): bedroom direction (yaw 180-315) shows HSV cyan = 122k-342k.
# Threshold 25000 fires correctly on all true-positives, never on true-negatives, with
# >10k margin above the 152813 peak and 10k below the 150659 ground-truth first-arrival.
COLOR_REACHED_PIXEL_THRESHOLD = 25000

# HSV band constants. inRange takes uint8 arrays; pre-build them once for speed.
_HSV_MG_LOW_HI = (np.array((140, 80, 80), dtype=np.uint8), np.array((180, 255, 255), dtype=np.uint8))
_HSV_MG_LOW_WRAP = (np.array((0, 80, 80), dtype=np.uint8), np.array((10, 255, 255), dtype=np.uint8))
_HSV_CY = (np.array((85, 80, 80), dtype=np.uint8), np.array((100, 255, 255), dtype=np.uint8))


def _count_color_pixels(frame_bgr, target: str) -> int:
    """HSV color count. target in {'magenta', 'cyan'}. Returns pixel count.

    Magenta uses a hue-wrap union (140-180 OR 0-10). Cyan is single-band (85-100).
    Saturation > 80 and Value > 80 filter out white/gray/black auto-WB artifacts that
    bit the previous BGR detector.
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if target == "magenta":
        m1 = cv2.inRange(hsv, _HSV_MG_LOW_HI[0], _HSV_MG_LOW_HI[1])
        m2 = cv2.inRange(hsv, _HSV_MG_LOW_WRAP[0], _HSV_MG_LOW_WRAP[1])
        mask = cv2.bitwise_or(m1, m2)
    elif target == "cyan":
        mask = cv2.inRange(hsv, _HSV_CY[0], _HSV_CY[1])
    else:
        return 0
    return int(mask.sum() // 255)


def _color_debug_stats(frame_bgr, target: str) -> dict:
    """Diagnostic hue/sat/val stats for the matched pixels. ~5ms; only call when --color-debug."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if target == "magenta":
        m1 = cv2.inRange(hsv, _HSV_MG_LOW_HI[0], _HSV_MG_LOW_HI[1])
        m2 = cv2.inRange(hsv, _HSV_MG_LOW_WRAP[0], _HSV_MG_LOW_WRAP[1])
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv, _HSV_CY[0], _HSV_CY[1])
    n = int(mask.sum() // 255)
    if n == 0:
        return {"n": 0, "h_mean": None, "s_mean": None, "v_mean": None}
    sel = hsv[mask.astype(bool)]
    return {
        "n": n,
        "h_mean": float(sel[:, 0].mean()),
        "s_mean": float(sel[:, 1].mean()),
        "v_mean": float(sel[:, 2].mean()),
    }


def ask_vlm_color(frame_bgr, prompt: str, timeout: float = 1.0, threshold: int | None = None, debug: bool = False) -> dict:
    """Sub-millisecond HSV color detection. 'prompt' encodes which color we're looking for via keyword."""
    target = "magenta" if "MAGENTA" in prompt or "magenta" in prompt[:200].lower() else "cyan"
    thresh = COLOR_REACHED_PIXEL_THRESHOLD if threshold is None else threshold
    if debug:
        stats = _color_debug_stats(frame_bgr, target)
        n = stats["n"]
        h, s, v = stats["h_mean"], stats["s_mean"], stats["v_mean"]
        hsv_tag = f" h={h:.0f} s={s:.0f} v={v:.0f}" if h is not None else ""
        return {"reached": n >= thresh, "note": f"{target}_px={n} thresh={thresh}{hsv_tag}"}
    n = _count_color_pixels(frame_bgr, target)
    return {"reached": n >= thresh, "note": f"{target}_px={n} thresh={thresh}"}


def ask_vlm(frame_bgr, prompt: str, backend: str = "color", timeout: float = 15.0,
            color_threshold: int | None = None, color_debug: bool = False) -> dict:
    if backend == "color":
        return ask_vlm_color(frame_bgr, prompt, timeout, threshold=color_threshold, debug=color_debug)
    if backend == "ollama":
        return ask_vlm_ollama(frame_bgr, prompt, timeout)
    return ask_vlm_claude(frame_bgr, prompt, timeout)


def setup_beacons(hue: HueBeacon) -> None:
    for lid in KITCHEN_LIGHTS:
        try: hue.set_light(lid, True, 100.0, xy=MAGENTA_XY)
        except Exception as e: print(f"  hue mg fail {lid}: {e}")
    for lid in BEDROOM_LIGHTS:
        try: hue.set_light(lid, True, 100.0, xy=CYAN_XY)
        except Exception as e: print(f"  hue cy fail {lid}: {e}")


def run(args: argparse.Namespace) -> int:
    # CLI overrides for drift compensation. Positive lr = right, negative lr = left.
    LR_BIAS = int(args.lr_bias)
    FB_HOVER_BIAS = int(args.fb_hover_bias)  # added to fb during hover/climb/descent only
    OUTBOUND_FB = OUTBOUND_FB_DEFAULT
    RETURN_FB = RETURN_FB_DEFAULT
    print(f"flight-tuning: lr_bias={LR_BIAS} fb_hover_bias={FB_HOVER_BIAS} outbound_fb={OUTBOUND_FB} return_fb={RETURN_FB}")
    out_dir = Path.home() / "captures" / time.strftime("%Y-%m-%d") / f"tello-ksimple-{time.strftime('%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "kitchen_simple.jsonl"
    log_f = open(log_path, "w")

    def log(phase: str, kind: str, **fields):
        log_f.write(json.dumps({"t": time.time(), "phase": phase, "kind": kind, **fields}) + "\n")
        log_f.flush()

    print(f"out_dir={out_dir}")
    log("init", "begin", args=vars(args))

    import urllib3; urllib3.disable_warnings()
    hue = HueBeacon()
    snap = hue.snapshot_scene()
    setup_beacons(hue)
    print("hue beacons set: kitchen=MAGENTA bedroom=CYAN")

    t = Tello()
    t.connect()
    bat = t.get_battery()
    print(f"pre-flight battery={bat}%")
    log("init", "battery", battery=bat)
    if bat < MIN_BATTERY:
        print(f"battery too low (<{MIN_BATTERY}%)")
        hue.restore_scene(snap)
        return 1

    aborted = False
    abort_reason = None

    def safe_land():
        try:
            t.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            t.land()
        except Exception as e:
            print(f"land err {e} — emergency")
            try: t.emergency()
            except Exception: pass

    def sig(*_):
        nonlocal aborted, abort_reason
        aborted = True
        abort_reason = "sigint"
        print("\n[signal] aborting")
        safe_land()
        sys.exit(1)
    signal.signal(signal.SIGINT, sig)
    signal.signal(signal.SIGTERM, sig)

    try:
        t.set_speed(20)
        t.streamon()
        time.sleep(3.0)
        fr = t.get_frame_read()
        for _ in range(8):
            _ = fr.frame
        log("init", "stream_ready")

        t.takeoff()
        log("armed", "takeoff_ok")
        time.sleep(0.5)

        # CLIMB
        log("climb", "begin")
        cs = time.time()
        while time.time() - cs < CLIMB_SEC and not aborted:
            t.send_rc_control(LR_BIAS, FB_HOVER_BIAS, CLIMB_VZ, 0)
            time.sleep(PERIOD)

        # OUTBOUND with async VLM + velocity ramp + 2-streak confirmation
        log("outbound", "begin")
        print("\n=== OUTBOUND (toward magenta kitchen) ===")
        ob_start = time.time()
        last_snap = 0.0
        snap_idx = 0
        reached = False
        yes_streak = 0
        peak_color_count = 0
        current_fb = OUTBOUND_FB  # may be scaled down on approach
        executor = ThreadPoolExecutor(max_workers=6)
        pending: list = []

        while time.time() - ob_start < OUTBOUND_TIMEOUT_SEC and not aborted:
            t.send_rc_control(LR_BIAS, current_fb, 0, 0)
            elapsed = time.time() - ob_start
            bat = t.get_battery()
            if bat < LAND_BATTERY:
                abort_reason = f"battery {bat}% < {LAND_BATTERY}%"
                aborted = True
                log("outbound", "low_battery", battery=bat)
                break
            # Drain completed futures (preserve submission order — sort by idx then check sequential YES)
            still_pending = []
            done_results = []
            for idx, snap_t, fut in pending:
                if fut.done():
                    done_results.append((idx, snap_t, fut.result()))
                else:
                    still_pending.append((idx, snap_t, fut))
            pending = still_pending
            done_results.sort(key=lambda x: x[0])
            for idx, snap_t, result in done_results:
                print(f"  [t={elapsed:.1f}s] OB snap{idx:03d} (snap@{snap_t:.1f}s) reached={result['reached']} | {result['note'][:60]}")
                log("outbound", "snap_result", idx=idx, snap_elapsed=snap_t, returned_elapsed=elapsed, **result)
                if result["reached"]:
                    yes_streak += 1
                    if yes_streak >= GOAL_STREAK:
                        reached = True
                else:
                    yes_streak = 0
            if reached:
                log("outbound", "goal_reached", elapsed=elapsed, peak_color=peak_color_count)
                print(f"  GOAL_REACHED (streak={yes_streak})")
                break
            # Snap + always-on color detection (free, sub-ms) + async VLM dispatch
            if elapsed - last_snap >= SNAP_EVERY_SEC:
                frame = fr.frame
                if frame is not None and frame.size > 0:
                    snap_idx += 1
                    frame_copy = frame.copy()
                    p = out_dir / f"outbound_{snap_idx:03d}.jpg"
                    cv2.imwrite(str(p), frame_copy)
                    # Sub-ms color signal for velocity ramp (regardless of selected backend)
                    color_count = _count_color_pixels(frame_copy, "magenta")
                    if color_count > peak_color_count:
                        peak_color_count = color_count
                    # Hard color trigger: if color massively dominates, stop NOW, no VLM wait.
                    # Gated by minimum duration so drone can't insta-stop from takeoff glow.
                    if color_count >= HARD_REACHED_THRESHOLD and elapsed >= MIN_OUTBOUND_DURATION_SEC:
                        print(f"  [hard-reach] color={color_count} >= {HARD_REACHED_THRESHOLD} → REACHED")
                        log("outbound", "hard_reached", color=color_count, elapsed=elapsed)
                        reached = True
                    # Velocity ramp: slow down when approaching
                    if color_count >= APPROACH_HSV_THRESHOLD:
                        new_fb = int(OUTBOUND_FB * APPROACH_FB_SCALE)
                        if new_fb != current_fb:
                            print(f"  [ramp] color={color_count} → slowing fb {current_fb}→{new_fb}")
                            log("outbound", "velocity_ramp", color=color_count, fb_old=current_fb, fb_new=new_fb)
                        current_fb = new_fb
                    # Gate VLM: only dispatch when color says we're approaching.
                    if color_count >= APPROACH_HSV_THRESHOLD or args.backend == "color":
                        fut = executor.submit(ask_vlm, frame_copy, OUTBOUND_PROMPT, args.backend, 15.0, args.color_threshold, args.color_debug)
                        pending.append((snap_idx, elapsed, fut))
                        log("outbound", "snap_dispatched", idx=snap_idx, elapsed=elapsed, battery=bat, color=color_count, fb=current_fb)
                    else:
                        log("outbound", "snap_skipped_vlm", idx=snap_idx, elapsed=elapsed, color=color_count, reason="below_approach_threshold")
                last_snap = elapsed
            time.sleep(PERIOD)
        executor.shutdown(wait=False)

        # KITCHEN_HOVER (or already aborting)
        if reached and not aborted:
            print("\n=== KITCHEN_HOVER ===")
            log("kitchen_hover", "begin")
            kh_start = time.time()
            while time.time() - kh_start < KITCHEN_HOVER_SEC:
                t.send_rc_control(LR_BIAS, FB_HOVER_BIAS, 0, 0)
                time.sleep(PERIOD)
            frame = fr.frame
            if frame is not None and frame.size > 0:
                cv2.imwrite(str(out_dir / "kitchen_confirm.jpg"), frame)
                log("kitchen_hover", "confirm_snap")
            print("<!-- TTS: \"Kitchen reached.\" -->")

            outbound_duration = elapsed  # for context in logs

            # TURN_180: rotate drone in place so forward camera faces home (cyan bedroom).
            print(f"\n=== TURN_180 ({TURN_DURATION_SEC}s @ {TURN_YAW_RATE}°/s CW) ===")
            log("turn_180", "begin", yaw_rate=TURN_YAW_RATE, duration=TURN_DURATION_SEC)
            tn_start = time.time()
            while time.time() - tn_start < TURN_DURATION_SEC and not aborted:
                t.send_rc_control(0, 0, 0, TURN_YAW_RATE)
                time.sleep(PERIOD)
            # Settle after rotation
            ts_start = time.time()
            while time.time() - ts_start < TURN_SETTLE_SEC and not aborted:
                t.send_rc_control(LR_BIAS, FB_HOVER_BIAS, 0, 0)
                time.sleep(PERIOD)
            # Snap post-turn frame
            frame = fr.frame
            if frame is not None and frame.size > 0:
                cv2.imwrite(str(out_dir / "post_turn_180.jpg"), frame.copy())
                log("turn_180", "post_turn_snap")
            log("turn_180", "complete")

            # RETURN (post-180): drone now faces home. Fly FORWARD toward cyan bedroom beacon.
            # Same loop shape as OUTBOUND: color-gated VLM dispatch, velocity ramp, 2-streak.
            print(f"\n=== RETURN (forward toward cyan home, post-180) ===")
            log("return", "begin")
            rt_start = time.time()
            last_snap = 0.0
            snap_idx = 0
            home = False
            yes_streak_r = 0
            peak_color_count_r = 0
            current_fb_r = RETURN_FB
            executor = ThreadPoolExecutor(max_workers=6)
            pending = []
            while time.time() - rt_start < RETURN_TIMEOUT_SEC and not aborted:
                t.send_rc_control(LR_BIAS, current_fb_r, 0, 0)
                elapsed = time.time() - rt_start
                bat = t.get_battery()
                if bat < LAND_BATTERY:
                    abort_reason = f"battery {bat}% < {LAND_BATTERY}%"
                    aborted = True
                    log("return", "low_battery", battery=bat)
                    break
                still_pending = []
                done_results = []
                for idx, snap_t, fut in pending:
                    if fut.done():
                        done_results.append((idx, snap_t, fut.result()))
                    else:
                        still_pending.append((idx, snap_t, fut))
                pending = still_pending
                done_results.sort(key=lambda x: x[0])
                for idx, snap_t, result in done_results:
                    print(f"  [t={elapsed:.1f}s] RT snap{idx:03d} (snap@{snap_t:.1f}s) reached={result['reached']} | {result['note'][:60]}")
                    log("return", "snap_result", idx=idx, snap_elapsed=snap_t, returned_elapsed=elapsed, **result)
                    if snap_t < MIN_RETURN_DURATION_SEC:
                        continue
                    if result["reached"]:
                        yes_streak_r += 1
                        if yes_streak_r >= GOAL_STREAK:
                            home = True
                    else:
                        yes_streak_r = 0
                if home:
                    log("return", "home_reached", elapsed=elapsed, peak_color=peak_color_count_r)
                    print(f"  HOME_REACHED (streak={yes_streak_r})")
                    break
                if elapsed - last_snap >= SNAP_EVERY_SEC:
                    frame = fr.frame
                    if frame is not None and frame.size > 0:
                        snap_idx += 1
                        frame_copy = frame.copy()
                        cv2.imwrite(str(out_dir / f"return_{snap_idx:03d}.jpg"), frame_copy)
                        color_count_r = _count_color_pixels(frame_copy, "cyan")
                        if color_count_r > peak_color_count_r:
                            peak_color_count_r = color_count_r
                        # Hard color trigger for cyan home beacon (gated by min duration)
                        if color_count_r >= HARD_REACHED_THRESHOLD and elapsed >= MIN_RETURN_DURATION_SEC:
                            print(f"  [hard-reach] cyan={color_count_r} >= {HARD_REACHED_THRESHOLD} → HOME")
                            log("return", "hard_reached", color=color_count_r, elapsed=elapsed)
                            home = True
                        if color_count_r >= APPROACH_HSV_THRESHOLD:
                            new_fb = int(RETURN_FB * APPROACH_FB_SCALE)
                            if new_fb != current_fb_r:
                                print(f"  [ramp] cyan={color_count_r} → slowing fb {current_fb_r}→{new_fb}")
                                log("return", "velocity_ramp", color=color_count_r, fb_old=current_fb_r, fb_new=new_fb)
                            current_fb_r = new_fb
                        if color_count_r >= APPROACH_HSV_THRESHOLD or args.backend == "color":
                            fut = executor.submit(ask_vlm, frame_copy, RETURN_PROMPT, args.backend, 15.0, args.color_threshold, args.color_debug)
                            pending.append((snap_idx, elapsed, fut))
                            log("return", "snap_dispatched", idx=snap_idx, elapsed=elapsed, battery=bat, color=color_count_r, fb=current_fb_r)
                        else:
                            log("return", "snap_skipped_vlm", idx=snap_idx, elapsed=elapsed, color=color_count_r, reason="below_approach_threshold")
                    last_snap = elapsed
                time.sleep(PERIOD)
            executor.shutdown(wait=False)
            if not home:
                log("return", "timeout", elapsed=elapsed, peak_color=peak_color_count_r)
                print(f"  RETURN_TIMEOUT after {elapsed:.1f}s (peak cyan={peak_color_count_r})")

            # DESCENT
            print("\n=== DESCENT ===")
            log("descent", "begin")
            ds = time.time()
            while time.time() - ds < DESCENT_SEC and not aborted:
                t.send_rc_control(LR_BIAS, FB_HOVER_BIAS, DESCENT_VZ, 0)
                time.sleep(PERIOD)

        # LAND
        print("\n=== LAND ===")
        log("landing", "begin", abort_reason=abort_reason)
        safe_land()
        time.sleep(2)
        try:
            post_bat = t.get_battery()
            print(f"post-flight battery={post_bat}%")
            log("landing", "complete", battery=post_bat)
        except Exception:
            pass

    finally:
        try: t.streamoff()
        except Exception: pass
        try:
            hue.restore_scene(snap)
            log("post_flight", "hue_restored")
            print("hue restored")
        except Exception as e:
            print(f"hue restore err: {e}")
        log_f.close()

    print(f"\nlog: {log_path}")
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--home", default="Sirius")
    p.add_argument("--vlm-endpoint", default=NAPOLEON)
    p.add_argument("--lr-bias", type=int, default=8, help="Per-flight lateral drift correction. +ve = right (counter left drift).")
    p.add_argument("--fb-hover-bias", type=int, default=15, help="Forward bias during hover/climb/descent (counters backward drift).")
    p.add_argument("--backend", choices=["color", "claude", "ollama"], default="ollama", help="Backend: ollama=napoleon moondream (~110ms, default), color=HSV pixel-count (<1ms), claude=Anthropic API (~1.5s)")
    p.add_argument("--color-threshold", type=int, default=None,
                   help=f"Override HSV pixel-count threshold for 'reached' (default {COLOR_REACHED_PIXEL_THRESHOLD}).")
    p.add_argument("--color-debug", action="store_true",
                   help="Color backend: log per-call hue/sat/val means of the matched pixels.")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.vlm_endpoint != NAPOLEON:
        NAPOLEON = args.vlm_endpoint
    sys.exit(run(args))
