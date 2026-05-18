"""Phase 1: hover and look. No nav. Rotate 360 in 8x45 steps, VLM-label each frame.

Run on loki (joined to Tello AP):
    uv run python phase1.py [--home-beacon-light Polaris] [--vlm-endpoint URL] [--dry-run]
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time
from typing import Optional

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from lib.hue_beacon import HueBeacon, LIGHT_IDS, KITCHEN_LIGHTS
from lib.pose import Pose
from lib.safety import SafetyMonitor, safe_land
from lib.telemetry import FlightLog

DEFAULT_VLM = "http://100.94.176.110:11434/api/generate"
DEFAULT_MODEL = "gemma4:31b"

LABEL_PROMPT = """You are looking at a frame from a small indoor drone's forward camera.

Two specific light beacons are pre-placed in this scene:
- KITCHEN beacon: two ceiling bulbs set to BRIGHT MAGENTA / VIBRANT PINK.
- HOME beacon: one bulb set to BRIGHT CYAN.

Inspect the frame and answer ONLY in this exact JSON shape, no extra text:
{"magenta": true|false, "cyan": true|false, "note": "<one short sentence>"}

Set "magenta": true only if you can clearly see bright magenta or pink light. Set "cyan": true only if you can clearly see bright cyan light. Otherwise false.
"""


def label_frame(frame_bgr, endpoint: str, model: str = DEFAULT_MODEL, timeout: float = 30.0) -> dict:
    """POST a single frame to the VLM for binary magenta/cyan labels."""
    import cv2  # lazy
    import requests
    ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return {"magenta": False, "cyan": False, "note": "imencode_failed", "error": True}
    b64 = base64.b64encode(buf).decode()
    try:
        r = requests.post(
            endpoint,
            json={
                "model": model,
                "prompt": LABEL_PROMPT,
                "images": [b64],
                "stream": False,
                "format": "json",
                "options": {"temperature": 0.1, "num_predict": 150},
            },
            timeout=timeout,
        )
        r.raise_for_status()
        text = r.json()["response"]
        parsed = json.loads(text)
        return {
            "magenta": bool(parsed.get("magenta", False)),
            "cyan": bool(parsed.get("cyan", False)),
            "note": str(parsed.get("note", "")),
            "raw": parsed,
        }
    except Exception as e:
        return {"magenta": False, "cyan": False, "note": f"vlm_error:{e}", "error": True}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Phase 1: hover and look.")
    p.add_argument("--home-beacon-light", default="Polaris")
    p.add_argument("--vlm-endpoint", default=DEFAULT_VLM)
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args(argv)


def run() -> int:
    ns = parse_args()
    if ns.home_beacon_light not in LIGHT_IDS:
        print(f"unknown home beacon {ns.home_beacon_light!r}; choose from {list(LIGHT_IDS.keys())}")
        return 2

    home_id = LIGHT_IDS[ns.home_beacon_light]
    pose = Pose()
    log = FlightLog(run_id=f"phase1-{time.strftime('%Y%m%d-%H%M%S')}")
    safety = SafetyMonitor(min_battery_takeoff=40, low_battery_land=22)
    log.event("phase1", "begin", home_beacon=ns.home_beacon_light, dry_run=ns.dry_run)

    hue: Optional[HueBeacon] = None
    snap: Optional[dict] = None
    try:
        try:
            hue = HueBeacon()
            snap = hue.snapshot_scene()
            hue.set_kitchen_magenta()
            hue.set_home_cyan(home_id)
            hue.dim_others(except_ids=KITCHEN_LIGHTS + [home_id])
            log.event("phase1", "hue_set")
        except Exception as e:
            log.event("phase1", "hue_setup_failed", err=str(e))

        tello = None
        fr = None
        if not ns.dry_run:
            from djitellopy import Tello
            tello = Tello()
            tello.connect()
            battery = tello.get_battery()
            log.event("phase1", "battery", battery=battery)
            ok, reason = safety.check_pre_takeoff(battery)
            if not ok:
                log.event("phase1", "pre_takeoff_fail", reason=reason)
                print(f"ABORT: {reason}")
                return 1
            safety.install_landing_signal_handler(tello, log)
            try:
                tello.set_speed(20)
            except Exception:
                pass
            tello.streamon()
            time.sleep(3.0)
            fr = tello.get_frame_read()
            for _ in range(8):
                _ = fr.frame
            tello.takeoff()
            time.sleep(4.0)  # extended IMU settle (was 2.0)
            # Skipping move_up(40): Tello default takeoff altitude (~80cm) is enough for Phase 1 rotation.
            # IMU consistency on this drone is currently OK for hover but not translation.
            pose.update_move("up", 80)
        else:
            log.event("phase1", "dry_run_skip_takeoff")

        results: list[dict] = []
        for step in range(8):
            yaw_target = step * 45
            log.event("phase1", "step_begin", step=step, yaw_target=yaw_target)
            if step > 0:
                if not ns.dry_run:
                    tello.rotate_clockwise(45)
                pose.update_move("rotate_cw", 45)
                time.sleep(1.0)

            if ns.dry_run or fr is None:
                frame = None
            else:
                frame = fr.frame

            if frame is None or getattr(frame, "size", 0) == 0:
                log.event("phase1", "no_frame", step=step)
                results.append({"step": step, "yaw_deg": yaw_target, "magenta": False, "cyan": False, "note": "no_frame"})
                continue

            try:
                log.frame("phase1", step, frame)
            except Exception:
                pass

            labels = label_frame(frame, ns.vlm_endpoint)
            log.event("phase1", "labels", step=step, **labels)
            results.append({
                "step": step,
                "yaw_deg": yaw_target,
                "magenta": labels.get("magenta", False),
                "cyan": labels.get("cyan", False),
                "note": labels.get("note", ""),
            })

        if not ns.dry_run and tello is not None:
            safe_land(tello, log, "phase1_normal_end")
            try:
                tello.streamoff()
            except Exception:
                pass

    finally:
        if hue is not None and snap is not None:
            try:
                hue.restore_scene(snap)
                log.event("phase1", "hue_restored")
            except Exception as e:
                log.event("phase1", "hue_restore_failed", err=str(e))
        log.close()

    any_magenta = any(r["magenta"] for r in results)
    any_cyan = any(r["cyan"] for r in results)
    summary_path = os.path.join(str(log.run_dir), "summary.md")
    with open(summary_path, "w") as f:
        f.write(f"# Phase 1 summary\n\nRun: {log.run_id}\n\n")
        for r in results:
            f.write(f"- step {r['step']} (yaw ~{r['yaw_deg']}°): magenta={'YES' if r['magenta'] else 'NO'}, cyan={'YES' if r['cyan'] else 'NO'} — {r.get('note','')}\n")
        f.write(f"\n**PASS:** {'YES' if (any_magenta and any_cyan) else 'NO'} — any_magenta={any_magenta}, any_cyan={any_cyan}\n")

    print()
    print(f"Phase 1 results (see {summary_path}):")
    for r in results:
        print(f"  step {r['step']} (yaw ~{r['yaw_deg']}°): magenta={'YES' if r['magenta'] else 'NO'}, cyan={'YES' if r['cyan'] else 'NO'}")
    print()
    if any_magenta and any_cyan:
        print("PASS: saw magenta AND cyan during sweep.")
        return 0
    print(f"FAIL: any_magenta={any_magenta}, any_cyan={any_cyan}")
    return 1


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
