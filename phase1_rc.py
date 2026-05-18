"""Phase 1 (rc-control v2): takeoff, climb to 1.75m, hover + rotate 360, snap, descend, land.

Hue test config (per Viro):
- Kitchen lights (Vega + Capella) -> bright MAGENTA / pink
- All bedroom lights (Sirius, Betelgeuse, Rigel, Altair, Antares, Deneb, Polaris) -> bright CYAN

Uses send_rc_control at 20Hz so the drone is actively driven the entire flight.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import cv2
from djitellopy import Tello

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from lib.hue_beacon import HueBeacon, LIGHT_IDS, BEDROOM_LIGHTS, KITCHEN_LIGHTS

MAGENTA_XY = (0.41, 0.17)   # Hue "pink" preset
CYAN_XY = (0.17, 0.30)

LOOP_HZ = 20
SETTLE_SEC = 2.0
CLIMB_SEC = 2.5          # rc up @ ~40cm/s -> +100cm; combined with ~80cm takeoff = ~1.75m
CLIMB_VZ = 55            # +55 rc throttle
HOVER_BEFORE_ROTATE_SEC = 1.5
ROTATE_DURATION_SEC = 10.0
ROTATE_YAW_RATE = 36     # 36 deg/s -> 360 in 10s
HOVER_AFTER_ROTATE_SEC = 1.0
DESCENT_SEC = 1.5
DESCENT_VZ = -45
FWD_BIAS = 20            # counter the observed backward drift (+fb during hover phases) — doubled per Viro 13:18

SNAPSHOT_YAWS = [0, 45, 90, 135, 180, 225, 270, 315]
OUT_DIR = Path.home() / "captures" / time.strftime("%Y-%m-%d") / f"tello-phase1rc2-{time.strftime('%H%M%S')}"

NAPOLEON = "http://100.94.176.110:11434/api/generate"
MODEL = "gemma4:31b"
LABEL_PROMPT = """You are looking at a frame from a small indoor drone's forward camera.

In this scene, ONE specific light is set to bright MAGENTA or PINK (it's a wall-mounted LED lightstrip).
EVERY other light in view is set to bright CYAN.

Reply ONLY in this exact JSON shape:
{"magenta": true|false, "cyan": true|false, "note": "<one short sentence>"}

magenta=true only if you can clearly see a bright magenta/pink light source.
cyan=true only if you can clearly see one or more bright cyan light sources.
"""


def label_frame(frame_bgr) -> dict:
    import base64
    import requests
    ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return {"magenta": False, "cyan": False, "note": "imencode_failed"}
    b64 = base64.b64encode(buf).decode()
    try:
        r = requests.post(NAPOLEON, json={
            "model": MODEL,
            "prompt": LABEL_PROMPT,
            "images": [b64],
            "stream": False,
            "format": "json",
            "options": {"temperature": 0.1, "num_predict": 150},
        }, timeout=30)
        r.raise_for_status()
        text = r.json()["response"]
        parsed = json.loads(text)
        return {
            "magenta": bool(parsed.get("magenta", False)),
            "cyan": bool(parsed.get("cyan", False)),
            "note": str(parsed.get("note", "")),
        }
    except Exception as e:
        return {"magenta": False, "cyan": False, "note": f"vlm_err:{e}"}


def setup_hue_test_beacons(hue: HueBeacon) -> None:
    """Kitchen Vega + Capella -> magenta. Bedroom (all 7) -> bright cyan."""
    for lid in KITCHEN_LIGHTS:
        try:
            hue.set_light(lid, True, 100.0, xy=MAGENTA_XY)
        except Exception as e:
            print(f"  hue set magenta failed for {lid}: {e}")
    for lid in BEDROOM_LIGHTS:
        try:
            hue.set_light(lid, True, 100.0, xy=CYAN_XY)
        except Exception as e:
            print(f"  hue set cyan failed for {lid}: {e}")


def verify_hue_state(hue: HueBeacon) -> None:
    """Read back light state to confirm the PUT actually landed."""
    state = hue.snapshot_scene()
    for name, lid in LIGHT_IDS.items():
        s = state.get(lid, {})
        xy = s.get("xy")
        bri = s.get("brightness")
        is_kitchen = lid in KITCHEN_LIGHTS
        target = MAGENTA_XY if is_kitchen else CYAN_XY
        room = "KITCHEN" if is_kitchen else "BEDROOM"
        if xy is not None:
            dx = abs(xy["x"] - target[0])
            dy = abs(xy["y"] - target[1])
            tag = "OK" if (dx < 0.05 and dy < 0.05) else "MISMATCH"
        else:
            tag = "NO_XY"
        print(f"  {name:11s} {room:7s} bri={bri} xy={xy} -> {tag}")


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"out_dir={OUT_DIR}")

    import urllib3
    urllib3.disable_warnings()
    hue = HueBeacon()
    snap = hue.snapshot_scene()
    print("hue snapshot captured")
    setup_hue_test_beacons(hue)
    print("hue beacons set; verifying:")
    verify_hue_state(hue)

    t = Tello()
    t.connect()
    bat = t.get_battery()
    print(f"pre-flight battery={bat}%")
    if bat < 30:
        print("battery too low")
        hue.restore_scene(snap)
        return 1

    snap_results: list[dict] = []

    try:
        t.set_speed(20)
        t.streamon()
        time.sleep(3.0)
        fr = t.get_frame_read()
        for _ in range(8):
            _ = fr.frame
        print("video stream ready")

        t.takeoff()
        period = 1.0 / LOOP_HZ
        loop_start = time.time()

        total_duration = SETTLE_SEC + CLIMB_SEC + HOVER_BEFORE_ROTATE_SEC + ROTATE_DURATION_SEC + HOVER_AFTER_ROTATE_SEC + DESCENT_SEC
        rotate_start = SETTLE_SEC + CLIMB_SEC + HOVER_BEFORE_ROTATE_SEC
        rotate_end = rotate_start + ROTATE_DURATION_SEC
        descent_start = rotate_end + HOVER_AFTER_ROTATE_SEC

        snap_done: set[int] = set()
        print(f"airborne, total {total_duration:.1f}s")

        while True:
            t0 = time.time()
            elapsed = t0 - loop_start
            if elapsed >= total_duration:
                break

            if elapsed < SETTLE_SEC:
                t.send_rc_control(0, FWD_BIAS, 0, 0)
                phase_tag = "settle"
            elif elapsed < SETTLE_SEC + CLIMB_SEC:
                t.send_rc_control(0, FWD_BIAS, CLIMB_VZ, 0)
                phase_tag = "climb"
            elif elapsed < rotate_start:
                t.send_rc_control(0, FWD_BIAS, 0, 0)
                phase_tag = "pre_rotate_hover"
                if 0 not in snap_done and elapsed > rotate_start - 0.4:
                    frame = fr.frame
                    if frame is not None and frame.size > 0:
                        path = OUT_DIR / "yaw_000.jpg"
                        cv2.imwrite(str(path), frame)
                        snap_done.add(0)
                        snap_results.append({"yaw_deg": 0, "frame_path": str(path), "t_elapsed": elapsed})
                        print(f"  [t={elapsed:.1f}s {phase_tag}] snapped yaw=0")
            elif elapsed < rotate_end:
                # NOTE: skipping forward bias during rotation — yaw + fb would curve the path.
                t.send_rc_control(0, 0, 0, ROTATE_YAW_RATE)
                phase_tag = "rotate"
                rot_elapsed = elapsed - rotate_start
                approx_yaw = int(rot_elapsed * ROTATE_YAW_RATE) % 360
                for target in SNAPSHOT_YAWS[1:]:
                    if approx_yaw >= target and target not in snap_done:
                        frame = fr.frame
                        if frame is not None and frame.size > 0:
                            path = OUT_DIR / f"yaw_{target:03d}.jpg"
                            cv2.imwrite(str(path), frame)
                            snap_done.add(target)
                            snap_results.append({"yaw_deg": target, "frame_path": str(path), "t_elapsed": elapsed})
                            print(f"  [t={elapsed:.1f}s {phase_tag}] snapped yaw={target}")
                        break
            elif elapsed < descent_start:
                t.send_rc_control(0, FWD_BIAS, 0, 0)
                phase_tag = "post_rotate_hover"
            else:
                t.send_rc_control(0, 0, DESCENT_VZ, 0)
                phase_tag = "descend"

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

        t.send_rc_control(0, 0, 0, 0)
        time.sleep(0.4)
        print("landing")
        t.land()
        time.sleep(2)
        post_bat = t.get_battery()
        print(f"post-flight battery={post_bat}% (burn={bat-post_bat}%)")

    finally:
        try:
            t.streamoff()
        except Exception:
            pass
        print("restoring hue scene")
        try:
            hue.restore_scene(snap)
        except Exception as e:
            print(f"hue restore failed: {e}")

    print(f"\nlabeling {len(snap_results)} frames via VLM...")
    for r in snap_results:
        frame = cv2.imread(r["frame_path"])
        # Pre-VLM raw stats
        mg = ((frame[:,:,0] > 150) & (frame[:,:,2] > 150) & (frame[:,:,1] < 100)).sum()
        cy = ((frame[:,:,0] > 150) & (frame[:,:,1] > 150) & (frame[:,:,2] < 100)).sum()
        labels = label_frame(frame)
        r["labels"] = labels
        r["pixel_magenta"] = int(mg)
        r["pixel_cyan"] = int(cy)
        print(f"  yaw {r['yaw_deg']:3d}: magenta={labels['magenta']} cyan={labels['cyan']} | mg_px={mg} cy_px={cy} | {labels['note'][:80]}")

    any_magenta = any(r["labels"]["magenta"] for r in snap_results)
    any_cyan = any(r["labels"]["cyan"] for r in snap_results)

    summary_path = OUT_DIR / "summary.md"
    with open(summary_path, "w") as f:
        f.write(f"# Phase 1 (rc-control v2) summary\n\nOut dir: {OUT_DIR}\n\n")
        for r in snap_results:
            lab = r["labels"]
            f.write(f"- yaw ~{r['yaw_deg']}°: magenta={'YES' if lab['magenta'] else 'NO'}, cyan={'YES' if lab['cyan'] else 'NO'} | mg_px={r['pixel_magenta']} cy_px={r['pixel_cyan']} — {lab['note']}\n")
        f.write(f"\n**PASS:** {'YES' if (any_magenta and any_cyan) else 'NO'} (any_magenta={any_magenta}, any_cyan={any_cyan})\n")

    with open(OUT_DIR / "results.json", "w") as f:
        json.dump(snap_results, f, indent=2)

    print()
    print(f"summary: {summary_path}")
    print(f"PASS: {'YES' if (any_magenta and any_cyan) else 'NO'} (any_magenta={any_magenta}, any_cyan={any_cyan})")
    return 0 if (any_magenta and any_cyan) else 1


if __name__ == "__main__":
    sys.exit(main())
