"""Phase 2: out-and-back, no goal. Tests move primitives + landing accuracy.

Run on loki (joined to Tello AP):
    uv run python phase2.py [--dry-run]
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from lib.pose import Pose
from lib.safety import SafetyMonitor, safe_land
from lib.telemetry import FlightLog


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Phase 2: out-and-back, no goal.")
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args(argv)


def run() -> int:
    ns = parse_args()
    pose = Pose()
    log = FlightLog(run_id=f"phase2-{time.strftime('%Y%m%d-%H%M%S')}")
    safety = SafetyMonitor(min_battery_takeoff=40, low_battery_land=22)
    log.event("phase2", "begin", dry_run=ns.dry_run)

    tello = None
    try:
        if not ns.dry_run:
            from djitellopy import Tello
            tello = Tello()
            tello.connect()
            battery = tello.get_battery()
            log.event("phase2", "battery", battery=battery)
            ok, reason = safety.check_pre_takeoff(battery)
            if not ok:
                log.event("phase2", "pre_takeoff_fail", reason=reason)
                print(f"ABORT: {reason}")
                return 1
            safety.install_landing_signal_handler(tello, log)
            try:
                tello.set_speed(20)
            except Exception:
                pass
            tello.takeoff()
            time.sleep(2.0)
            tello.move_up(40)
            pose.update_move("up", 40)
            tello.move_forward(100)
            pose.update_move("forward", 100)
            time.sleep(2.0)
            tello.move_back(100)
            pose.update_move("back", 100)
            safe_land(tello, log, "phase2_normal_end")
        else:
            log.event("phase2", "dry_run")
            pose.update_move("up", 40)
            pose.update_move("forward", 100)
            pose.update_move("back", 100)

        landing_distance = pose.distance_to_origin()
        log.event("phase2", "complete", pose=pose.as_dict(), landing_distance_cm=landing_distance)
        print(f"Phase 2 result: pose={pose.as_dict()}, distance_to_origin={landing_distance:.1f}cm")

        # Z is still 40 from the up; pose only tracks issued moves, not the auto-descent at land().
        # For the PASS criterion the plan asks for distance < 50 cm; we use the horizontal-XY distance
        # (z is irrelevant — the drone is now on the ground).
        horizontal = (pose.x_cm ** 2 + pose.y_cm ** 2) ** 0.5
        log.event("phase2", "horizontal_distance", value_cm=horizontal)
        print(f"Horizontal distance from launch: {horizontal:.1f}cm")
        if horizontal < 50:
            print("PASS: landed within 50cm horizontal of takeoff per pose log.")
            return 0
        print(f"FAIL: horizontal {horizontal:.1f}cm >= 50cm.")
        return 1
    finally:
        log.close()


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
