"""API sanity test: takeoff, 2s passive, 2s explicit fwd +50, 2s passive, land.

Tells us whether send_rc_control actually moves the drone (positive fb visible),
or whether commands are being dropped / not applied. No Hue, no VLM, just motion.
"""

from __future__ import annotations

import sys
import time
from djitellopy import Tello


def main() -> int:
    t = Tello()
    t.connect()
    bat = t.get_battery()
    print(f"battery={bat}%")
    if bat < 25:
        print("battery too low to test")
        return 1

    LOOP_HZ = 20
    period = 1.0 / LOOP_HZ

    t.set_speed(20)
    t.takeoff()

    phases = [
        ("passive_1", 2.0,  (0, 0, 0, 0)),
        ("fwd_plus_50", 2.0, (0, 50, 0, 0)),
        ("passive_2", 2.0,  (0, 0, 0, 0)),
    ]

    try:
        for label, duration, rc in phases:
            print(f"\n=== {label} for {duration}s — rc={rc} ===")
            t_start = time.time()
            while True:
                t.send_rc_control(*rc)
                elapsed = time.time() - t_start
                if elapsed >= duration:
                    break
                time.sleep(period)
        t.send_rc_control(0, 0, 0, 0)
        time.sleep(0.4)
    finally:
        print("\nlanding")
        try:
            t.land()
        except Exception as e:
            print(f"land err: {e}")
            try: t.emergency()
            except Exception: pass
        try:
            print(f"post-flight battery={t.get_battery()}%")
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
