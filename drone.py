"""Tello drone wrapper. CLI for one-shot reads + first-flight choreography.

Usage:
    uv run python drone.py battery
    uv run python drone.py snapshot [out.jpg]
    uv run python drone.py first-flight
    uv run python drone.py takeoff-land
    uv run python drone.py emergency

Caesar (or whichever Mac is flying) must be joined to the TELLO-XXXXXX WiFi AP
before any of these will work. Drone IP is 192.168.10.1.
"""

import sys
import time
from pathlib import Path

from djitellopy import Tello


def _connect() -> Tello:
    t = Tello()
    t.connect()
    return t


def battery() -> None:
    t = _connect()
    print(f"Battery: {t.get_battery()}%")
    print(f"Temp:    {t.get_temperature()}C")
    print(f"Height:  {t.get_height()}cm")


def snapshot(out: str = "tello_snap.jpg") -> None:
    import cv2

    t = _connect()
    t.streamon()
    time.sleep(1.5)
    frame = t.get_frame_read().frame
    cv2.imwrite(out, frame)
    t.streamoff()
    print(f"Wrote {out} ({frame.shape[1]}x{frame.shape[0]})")


def takeoff_land() -> None:
    t = _connect()
    print(f"Pre-flight battery: {t.get_battery()}%")
    t.takeoff()
    time.sleep(3)
    t.land()
    print("Done.")


def first_flight() -> None:
    """1 m forward, 1 m back, rotate 90, land. Boring on purpose."""
    t = _connect()
    bat = t.get_battery()
    print(f"Pre-flight battery: {bat}%")
    if bat < 30:
        print("Battery too low for first flight. Charge to >=30% first.")
        return
    t.takeoff()
    time.sleep(2)
    t.move_forward(100)
    time.sleep(1)
    t.move_back(100)
    time.sleep(1)
    t.rotate_clockwise(90)
    time.sleep(1)
    t.land()
    print(f"Post-flight battery: {t.get_battery()}%")


def emergency() -> None:
    """Cut motors immediately. Drone will fall. Use only if it's about to crash."""
    t = _connect()
    t.emergency()
    print("Motors cut.")


COMMANDS = {
    "battery": battery,
    "snapshot": snapshot,
    "takeoff-land": takeoff_land,
    "first-flight": first_flight,
    "emergency": emergency,
}


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in COMMANDS:
        print("Commands:", ", ".join(COMMANDS))
        sys.exit(1)
    cmd = sys.argv[1]
    args = sys.argv[2:]
    COMMANDS[cmd](*args)


if __name__ == "__main__":
    main()
