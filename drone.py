"""Tello drone wrapper. CLI for one-shot reads + first-flight choreography.

Usage:
    uv run python drone.py battery
    uv run python drone.py snapshot [out.jpg]
    uv run python drone.py first-flight
    uv run python drone.py takeoff-land
    uv run python drone.py emergency
    uv run python drone.py power-on [boot_wait_s]   # tap power button via the dock
    uv run python drone.py power-off                # tap to power down (docked only)
    uv run python drone.py boot [boot_wait_s]       # power-on + join the Tello WiFi
    uv run python drone.py boot-fly                 # boot + first-flight, one call

The flying host must be joined to the TELLO-XXXXXX WiFi AP before the flight
commands work (drone IP 192.168.10.1). The dock commands (power-on/off, boot)
talk to the ESP32 button-presser over your home network instead; see
tello_dock/tello_dock.ino. Set TELLO_DOCK_URL and TELLO_SSID to override
the dock address and your drone's AP name.
"""

import json
import os
import sys
import time
import urllib.request
from pathlib import Path

from djitellopy import Tello

# ESP32 button-presser dock (see tello_dock/tello_dock.ino).
# Override the dock address with TELLO_DOCK_URL; set the drone's open-AP SSID
# (e.g. TELLO-A1B2C3) in TELLO_SSID so `boot` can hop WiFi to it.
TELLO_DOCK_URL = os.environ.get("TELLO_DOCK_URL", "http://tello-dock.local")
TELLO_SSID = os.environ.get("TELLO_SSID", "")


def _connect() -> Tello:
    t = Tello()
    t.connect()
    return t


def _dock_get(path: str, timeout: float = 10.0) -> dict:
    url = TELLO_DOCK_URL.rstrip("/") + path
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode())


def power_on(boot_wait: str = "9") -> None:
    """Tap the power button via the dock servo, then wait for the Tello to boot.

    The drone must be seated in the cradle. This only presses the button; it does
    not join the Tello WiFi (use `boot` for the full cold-to-connected chain).
    """
    print(f"Pressing power button via dock at {TELLO_DOCK_URL} ...")
    print(f"  dock: {_dock_get('/on')}")
    wait = int(boot_wait)
    print(f"Waiting {wait}s for the Tello to boot and broadcast its AP ...")
    time.sleep(wait)
    print("Tello should be up. Join TELLO-XXXXXX WiFi, or use `boot`.")


def power_off() -> None:
    """Tap the power button to turn the drone OFF.

    Only works while the drone is docked (button under the servo arm). Land it
    first if it's flying. A Tello also auto-powers-off after sitting idle.
    """
    print("Pressing power button to power OFF (drone must be docked) ...")
    print(f"  dock: {_dock_get('/off')}")


def _wifi_device() -> str:
    import subprocess

    out = subprocess.check_output(["networksetup", "-listallhardwareports"], text=True)
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if "Wi-Fi" in line or "AirPort" in line:
            for j in range(i, min(i + 3, len(lines))):
                if lines[j].startswith("Device:"):
                    return lines[j].split(":", 1)[1].strip()
    return ""


def join_tello_wifi(timeout: str = "25") -> bool:
    """macOS: join the Tello's open AP on the WiFi interface.

    Set TELLO_SSID to your drone's SSID. loki keeps the tailnet alive over
    Ethernet while WiFi hops to the drone. Returns True once 192.168.10.1 answers.
    """
    import subprocess

    if not TELLO_SSID:
        print("Set TELLO_SSID to your drone's AP name (e.g. TELLO-A1B2C3).")
        return False
    dev = _wifi_device()
    if not dev:
        print("Could not find a Wi-Fi interface via networksetup.")
        return False
    print(f"Joining {TELLO_SSID} on {dev} ...")
    subprocess.run(["networksetup", "-setairportnetwork", dev, TELLO_SSID], check=False)
    deadline = time.time() + int(timeout)
    while time.time() < deadline:
        r = subprocess.run(
            ["ping", "-c", "1", "-t", "1", "192.168.10.1"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if r.returncode == 0:
            print("Connected to the Tello AP.")
            return True
        time.sleep(1)
    print("Timed out waiting for 192.168.10.1.")
    return False


def boot(boot_wait: str = "9") -> None:
    """Full cold start: press power, wait for boot, then join the Tello WiFi."""
    power_on(boot_wait)
    join_tello_wifi()


def boot_fly() -> None:
    """Cold start, then the boring first-flight choreography."""
    boot()
    time.sleep(2)
    first_flight()


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
    "power-on": power_on,
    "power-off": power_off,
    "join-wifi": join_tello_wifi,
    "boot": boot,
    "boot-fly": boot_fly,
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
