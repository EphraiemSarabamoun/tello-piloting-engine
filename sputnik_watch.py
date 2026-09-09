#!/usr/bin/env python3
"""Sputnik (Tello) readiness watcher. Runs on loki in tmux session `tello-watch`.

The regular Tello can't join home WiFi (station mode is EDU-only firmware) and
has no remote power-on, so this daemon closes the gap from the fleet side: the
moment the drone's power button is pressed, loki's free WiFi (en1) associates
with the drone's AP (TELLO-9BFA65, saved from the 2026-05-13 first flight),
opens the SDK session, reads battery, and announces readiness out loud. No
more SSID-dance latency between "drone on" and "fleet can fly it".

States:
  scanning  — join attempt every POLL_SECS (fails fast while drone is off)
  ready     — SDK keepalive every KEEPALIVE_SECS so the drone doesn't idle-
              sleep while a flight is imminent; pauses if /tmp/tello-in-use
              exists (a pilot session owns the command channel)
  cooldown  — ready window expired with no flight: stop propping the drone
              awake and wait for its own idle power-off before re-arming, so
              a forgotten drone never gets keepalive'd into a drained LiPo

State surface for other sessions: /tmp/tello-state.json
"""

import json
import os
import socket
import subprocess
import time

SSID = os.environ.get("TELLO_SSID", "TELLO-9BFA65")
IFACE = os.environ.get("TELLO_IFACE", "en1")
TELLO_ADDR = ("192.168.10.1", 8889)
POLL_SECS = 15
KEEPALIVE_SECS = 8
READY_WINDOW = 600
COOLDOWN_POLL = 120
IN_USE_FLAG = "/tmp/tello-in-use"
STATE_FILE = "/tmp/tello-state.json"
NETWORKSETUP = "/usr/sbin/networksetup"


def log(msg):
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}", flush=True)


def write_state(**kw):
    try:
        with open(STATE_FILE, "w") as f:
            json.dump({**kw, "ts": time.time()}, f)
    except OSError:
        pass


def current_ssid():
    out = subprocess.run([NETWORKSETUP, "-getairportnetwork", IFACE],
                         capture_output=True, text=True).stdout
    return out.split(": ", 1)[1].strip() if ": " in out else None


# Attempt to join the drone Wi-Fi network, then verify the currently associated network.
def try_join():
    try:
        subprocess.run([NETWORKSETUP, "-setairportnetwork", IFACE, SSID],
                       capture_output=True, text=True, timeout=25)
    except subprocess.TimeoutExpired:
        return False
    return current_ssid() == SSID


# Send one UDP SDK command and wait for its response, returning None on timeout or socket failure.
def sdk(sock, cmd, timeout=3.0):
    sock.settimeout(timeout)
    try:
        sock.sendto(cmd.encode(), TELLO_ADDR)
        data, _ = sock.recvfrom(1024)
        return data.decode(errors="replace").strip()
    except (socket.timeout, OSError):
        return None


def say(msg):
    subprocess.Popen(["/usr/bin/say", msg])


# Watch for the drone network, announce SDK readiness, and maintain a bounded ready window; after an unused window, pause until a join attempt fails.
def main():
    subprocess.run([NETWORKSETUP, "-setairportpower", IFACE, "on"],
                   capture_output=True)
    log(f"watching for {SSID} on {IFACE}")
    write_state(status="scanning")
    while True:
        if current_ssid() != SSID and not try_join():
            time.sleep(POLL_SECS)
            continue

        log("associated with drone AP")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", 0))
        ok = None
        for _ in range(3):
            ok = sdk(sock, "command")
            if ok:
                break
        if not ok:
            log("no SDK response; back to scanning")
            sock.close()
            time.sleep(POLL_SECS)
            continue

        batt = sdk(sock, "battery?") or "?"
        log(f"SDK up, battery {batt}%")
        say(f"Sputnik is awake. Battery {batt} percent. Ready to fly.")
        write_state(status="ready", battery=batt)

        t0 = time.time()
        flown = False
        lost = False
        while time.time() - t0 < READY_WINDOW:
            if os.path.exists(IN_USE_FLAG):
                flown = True
                time.sleep(KEEPALIVE_SECS)
                continue
            if current_ssid() != SSID or sdk(sock, "command") is None:
                log("drone gone (powered off or pilot took the AP)")
                lost = True
                break
            time.sleep(KEEPALIVE_SECS)
        sock.close()

        if lost or flown:
            write_state(status="scanning")
            continue

        log("ready window expired unused; letting the drone sleep")
        say("Sputnik was never flown. Letting it sleep.")
        write_state(status="cooldown")
        while try_join():
            time.sleep(COOLDOWN_POLL)
        log("drone powered down; re-armed")
        write_state(status="scanning")


if __name__ == "__main__":
    main()
