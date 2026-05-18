"""Tello Tier 2 commander — VLM on napoleon decides actions every cycle.

Run on loki (joined to Tello AP):

    uv run python mission.py [max_cycles] [goal]

Defaults: 8 cycles, exploratory goal.

Each cycle:
  1. Capture frame
  2. POST to napoleon's Ollama (gemma4:31b vision) with frame + structured prompt
  3. Parse JSON {description, action, reason}
  4. Execute action (small discrete moves)
  5. Repeat until LAND or cycle cap
"""

import base64
import json
import os
import signal
import sys
import time

import cv2
import requests
from djitellopy import Tello

NAPOLEON_OLLAMA = "http://100.94.176.110:11434/api/generate"
MODEL = "gemma4:31b"
MIN_BATTERY = 45
LOW_BATTERY_LAND = 22
LOG_DIR = "/tmp/mission_logs"

DEFAULT_GOAL = "Explore the room safely. Look for interesting objects, describe what you see, and avoid hitting anything."

PROMPT_TMPL = """You are piloting a small indoor drone (Tello). The image is the live view from the drone's forward camera RIGHT NOW.

Your goal: {goal}

Current state:
- Cycle: {cycle}/{max_cycles}
- Battery: {battery}%
- Estimated altitude: ~1.0-1.5m

Choose ONE action from this list:
  HOVER, MOVE_FORWARD, MOVE_BACK, ROTATE_CW, ROTATE_CCW, MOVE_UP, MOVE_DOWN, LAND

Rules:
- Avoid hitting walls, furniture, cables, glass, and people. Maintain at least 60cm of clearance.
- Don't fly higher than 2m total.
- LAND if battery is critically low, the surroundings are dangerous, or the goal is achieved.
- Default to HOVER when uncertain.
- Each MOVE is ~30cm; each ROTATE is ~30 degrees.

Reply ONLY in this exact JSON shape, no extra text:
{{"description": "<brief description of what you see>", "action": "<ACTION>", "reason": "<one-sentence reason>"}}
"""


def query_vlm(frame_bgr, cycle, max_cycles, battery, goal, timeout=45):
    _, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    b64 = base64.b64encode(buf).decode()
    prompt = PROMPT_TMPL.format(cycle=cycle, max_cycles=max_cycles, battery=battery, goal=goal)
    r = requests.post(NAPOLEON_OLLAMA, json={
        "model": MODEL,
        "prompt": prompt,
        "images": [b64],
        "stream": False,
        "format": "json",
        "options": {"temperature": 0.3, "num_predict": 200}
    }, timeout=timeout)
    r.raise_for_status()
    text = r.json()["response"]
    return json.loads(text)


def execute(t, action):
    a = action.upper()
    if a == "HOVER":
        time.sleep(1.5)
    elif a == "MOVE_FORWARD":
        t.move_forward(30)
    elif a == "MOVE_BACK":
        t.move_back(30)
    elif a == "ROTATE_CW":
        t.rotate_clockwise(30)
    elif a == "ROTATE_CCW":
        t.rotate_counter_clockwise(30)
    elif a == "MOVE_UP":
        t.move_up(30)
    elif a == "MOVE_DOWN":
        t.move_down(30)
    elif a == "LAND":
        return False  # signal stop
    else:
        time.sleep(0.5)
    return True


def safe_land(t):
    try:
        t.land()
        print("[safe_land] landed")
    except Exception as e:
        print(f"[safe_land] land failed ({e}); emergency")
        try: t.emergency()
        except Exception as e2: print(f"[safe_land] emergency failed: {e2}")


def main():
    max_cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    goal = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_GOAL
    os.makedirs(LOG_DIR, exist_ok=True)
    run_id = time.strftime("%Y%m%d-%H%M%S")
    log_path = os.path.join(LOG_DIR, f"mission_{run_id}.jsonl")
    log_f = open(log_path, "w")
    print(f"[init] log → {log_path}")

    t = Tello()
    t.connect()
    bat = t.get_battery()
    print(f"[boot] battery={bat}%")
    if bat < MIN_BATTERY:
        print(f"battery {bat}% < {MIN_BATTERY}% — abort"); return

    def sig(*_):
        print("\n[signal] landing")
        safe_land(t)
        log_f.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, sig)
    signal.signal(signal.SIGTERM, sig)

    try: t.set_speed(20)
    except Exception: pass

    t.streamon()
    time.sleep(2)
    fr = t.get_frame_read()
    t.takeoff()
    time.sleep(2)

    try:
        for cycle in range(1, max_cycles + 1):
            frame = fr.frame
            if frame is None or frame.size == 0:
                print(f"[cycle {cycle}] no frame, hovering")
                time.sleep(1)
                continue
            frame_path = os.path.join(LOG_DIR, f"cycle_{run_id}_{cycle:02d}.jpg")
            cv2.imwrite(frame_path, frame)

            bat = t.get_battery()
            print(f"\n=== cycle {cycle}/{max_cycles} battery={bat}% ===")
            if bat < LOW_BATTERY_LAND:
                print(f"[battery] {bat}% < {LOW_BATTERY_LAND}% — landing")
                break

            t0 = time.time()
            try:
                decision = query_vlm(frame, cycle, max_cycles, bat, goal)
            except Exception as e:
                print(f"[vlm] error: {e} — hovering this cycle")
                log_f.write(json.dumps({"cycle": cycle, "error": str(e), "t": t0}) + "\n")
                log_f.flush()
                time.sleep(1)
                continue
            dt = time.time() - t0

            desc = decision.get("description", "?")
            action = decision.get("action", "HOVER")
            reason = decision.get("reason", "?")
            print(f"  scene  ({dt:.1f}s): {desc}")
            print(f"  action: {action}")
            print(f"  reason: {reason}")

            log_f.write(json.dumps({
                "cycle": cycle, "battery": bat, "frame": frame_path,
                "decision": decision, "vlm_dt": dt, "t": t0
            }) + "\n")
            log_f.flush()

            keep_going = True
            try:
                keep_going = execute(t, action)
            except Exception as e:
                print(f"[exec] {action} failed: {e}")
            if not keep_going:
                print("[mission] LAND requested")
                break
            time.sleep(0.5)
    finally:
        safe_land(t)
        try: t.streamoff()
        except Exception: pass
        log_f.close()
        print(f"[exit] battery={t.get_battery()}%, log={log_path}")


if __name__ == "__main__":
    main()
