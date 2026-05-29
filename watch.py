"""Live controller monitor for loki's screen. Runs gamepad-reader, prints a clean
updating line, and logs detections to /tmp/watch.log so the remote session can
read the verdict too. Shows INPUT! with live values when the sticks/buttons move,
or 'waiting...' when nothing is arriving."""
import json
import subprocess
import sys
import time

log = open("/tmp/watch.log", "w")
def L(s):
    log.write(s + "\n")
    log.flush()

p = subprocess.Popen(["./gamepad-reader"], stdout=subprocess.PIPE, text=True, bufsize=1)
seen = False
last = 0.0
mx = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
allb = set()
print("Watching controller. CLICK THIS WINDOW, then wiggle the sticks. Ctrl-C to stop.\n")
L("watch start")
for line in p.stdout:
    line = line.strip()
    if not line:
        continue
    try:
        d = json.loads(line)
    except Exception:
        continue
    if "event" in d:
        print("event:", d)
        L("event:" + str(d))
        continue
    for k in mx:
        if k in d:
            mx[k] = max(mx[k], abs(d[k]))
    btns = [k for k in ("a", "b", "x", "y", "lb", "rb", "start", "back") if d.get(k)]
    allb.update(btns)
    moved = any(abs(d.get(k, 0)) > 0.25 for k in mx) or btns
    now = time.time()
    if moved:
        if not seen:
            L("FIRST INPUT DETECTED")
        seen = True
        sys.stdout.write(
            f"\rINPUT!  lx={d.get('lx',0):+.2f} ly={d.get('ly',0):+.2f} "
            f"rx={d.get('rx',0):+.2f} ry={d.get('ry',0):+.2f} btns={btns}        "
        )
        sys.stdout.flush()
        L(f"input lx={d.get('lx',0):+.2f} ly={d.get('ly',0):+.2f} rx={d.get('rx',0):+.2f} ry={d.get('ry',0):+.2f} btns={btns}")
    elif now - last > 1.0:
        last = now
        sys.stdout.write(f"\rwaiting...  seen_input={seen}  maxabs={ {k: round(v,2) for k,v in mx.items()} }   ")
        sys.stdout.flush()
        L(f"waiting seen={seen} maxabs={dict((k, round(v,2)) for k,v in mx.items())} btns_ever={sorted(allb)}")
