import json

mx = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
btns = set()
n = 0
conn = False
vendor = None
for line in open("/tmp/gp.jsonl"):
    line = line.strip()
    if not line:
        continue
    try:
        d = json.loads(line)
    except Exception:
        continue
    if d.get("event") == "connect":
        vendor = d.get("vendor")
        continue
    n += 1
    if d.get("connected"):
        conn = True
    for k in mx:
        if k in d:
            mx[k] = max(mx[k], abs(d[k]))
    for k in ("a", "b", "x", "y", "lb", "rb", "start", "back"):
        if d.get(k):
            btns.add(k)
print("vendor:", vendor, "state_lines:", n, "connected_seen:", conn)
print("max abs axes:", {k: round(v, 2) for k, v in mx.items()})
print("buttons pressed:", sorted(btns))
ok = any(v > 0.3 for v in mx.values()) or bool(btns)
print("VERDICT:", "GCCONTROLLER LIVE READS OK" if ok else "NO INPUT (GCController also blocked)")
