#!/bin/bash
# Diagnose why live controller reads return zero. Run via tcc-run so it inherits
# Terminal.app's GUI session + Full Disk Access (needed to read the TCC db).
cd /Users/ephraiemsarabamoun/projects/tello
{
  echo "=== PROBE (cocoa, 10s) ==="
  SDL_VIDEODRIVER=cocoa ./.venv/bin/python -u pilot.py probe 10 2>&1 | grep -vi hello | tail -6

  echo "=== Input Monitoring (kTCCServiceListenEvent) -- USER TCC.db ==="
  sqlite3 "$HOME/Library/Application Support/com.apple.TCC/TCC.db" \
    "SELECT service, client, auth_value FROM access WHERE service='kTCCServiceListenEvent';" 2>&1

  echo "=== Input Monitoring -- SYSTEM TCC.db ==="
  sudo sqlite3 "/Library/Application Support/com.apple.TCC/TCC.db" \
    "SELECT service, client, auth_value FROM access WHERE service='kTCCServiceListenEvent';" 2>&1 | head -20

  echo "=== recent TCC denials (last 90s) ==="
  log show --last 90s --predicate 'subsystem == "com.apple.TCC" AND eventMessage CONTAINS "ListenEvent"' --style compact 2>/dev/null | tail -15
} > /tmp/diag.out 2>&1
