#!/bin/bash
# Scope the fix: did the Terminal grant land? is there a Swift toolchain for a
# GameController-framework helper (which avoids Input Monitoring entirely)?
echo "=== USER TCC: Input Monitoring rows ==="
sqlite3 "$HOME/Library/Application Support/com.apple.TCC/TCC.db" \
  "SELECT service, client, client_type, auth_value FROM access WHERE service='kTCCServiceListenEvent';" 2>&1
echo "=== USER TCC: anything mentioning Terminal/python ==="
sqlite3 "$HOME/Library/Application Support/com.apple.TCC/TCC.db" \
  "SELECT service, client, auth_value FROM access WHERE client LIKE '%erminal%' OR client LIKE '%python%';" 2>&1
echo "=== SYSTEM TCC: Input Monitoring rows ==="
sudo sqlite3 "/Library/Application Support/com.apple.TCC/TCC.db" \
  "SELECT service, client, client_type, auth_value FROM access WHERE service='kTCCServiceListenEvent';" 2>&1
echo "=== Swift / clang toolchain ==="
which swift swiftc clang 2>&1
xcode-select -p 2>&1
echo "=== pygame's linked SDL version ==="
/Users/ephraiemsarabamoun/projects/tello/.venv/bin/python -c "import pygame; print('SDL', pygame.get_sdl_version())" 2>&1 | tail -1
