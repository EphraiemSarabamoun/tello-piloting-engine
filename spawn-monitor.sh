#!/bin/bash
# Open a VISIBLE, frontmost Terminal on loki running the live stick monitor.
# Used to confirm stick directions before flight. Logs to /tmp/tello-monitor.log.
pkill -f gamepad-reader 2>/dev/null
pkill -f "pilot.py monitor" 2>/dev/null
sleep 1
caffeinate -u -t 2
osascript <<'OSA'
tell application "Terminal"
    activate
    set w to do script "cd ~/projects/tello && ./.venv/bin/python pilot.py monitor 120"
    set custom title of w to "STICK SIGN CHECK -- click me + push sticks"
end tell
OSA
