#!/bin/bash
# Open the controller flight loop in FPV mode on loki. A separate "Tello FPV"
# video window appears -- THAT is the one to keep focused while flying.
pkill -f gamepad-reader 2>/dev/null
pkill -f "pilot.py" 2>/dev/null
sleep 1
caffeinate -u -t 2
osascript <<'OSA'
tell application "Terminal"
    activate
    set w to do script "cd ~/projects/tello && ./.venv/bin/python pilot.py fly --fpv"
    set custom title of w to "TELLO FPV (status) -- focus the VIDEO window to fly"
end tell
OSA
