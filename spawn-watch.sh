#!/bin/bash
# Open a VISIBLE, frontmost Terminal window on loki's display running the live
# controller monitor. Tests whether a focused/foreground app gets input.
pkill -f gamepad-reader 2>/dev/null
pkill -f watch.py 2>/dev/null
sleep 1
caffeinate -u -t 2
osascript <<'OSA'
tell application "Terminal"
    activate
    set w to do script "cd ~/projects/tello && ./.venv/bin/python watch.py"
    set custom title of w to "GAMEPAD MONITOR -- click me + wiggle"
end tell
OSA
