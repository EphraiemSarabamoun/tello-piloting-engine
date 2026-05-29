#!/bin/bash
# Timing-robust capture: NO recompile (use the built binary), 40s window.
cd /Users/ephraiemsarabamoun/projects/tello
{
  echo "=== capturing 40s CONTINUOUS -- KEEP WIGGLING THE ENTIRE TIME ==="
  ./gamepad-reader > /tmp/gp.jsonl 2>&1 &
  PID=$!
  sleep 40
  kill "$PID" 2>/dev/null
  ./.venv/bin/python gp_analyze.py
} > /tmp/gp.summary 2>&1
