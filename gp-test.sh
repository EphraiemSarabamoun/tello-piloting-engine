#!/bin/bash
# Recompile the GameController reader, capture 12s of input, summarize. Run via
# tcc-run (GUI session). Summary -> /tmp/gp.summary.
cd /Users/ephraiemsarabamoun/projects/tello
{
  swiftc -O gamepad_reader.swift -o gamepad-reader -framework GameController -framework AppKit 2>&1 | head
  echo "=== capturing 12s -- WIGGLE STICKS + TAP BUTTONS NOW ==="
  ./gamepad-reader > /tmp/gp.jsonl 2>&1 &
  PID=$!
  sleep 12
  kill "$PID" 2>/dev/null
  ./.venv/bin/python gp_analyze.py
} > /tmp/gp.summary 2>&1
