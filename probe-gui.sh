#!/bin/bash
# Run via tcc-run (GUI session). Tests the cocoa video driver, which gives SDL a
# real run loop to service the controller's HID source. Verdicts -> /tmp/probe.out.
cd /Users/ephraiemsarabamoun/projects/tello
{
  echo "=== B: cocoa ==="
  SDL_VIDEODRIVER=cocoa ./.venv/bin/python -u pilot.py probe 12 2>&1 | grep -vi hello | tail -6
  echo "=== C: cocoa + joystick_thread ==="
  SDL_VIDEODRIVER=cocoa SDL_JOYSTICK_THREAD=1 ./.venv/bin/python -u pilot.py probe 12 2>&1 | grep -vi hello | tail -6
} > /tmp/probe.out 2>&1
