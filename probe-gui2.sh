#!/bin/bash
# Post-Input-Monitoring-grant: find which SDL config delivers live reads.
# Run via tcc-run so it inherits Terminal's grant. Verdicts -> /tmp/probe.out.
cd /Users/ephraiemsarabamoun/projects/tello
{
  echo "=== B: cocoa ==="
  SDL_VIDEODRIVER=cocoa ./.venv/bin/python -u pilot.py probe 10 2>&1 | grep -vi hello | tail -5
  echo "=== C: cocoa + joystick_thread ==="
  SDL_VIDEODRIVER=cocoa SDL_JOYSTICK_THREAD=1 ./.venv/bin/python -u pilot.py probe 10 2>&1 | grep -vi hello | tail -5
  echo "=== E: dummy ==="
  SDL_VIDEODRIVER=dummy ./.venv/bin/python -u pilot.py probe 10 2>&1 | grep -vi hello | tail -5
  echo "=== F: dummy + joystick_thread ==="
  SDL_VIDEODRIVER=dummy SDL_JOYSTICK_THREAD=1 ./.venv/bin/python -u pilot.py probe 10 2>&1 | grep -vi hello | tail -5
} > /tmp/probe.out 2>&1
