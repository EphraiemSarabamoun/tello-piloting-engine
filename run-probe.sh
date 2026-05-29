#!/bin/bash
# Run the controller live-read probe and write the verdict to /tmp/probe.out.
# Invoked via tcc-run so it executes inside loki's Terminal.app GUI session.
cd /Users/ephraiemsarabamoun/projects/tello
./.venv/bin/python -u pilot.py probe 25 2>&1 | grep -vi "hello from the pygame" > /tmp/probe.out 2>&1
