"""Unit tests for lib.safety. Injected times, no real sleeps."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from lib.pose import Pose  # noqa: E402
from lib.safety import SafetyMonitor  # noqa: E402


def test_pre_takeoff_low_battery_rejects():
    sm = SafetyMonitor(min_battery_takeoff=50)
    ok, reason = sm.check_pre_takeoff(battery=49)
    assert ok is False
    assert reason is not None and "battery" in reason.lower()


def test_pre_takeoff_at_threshold_ok():
    sm = SafetyMonitor(min_battery_takeoff=50)
    ok, reason = sm.check_pre_takeoff(battery=50)
    assert ok is True
    assert reason is None


def test_in_flight_battery_below_land_threshold_aborts():
    sm = SafetyMonitor(low_battery_land=22, max_phase_seconds=120, max_frame_freeze_seconds=3.0, max_distance_cm=1500)
    ok, reason = sm.check_in_flight(
        battery=21,
        last_frame_time=100.0,
        phase_start=100.0,
        pose=Pose(),
        now=101.0,
    )
    assert ok is False
    assert reason is not None and "battery" in reason.lower()


def test_in_flight_phase_timeout_aborts():
    sm = SafetyMonitor(low_battery_land=22, max_phase_seconds=60.0, max_frame_freeze_seconds=3.0, max_distance_cm=1500)
    ok, reason = sm.check_in_flight(
        battery=80,
        last_frame_time=200.0,
        phase_start=100.0,
        pose=Pose(),
        now=200.0,  # 100s after phase_start > 60s
    )
    assert ok is False
    assert reason is not None and "phase" in reason.lower()


def test_in_flight_frame_freeze_aborts():
    sm = SafetyMonitor(low_battery_land=22, max_phase_seconds=120.0, max_frame_freeze_seconds=2.0, max_distance_cm=1500)
    ok, reason = sm.check_in_flight(
        battery=80,
        last_frame_time=100.0,
        phase_start=99.0,
        pose=Pose(),
        now=105.0,  # 5s since last frame > 2s
    )
    assert ok is False
    assert reason is not None and "frame" in reason.lower()


def test_in_flight_distance_aborts():
    sm = SafetyMonitor(low_battery_land=22, max_phase_seconds=120.0, max_frame_freeze_seconds=3.0, max_distance_cm=1000.0)
    p = Pose(x_cm=1500.0, y_cm=0.0)
    ok, reason = sm.check_in_flight(
        battery=80,
        last_frame_time=100.0,
        phase_start=100.0,
        pose=p,
        now=101.0,
    )
    assert ok is False
    assert reason is not None and "distance" in reason.lower()


def test_in_flight_all_ok():
    sm = SafetyMonitor()
    ok, reason = sm.check_in_flight(
        battery=80,
        last_frame_time=100.0,
        phase_start=100.0,
        pose=Pose(),
        now=101.0,
    )
    assert ok is True
    assert reason is None
