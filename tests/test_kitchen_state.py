"""Unit tests for kitchen.py state machine + run_mission cleanup contract.

No real Tello, no real VLM, no real Hue. All deps injected as mocks.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(THIS_DIR, ".."))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from kitchen import (
    ActionRequest,
    MissionConfig,
    MissionState,
    Phase,
    run_mission,
)
from lib.vlm_planner import Decision


def _goal_decision() -> Decision:
    return Decision(description="magenta dominates", action="GOAL_REACHED", confidence=0.95, reason="beacon visible")


def _forward_decision() -> Decision:
    return Decision(description="open path", action="FORWARD", confidence=0.7, reason="clear ahead")


def _config(**overrides) -> MissionConfig:
    base = dict(
        outbound_budget_sec=120.0,
        return_budget_sec=120.0,
        max_frame_freeze_sec=3.0,
        max_distance_cm=10000.0,
        goal_streak_threshold=3,
    )
    base.update(overrides)
    return MissionConfig(**base)


def test_outbound_three_goal_reached_transitions_to_kitchen_confirm():
    state = MissionState(config=_config())
    state.begin_phase(Phase.OUTBOUND, now=0.0)

    state.tick(True, 80, _goal_decision(), now=1.0)
    assert state.phase == Phase.OUTBOUND
    assert state.goal_streak == 1

    state.tick(True, 80, _goal_decision(), now=5.0)
    assert state.phase == Phase.OUTBOUND
    assert state.goal_streak == 2

    state.tick(True, 80, _goal_decision(), now=9.0)
    assert state.phase == Phase.KITCHEN_CONFIRM
    assert state.goal_streak == 3


def test_outbound_goal_streak_resets_on_non_goal_decision():
    state = MissionState(config=_config())
    state.begin_phase(Phase.OUTBOUND, now=0.0)

    state.tick(True, 80, _goal_decision(), now=1.0)
    state.tick(True, 80, _goal_decision(), now=5.0)
    assert state.goal_streak == 2

    state.tick(True, 80, _forward_decision(), now=9.0)
    assert state.goal_streak == 0
    assert state.phase == Phase.OUTBOUND


def test_battery_drop_below_22_triggers_abort_in_outbound():
    state = MissionState(config=_config(low_battery_land=22))
    state.begin_phase(Phase.OUTBOUND, now=0.0)
    req = state.tick(True, 21, _forward_decision(), now=1.0)
    assert state.phase == Phase.LANDING
    assert req.kind == "LAND"
    assert state.abort_reason is not None
    assert "battery" in state.abort_reason.lower()


def test_phase_timeout_in_outbound_lands_in_place():
    state = MissionState(config=_config(outbound_budget_sec=60.0))
    state.begin_phase(Phase.OUTBOUND, now=100.0)
    # 200s elapsed > 60s budget
    req = state.tick(True, 80, _forward_decision(), now=300.0)
    assert state.phase == Phase.LANDING
    assert req.kind == "LAND"
    assert state.abort_reason is not None and "timeout" in state.abort_reason.lower()


def test_vlm_unreachable_returns_hover_decision_and_continues():
    """A Decision built from a vlm failure is the HOVER fallback. State should keep going."""
    state = MissionState(config=_config())
    state.begin_phase(Phase.OUTBOUND, now=0.0)
    vlm_failure_decision = Decision(
        description="",
        action="HOVER",
        confidence=0.0,
        reason="vlm_error: timeout",
        raw={"error": "vlm_timeout"},
    )
    req = state.tick(True, 80, vlm_failure_decision, now=1.0)
    assert state.phase == Phase.OUTBOUND
    assert req.kind == "HOVER"
    assert state.abort_reason is None


def test_return_two_goal_reached_transitions_to_home_detected():
    state = MissionState(config=_config(home_streak_threshold=2))
    state.begin_phase(Phase.RETURN, now=0.0)
    state.tick(True, 70, _goal_decision(), now=1.0)
    assert state.phase == Phase.RETURN
    state.tick(True, 70, _goal_decision(), now=5.0)
    assert state.phase == Phase.HOME_DETECTED


def test_action_mapping_translates_to_pose_verbs():
    state = MissionState(config=_config())
    assert state._action_to_request("FORWARD").kind == "forward"
    assert state._action_to_request("FORWARD").magnitude == state.config.forward_step_cm
    assert state._action_to_request("BACK").kind == "back"
    assert state._action_to_request("ROTATE_CW").kind == "rotate_cw"
    assert state._action_to_request("ROTATE_CCW").kind == "rotate_ccw"
    assert state._action_to_request("UP").kind == "up"
    assert state._action_to_request("DOWN").kind == "down"
    assert state._action_to_request("HOVER").kind == "HOVER"


def test_no_frame_returns_hover_no_abort():
    state = MissionState(config=_config())
    state.begin_phase(Phase.OUTBOUND, now=0.0)
    req = state.tick(False, 80, None, now=1.0)
    assert req.kind == "HOVER"
    assert state.phase == Phase.OUTBOUND
    assert state.abort_reason is None


def test_finally_block_restores_hue_and_lands():
    """run_mission must call hue.restore_scene + tello.land in the finally block, even if mid-flight raises."""
    tello = MagicMock()
    tello.get_battery.return_value = 80

    hue = MagicMock()
    snap_state = {"snap": True}
    hue.snapshot_scene.return_value = snap_state

    log = MagicMock()
    log.event = MagicMock()
    log.frame = MagicMock()
    log.close = MagicMock()
    log.run_dir = "/tmp/test-tello-run"

    safety = MagicMock()
    safety.check_pre_takeoff.return_value = (True, None)
    safety.install_landing_signal_handler = MagicMock()

    # VLM blows up mid-flight; ensure finally still fires.
    vlm_outbound = MagicMock()
    vlm_outbound.health_check.return_value = True
    vlm_outbound.decide.side_effect = RuntimeError("vlm boom")

    vlm_return = MagicMock()
    vlm_return.health_check.return_value = True
    vlm_return.decide.return_value = Decision(
        description="cyan", action="GOAL_REACHED", confidence=0.9, reason="home"
    )

    frames = [object(), object(), object(), None]
    iter_frames = iter(frames)

    def frame_source():
        try:
            return next(iter_frames)
        except StopIteration:
            return None

    sleeps: list[float] = []
    times = [0.0]

    def fake_sleep(t: float) -> None:
        sleeps.append(float(t))
        times[0] += float(t)

    def fake_now() -> float:
        times[0] += 0.01
        return times[0]

    config = MissionConfig(
        dry_run=True,
        outbound_budget_sec=60.0,
        return_budget_sec=60.0,
        cycle_period_sec=0.1,
    )

    # Cap loop iterations defensively; the abort or stream-end should land us in finally either way.
    result = run_mission(
        config=config,
        tello=tello,
        hue=hue,
        log=log,
        vlm_outbound=vlm_outbound,
        vlm_return=vlm_return,
        safety_monitor=safety,
        frame_source=frame_source,
        sleep=fake_sleep,
        now=fake_now,
        battery_source=lambda: 80,
        max_cycles=8,
    )

    assert isinstance(result, dict)
    assert hue.restore_scene.called, "finally must restore Hue scene"
    hue.restore_scene.assert_called_with(snap_state)
    assert tello.land.called, "finally must call tello.land via safe_land"
    assert log.close.called, "finally must close the log"
    # Mission ended either in LANDING or a downstream completion phase, not stuck in OUTBOUND.
    assert result["phase"] in ("LANDING", "POST_FLIGHT", "ABORTED", "HOME_DETECTED", "KITCHEN_CONFIRM")


def test_pre_takeoff_low_battery_aborts_and_still_runs_finally():
    """If pre-takeoff battery check fails, hue snapshot may not have happened, but log.close must still run."""
    tello = MagicMock()
    tello.get_battery.return_value = 10
    hue = MagicMock()
    hue.snapshot_scene.return_value = {"snap": "v"}
    log = MagicMock()
    log.run_dir = "/tmp/test-tello-run-2"
    safety = MagicMock()
    safety.check_pre_takeoff.return_value = (False, "battery 10% < 50%")

    config = MissionConfig(dry_run=True)
    result = run_mission(
        config=config,
        tello=tello,
        hue=hue,
        log=log,
        vlm_outbound=MagicMock(health_check=MagicMock(return_value=False)),
        vlm_return=MagicMock(health_check=MagicMock(return_value=False)),
        safety_monitor=safety,
        frame_source=lambda: None,
        sleep=lambda t: None,
        now=lambda: 0.0,
        battery_source=lambda: 10,
        max_cycles=1,
    )
    assert result["phase"] == "ABORTED"
    assert result["abort_reason"] is not None and "battery" in result["abort_reason"].lower()
    assert log.close.called


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
