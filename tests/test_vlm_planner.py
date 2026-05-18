"""Offline unit tests for lib/vlm_planner.

No network. requests.post and requests.get are mocked; gemma is never called.
"""

import json
import os
import sys
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
import requests

# Project root is one level above tests/. Insert for `from lib.vlm_planner import ...`.
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from lib.vlm_planner import ACTIONS, Decision, VlmPlanner  # noqa: E402

PROMPTS_DIR = os.path.join(PROJECT_ROOT, "prompts")
OUTBOUND_TPL = os.path.join(PROMPTS_DIR, "outbound.tpl")
RETURN_TPL = os.path.join(PROMPTS_DIR, "return.tpl")


def make_synthetic_magenta_frame(width=720, height=480):
    """Frame with a big magenta blob in the center."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cx, cy = width // 2, height // 2
    frame[cy - 100:cy + 100, cx - 150:cx + 150] = [255, 0, 255]
    return frame


def make_synthetic_dark_frame(width=720, height=480):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _fake_ollama_response(payload: dict) -> MagicMock:
    """Build a MagicMock that mimics a requests.Response for the /api/generate endpoint."""
    resp = MagicMock()
    resp.status_code = 200
    resp.raise_for_status = MagicMock()
    resp.json = MagicMock(return_value={"response": json.dumps(payload)})
    return resp


def _fake_tags_response(model_names: list[str]) -> MagicMock:
    resp = MagicMock()
    resp.status_code = 200
    resp.raise_for_status = MagicMock()
    resp.json = MagicMock(return_value={
        "models": [{"name": n, "model": n} for n in model_names]
    })
    return resp


@pytest.fixture
def planner():
    return VlmPlanner(prompt_template_path=OUTBOUND_TPL)


@pytest.fixture
def pose():
    return {"x": 1.2, "y": 0.0, "z": 1.1, "yaw": 15.0}


def test_decide_returns_decision_with_valid_action(planner, pose):
    fake = _fake_ollama_response({
        "description": "open hallway with a pink glow at the far end",
        "action": "FORWARD",
        "confidence": 0.82,
        "reason": "magenta visible center, path is clear",
    })
    frame = make_synthetic_magenta_frame()
    with patch("lib.vlm_planner.requests.post", return_value=fake) as post_mock:
        d = planner.decide(
            frame_bgr=frame,
            goal_descriptor="reach the kitchen magenta light",
            history=[],
            pose=pose,
            battery_pct=87,
            phase_elapsed_sec=4.0,
            max_phase_sec=120.0,
        )
    assert isinstance(d, Decision)
    assert d.action == "FORWARD"
    assert d.action in ACTIONS
    assert 0.0 <= d.confidence <= 1.0
    assert d.confidence == pytest.approx(0.82)
    assert "hallway" in d.description
    assert post_mock.call_count == 1


def test_decide_coerces_unknown_action_to_hover(planner, pose):
    fake = _fake_ollama_response({
        "description": "I see a thing",
        "action": "WIGGLE",
        "confidence": 0.4,
        "reason": "not a real action",
    })
    frame = make_synthetic_dark_frame()
    with patch("lib.vlm_planner.requests.post", return_value=fake):
        d = planner.decide(
            frame_bgr=frame,
            goal_descriptor="reach the kitchen magenta light",
            history=[],
            pose=pose,
            battery_pct=70,
            phase_elapsed_sec=10.0,
            max_phase_sec=120.0,
        )
    assert d.action == "HOVER"
    assert "invalid_action:WIGGLE" in d.raw.get("warning", "")


def test_decide_returns_hover_on_exception(planner, pose):
    frame = make_synthetic_dark_frame()
    with patch("lib.vlm_planner.requests.post", side_effect=requests.Timeout("boom")):
        d = planner.decide(
            frame_bgr=frame,
            goal_descriptor="reach the kitchen magenta light",
            history=[],
            pose=pose,
            battery_pct=60,
            phase_elapsed_sec=30.0,
            max_phase_sec=120.0,
            timeout=1.0,
        )
    assert d.action == "HOVER"
    assert d.confidence == 0.0
    assert "vlm_error" in d.reason
    assert d.raw.get("error") == "Timeout"


def test_decide_renders_template_with_all_placeholders(tmp_path, pose):
    tpl = (
        "goal={goal_descriptor}\n"
        "last={last_actions}\n"
        "pose=({pose_x},{pose_y},{pose_z},{pose_yaw})\n"
        "battery={battery}\n"
        "phase={phase_elapsed}/{max_phase}\n"
        "actions={allowed_actions}\n"
    )
    p = tmp_path / "fake.tpl"
    p.write_text(tpl)
    planner = VlmPlanner(prompt_template_path=str(p))

    history = [
        Decision(description="", action="FORWARD", confidence=0.7, reason=""),
        Decision(description="", action="ROTATE_CW", confidence=0.5, reason=""),
        Decision(description="", action="HOVER", confidence=0.4, reason=""),
    ]

    rendered = planner._render_prompt(
        goal_descriptor="kitchen magenta",
        history=history,
        pose=pose,
        battery_pct=55,
        phase_elapsed_sec=12.5,
        max_phase_sec=120.0,
    )

    assert "goal=kitchen magenta" in rendered
    assert "FORWARD->ROTATE_CW->HOVER" in rendered
    assert "pose=(1.2,0.0,1.1,15.0)" in rendered
    assert "battery=55" in rendered
    assert "phase=12.5/120.0" in rendered
    for a in ACTIONS:
        assert a in rendered

    # Also sanity check the real outbound + return templates accept the same kwargs.
    for real_tpl in (OUTBOUND_TPL, RETURN_TPL):
        VlmPlanner(prompt_template_path=real_tpl)._render_prompt(
            goal_descriptor="x",
            history=[],
            pose=pose,
            battery_pct=50,
            phase_elapsed_sec=0.0,
            max_phase_sec=60.0,
        )


def test_health_check_finds_gemma4(planner):
    fake = _fake_tags_response(["gemma4:31b", "nemotron3:33b"])
    with patch("lib.vlm_planner.requests.get", return_value=fake):
        assert planner.health_check() is True


def test_health_check_missing_model_returns_false(planner):
    fake = _fake_tags_response(["nemotron3:33b", "qwen3.6:35b"])
    with patch("lib.vlm_planner.requests.get", return_value=fake):
        assert planner.health_check() is False
