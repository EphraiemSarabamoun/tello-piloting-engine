"""Goal-conditioned VLM planner for the Tello indoor nav mission.

POST pattern mirrors mission.py: napoleon Ollama, gemma4:31b, format=json,
stream=false. The planner returns a Decision every cycle; never raises.
"""

import base64
import json
import logging
from dataclasses import dataclass, field
from typing import Any

import cv2
import requests

NAPOLEON_OLLAMA = "http://100.94.176.110:11434/api/generate"
NAPOLEON_TAGS = "http://100.94.176.110:11434/api/tags"
MODEL = "gemma4:31b"

ACTIONS = [
    "FORWARD",
    "BACK",
    "ROTATE_CW",
    "ROTATE_CCW",
    "UP",
    "DOWN",
    "HOVER",
    "GOAL_REACHED",
]

log = logging.getLogger(__name__)


@dataclass
class Decision:
    description: str
    action: str
    confidence: float
    reason: str
    raw: dict = field(default_factory=dict)


def _format_history(history: list["Decision"]) -> str:
    if not history:
        return "(none)"
    tail = history[-3:]
    return "->".join(d.action for d in tail)


def _format_pose(pose: Any) -> tuple[float, float, float, float]:
    """Pull (x, y, z, yaw) out of either a dict or an object with attrs."""
    def get(key: str, default: float = 0.0) -> float:
        if pose is None:
            return default
        if isinstance(pose, dict):
            v = pose.get(key, default)
        else:
            v = getattr(pose, key, default)
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    return get("x"), get("y"), get("z"), get("yaw")


class VlmPlanner:
    """Goal-conditioned planner that calls a remote VLM each cycle."""

    def __init__(
        self,
        prompt_template_path: str,
        model: str = MODEL,
        endpoint: str = NAPOLEON_OLLAMA,
        tags_endpoint: str = NAPOLEON_TAGS,
    ):
        with open(prompt_template_path) as f:
            self._template = f.read()
        self.template_path = prompt_template_path
        self.model = model
        self.endpoint = endpoint
        self.tags_endpoint = tags_endpoint

    def _render_prompt(
        self,
        goal_descriptor: str,
        history: list[Decision],
        pose: Any,
        battery_pct: int,
        phase_elapsed_sec: float,
        max_phase_sec: float,
    ) -> str:
        x, y, z, yaw = _format_pose(pose)
        return self._template.format(
            goal_descriptor=goal_descriptor,
            last_actions=_format_history(history),
            pose_x=round(x, 2),
            pose_y=round(y, 2),
            pose_z=round(z, 2),
            pose_yaw=round(yaw, 1),
            battery=int(battery_pct),
            phase_elapsed=round(float(phase_elapsed_sec), 1),
            max_phase=round(float(max_phase_sec), 1),
            allowed_actions=", ".join(ACTIONS),
        )

    def decide(
        self,
        frame_bgr,
        goal_descriptor: str,
        history: list[Decision],
        pose,
        battery_pct: int,
        phase_elapsed_sec: float,
        max_phase_sec: float,
        timeout: float = 45.0,
    ) -> Decision:
        """One inference cycle. Returns a Decision; never raises."""
        try:
            prompt = self._render_prompt(
                goal_descriptor=goal_descriptor,
                history=history,
                pose=pose,
                battery_pct=battery_pct,
                phase_elapsed_sec=phase_elapsed_sec,
                max_phase_sec=max_phase_sec,
            )
        except KeyError as e:
            return Decision(
                description="",
                action="HOVER",
                confidence=0.0,
                reason=f"vlm_error: prompt KeyError {e}",
                raw={"error": "prompt_render", "missing_key": str(e)},
            )

        try:
            ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError("cv2.imencode failed")
            b64 = base64.b64encode(buf).decode()

            r = requests.post(
                self.endpoint,
                json={
                    "model": self.model,
                    "prompt": prompt,
                    "images": [b64],
                    "stream": False,
                    "format": "json",
                    "options": {"temperature": 0.2, "num_predict": 300},
                },
                timeout=timeout,
            )
            r.raise_for_status()
            text = r.json()["response"]
            parsed = json.loads(text)
        except Exception as e:
            return Decision(
                description="",
                action="HOVER",
                confidence=0.0,
                reason=f"vlm_error: {e}",
                raw={"error": type(e).__name__, "detail": str(e)},
            )

        return self._coerce(parsed)

    def _coerce(self, parsed: dict) -> Decision:
        raw = dict(parsed) if isinstance(parsed, dict) else {"response": parsed}
        action_in = str(raw.get("action", "HOVER")).strip().upper()
        if action_in not in ACTIONS:
            log.warning("vlm returned invalid action %r; coercing to HOVER", action_in)
            raw["warning"] = f"invalid_action:{action_in}"
            action = "HOVER"
        else:
            action = action_in

        try:
            conf = float(raw.get("confidence", 0.5))
        except (TypeError, ValueError):
            conf = 0.5
        conf = max(0.0, min(1.0, conf))

        return Decision(
            description=str(raw.get("description", "")),
            action=action,
            confidence=conf,
            reason=str(raw.get("reason", "")),
            raw=raw,
        )

    def health_check(self, timeout: float = 5.0) -> bool:
        """GET /api/tags and confirm the configured model is loaded."""
        try:
            r = requests.get(self.tags_endpoint, timeout=timeout)
            r.raise_for_status()
            data = r.json()
            for m in data.get("models", []):
                if m.get("name") == self.model or m.get("model") == self.model:
                    return True
            return False
        except Exception as e:
            log.warning("health_check failed: %s", e)
            return False
