"""Claude API fallback for VlmPlanner. Same interface, hits Anthropic vision instead of napoleon Ollama.

Drop-in: import VlmPlannerClaude as VlmPlanner if napoleon is offline.
Requires ANTHROPIC_API_KEY env var (already in ~/.zshrc.secrets per credentials file-canonical setup).
"""

from __future__ import annotations

import base64
import json
import os
from dataclasses import dataclass

import cv2
import requests

ANTHROPIC_API = "https://api.anthropic.com/v1/messages"
MODEL = "claude-haiku-4-5"
ACTIONS = ["FORWARD", "BACK", "ROTATE_CW", "ROTATE_CCW", "UP", "DOWN", "HOVER", "GOAL_REACHED"]


@dataclass
class Decision:
    description: str
    action: str
    confidence: float
    reason: str
    raw: dict


class VlmPlannerClaude:
    def __init__(self, prompt_template_path: str, model: str = MODEL, api_key: str | None = None):
        self._template = open(prompt_template_path).read()
        self.model = model
        self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
        if not self.api_key:
            raise RuntimeError("ANTHROPIC_API_KEY not set")

    def health_check(self, timeout: float = 5.0) -> bool:
        try:
            r = requests.post(
                ANTHROPIC_API,
                headers={
                    "x-api-key": self.api_key,
                    "anthropic-version": "2023-06-01",
                    "content-type": "application/json",
                },
                json={"model": self.model, "max_tokens": 4, "messages": [{"role": "user", "content": "ok"}]},
                timeout=timeout,
            )
            return r.status_code == 200
        except Exception:
            return False

    def decide(self, frame_bgr, goal_descriptor: str, history: list, pose, battery_pct: int,
               phase_elapsed_sec: float, max_phase_sec: float, timeout: float = 30.0) -> Decision:
        _, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        b64 = base64.b64encode(buf).decode()
        last_actions = "->".join(d.action for d in history[-3:]) if history else "none"
        prompt = self._template.format(
            goal_descriptor=goal_descriptor,
            last_actions=last_actions,
            pose_x=round(pose.x_cm, 1),
            pose_y=round(pose.y_cm, 1),
            pose_z=round(pose.z_cm, 1),
            pose_yaw=round(pose.yaw_deg, 1),
            battery=battery_pct,
            phase_elapsed=round(phase_elapsed_sec, 1),
            max_phase=round(max_phase_sec, 1),
            allowed_actions=", ".join(ACTIONS),
        )
        try:
            r = requests.post(
                ANTHROPIC_API,
                headers={
                    "x-api-key": self.api_key,
                    "anthropic-version": "2023-06-01",
                    "content-type": "application/json",
                },
                json={
                    "model": self.model,
                    "max_tokens": 400,
                    "messages": [{
                        "role": "user",
                        "content": [
                            {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": b64}},
                            {"type": "text", "text": prompt + "\n\nReply ONLY with the JSON object, no markdown fences."},
                        ],
                    }],
                },
                timeout=timeout,
            )
            r.raise_for_status()
            text = r.json()["content"][0]["text"].strip()
            if text.startswith("```"):
                text = text.split("```")[1].lstrip("json").strip()
            data = json.loads(text)
        except Exception as e:
            return Decision(description="vlm_error", action="HOVER", confidence=0.0,
                            reason=f"vlm_error: {e}", raw={"error": str(e)})

        action = str(data.get("action", "HOVER")).upper()
        if action not in ACTIONS:
            action = "HOVER"
        return Decision(
            description=str(data.get("description", "?")),
            action=action,
            confidence=float(data.get("confidence", 0.5)),
            reason=str(data.get("reason", "?")),
            raw=data,
        )
