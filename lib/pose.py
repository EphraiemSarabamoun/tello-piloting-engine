"""Dead-reckoning pose tracker. Body frame: +x fwd, +y left, +z up. Yaw 0 = launch heading."""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict


@dataclass
class Pose:
    x_cm: float = 0.0
    y_cm: float = 0.0
    z_cm: float = 0.0
    yaw_deg: float = 0.0

    def update_move(self, kind: str, magnitude: float) -> None:
        """Apply a body-frame move command to world pose."""
        m = float(magnitude)
        k = kind.lower()
        if k in ("forward", "back", "left", "right"):
            yaw_rad = math.radians(self.yaw_deg)
            # Body unit vectors in world frame: forward = (cos yaw, sin yaw), left = (-sin yaw, cos yaw)
            fwd_x, fwd_y = math.cos(yaw_rad), math.sin(yaw_rad)
            left_x, left_y = -math.sin(yaw_rad), math.cos(yaw_rad)
            if k == "forward":
                self.x_cm += m * fwd_x
                self.y_cm += m * fwd_y
            elif k == "back":
                self.x_cm -= m * fwd_x
                self.y_cm -= m * fwd_y
            elif k == "left":
                self.x_cm += m * left_x
                self.y_cm += m * left_y
            elif k == "right":
                self.x_cm -= m * left_x
                self.y_cm -= m * left_y
        elif k == "up":
            self.z_cm += m
        elif k == "down":
            self.z_cm -= m
        elif k == "rotate_ccw":
            self.yaw_deg = (self.yaw_deg + m) % 360.0
        elif k == "rotate_cw":
            self.yaw_deg = (self.yaw_deg - m) % 360.0
        else:
            raise ValueError(f"unknown move kind: {kind}")

    def distance_to_origin(self) -> float:
        return math.sqrt(self.x_cm * self.x_cm + self.y_cm * self.y_cm + self.z_cm * self.z_cm)

    def inverse_heading_deg(self) -> float:
        """Yaw that would face back toward origin's heading reference (yaw + 180 mod 360)."""
        return (self.yaw_deg + 180.0) % 360.0

    def as_dict(self) -> dict:
        return asdict(self)
