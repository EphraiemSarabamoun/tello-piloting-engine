"""Safety guard predicates + SIGINT-to-land. Mirrors follow.py's safe_land pattern."""

from __future__ import annotations

import signal
import sys
import time
from typing import Any, Optional


def safe_land(tello: Any, log: Any, reason: str) -> None:
    """Stop RC, land, fall back to emergency. Logs reason."""
    if log is not None:
        try:
            log.event("safety", "safe_land_begin", reason=reason)
        except Exception:
            pass
    try:
        try:
            tello.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
        except Exception:
            pass
        tello.land()
        if log is not None:
            try:
                log.event("safety", "safe_land_done", reason=reason)
            except Exception:
                pass
    except Exception as e:
        if log is not None:
            try:
                log.event("safety", "safe_land_failed", reason=reason, err=str(e))
            except Exception:
                pass
        try:
            tello.emergency()
            if log is not None:
                try:
                    log.event("safety", "emergency_fired", reason=reason)
                except Exception:
                    pass
        except Exception as e2:
            if log is not None:
                try:
                    log.event("safety", "emergency_failed", reason=reason, err=str(e2))
                except Exception:
                    pass


class SafetyMonitor:
    """Pre-takeoff and in-flight predicates."""

    def __init__(
        self,
        min_battery_takeoff: int = 50,
        low_battery_land: int = 22,
        max_phase_seconds: float = 120.0,
        max_frame_freeze_seconds: float = 3.0,
        max_distance_cm: float = 1500.0,
    ) -> None:
        self.min_battery_takeoff = min_battery_takeoff
        self.low_battery_land = low_battery_land
        self.max_phase_seconds = max_phase_seconds
        self.max_frame_freeze_seconds = max_frame_freeze_seconds
        self.max_distance_cm = max_distance_cm

    def check_pre_takeoff(self, battery: int) -> tuple[bool, Optional[str]]:
        if battery < self.min_battery_takeoff:
            return False, f"battery {battery}% < min_takeoff {self.min_battery_takeoff}%"
        return True, None

    def check_in_flight(
        self,
        battery: int,
        last_frame_time: float,
        phase_start: float,
        pose: Any,
        now: Optional[float] = None,
    ) -> tuple[bool, Optional[str]]:
        t = time.monotonic() if now is None else now
        if battery < self.low_battery_land:
            return False, f"battery {battery}% < low_land {self.low_battery_land}%"
        if t - phase_start > self.max_phase_seconds:
            return False, f"phase timeout ({t - phase_start:.1f}s > {self.max_phase_seconds:.0f}s)"
        if last_frame_time > 0 and (t - last_frame_time) > self.max_frame_freeze_seconds:
            return False, f"frame freeze ({t - last_frame_time:.1f}s > {self.max_frame_freeze_seconds:.1f}s)"
        if pose is not None:
            try:
                d = pose.distance_to_origin()
            except Exception:
                d = 0.0
            if d > self.max_distance_cm:
                return False, f"distance {d:.0f}cm > max {self.max_distance_cm:.0f}cm"
        return True, None

    def install_landing_signal_handler(self, tello: Any, log: Any) -> None:
        """SIGINT/SIGTERM → send_rc_control(0,0,0,0) → land → exit. Mirrors follow.FaceFollower."""
        def handler(signum, _frame):
            reason = f"signal_{signum}"
            if log is not None:
                try:
                    log.event("safety", "signal_received", signum=int(signum))
                except Exception:
                    pass
            print(f"\n[signal {signum}] landing")
            safe_land(tello, log, reason)
            sys.exit(0)
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)
