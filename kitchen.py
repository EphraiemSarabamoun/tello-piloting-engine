"""Autonomous Tello mission: launch -> kitchen (magenta beacon) -> home (cyan beacon) -> land.

State machine + run_mission() entry point. Unit-tested via tests/test_kitchen_state.py.

Run on loki (joined to Tello AP):

    uv run python kitchen.py [--home-beacon-light Polaris] [--vlm-endpoint URL] [--dry-run]
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from lib.hue_beacon import HueBeacon, LIGHT_IDS, KITCHEN_LIGHTS
from lib.pose import Pose
from lib.safety import SafetyMonitor, safe_land
from lib.telemetry import FlightLog
from lib.vlm_planner import Decision, VlmPlanner


class Phase(str, Enum):
    INIT = "INIT"
    PRE_FLIGHT = "PRE_FLIGHT"
    ARMED = "ARMED"
    ASCEND_TO_CRUISE = "ASCEND_TO_CRUISE"
    OUTBOUND = "OUTBOUND"
    KITCHEN_CONFIRM = "KITCHEN_CONFIRM"
    RETURN = "RETURN"
    HOME_DETECTED = "HOME_DETECTED"
    LANDING = "LANDING"
    POST_FLIGHT = "POST_FLIGHT"
    ABORTED = "ABORTED"


@dataclass
class MissionConfig:
    home_beacon_light_name: str = "Polaris"
    vlm_endpoint: str = "http://100.94.176.110:11434/api/generate"
    outbound_prompt_path: str = os.path.join(THIS_DIR, "prompts", "outbound.tpl")
    return_prompt_path: str = os.path.join(THIS_DIR, "prompts", "return.tpl")
    min_battery_takeoff: int = 50
    low_battery_land: int = 22
    cruise_altitude_cm: int = 40
    cycle_period_sec: float = 4.0
    outbound_budget_sec: float = 120.0
    return_budget_sec: float = 120.0
    max_frame_freeze_sec: float = 3.0
    max_distance_cm: float = 1500.0
    goal_streak_threshold: int = 3
    home_streak_threshold: int = 2
    kitchen_hover_sec: float = 3.0
    descent_step_cm: int = 20
    forward_step_cm: int = 40
    back_step_cm: int = 30
    rotate_step_deg: int = 25
    up_step_cm: int = 20
    down_step_cm: int = 20
    dry_run: bool = False


@dataclass
class ActionRequest:
    kind: str
    magnitude: float = 0.0


@dataclass
class MissionState:
    """Pure state-transition logic. No tello / hue / vlm imports needed for unit tests."""

    config: MissionConfig
    phase: Phase = Phase.INIT
    pose: Pose = field(default_factory=Pose)
    history: list[Decision] = field(default_factory=list)
    goal_streak: int = 0
    home_streak: int = 0
    phase_start: float = 0.0
    last_frame_time: float = 0.0
    abort_reason: Optional[str] = None
    final_battery: Optional[int] = None

    def begin_phase(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self.phase_start = now
        if phase == Phase.OUTBOUND:
            self.goal_streak = 0
        elif phase == Phase.RETURN:
            self.home_streak = 0

    def phase_budget(self) -> float:
        if self.phase == Phase.OUTBOUND:
            return self.config.outbound_budget_sec
        if self.phase == Phase.RETURN:
            return self.config.return_budget_sec
        return 600.0

    def tick(
        self,
        frame_present: bool,
        battery: int,
        decision: Optional[Decision],
        now: float,
    ) -> ActionRequest:
        """Advance one cycle. Returns the action to execute. Mutates self.phase if transition is needed."""
        if not frame_present:
            return ActionRequest("HOVER")
        if frame_present:
            self.last_frame_time = now

        if battery < self.config.low_battery_land:
            self.abort_reason = f"battery {battery}% below land threshold {self.config.low_battery_land}%"
            self.phase = Phase.LANDING
            return ActionRequest("LAND")

        elapsed = now - self.phase_start
        if elapsed > self.phase_budget():
            self.abort_reason = f"phase {self.phase.value} timeout ({elapsed:.1f}s > {self.phase_budget():.0f}s)"
            self.phase = Phase.LANDING
            return ActionRequest("LAND")

        if self.last_frame_time > 0 and (now - self.last_frame_time) > self.config.max_frame_freeze_sec:
            self.abort_reason = f"frame freeze ({now - self.last_frame_time:.1f}s)"
            self.phase = Phase.LANDING
            return ActionRequest("LAND")

        if self.pose.distance_to_origin() > self.config.max_distance_cm:
            self.abort_reason = f"distance {self.pose.distance_to_origin():.0f}cm > max"
            self.phase = Phase.LANDING
            return ActionRequest("LAND")

        if self.phase == Phase.OUTBOUND:
            return self._tick_outbound(decision)
        if self.phase == Phase.RETURN:
            return self._tick_return(decision)
        return ActionRequest("HOVER")

    def _tick_outbound(self, decision: Optional[Decision]) -> ActionRequest:
        if decision is None:
            return ActionRequest("HOVER")
        self.history.append(decision)
        if decision.action == "GOAL_REACHED":
            self.goal_streak += 1
            if self.goal_streak >= self.config.goal_streak_threshold:
                self.phase = Phase.KITCHEN_CONFIRM
            return ActionRequest("HOVER")
        self.goal_streak = 0
        return self._action_to_request(decision.action)

    def _tick_return(self, decision: Optional[Decision]) -> ActionRequest:
        if decision is None:
            return ActionRequest("HOVER")
        self.history.append(decision)
        if decision.action == "GOAL_REACHED":
            self.home_streak += 1
            if self.home_streak >= self.config.home_streak_threshold:
                self.phase = Phase.HOME_DETECTED
            return ActionRequest("HOVER")
        self.home_streak = 0
        return self._action_to_request(decision.action)

    def _action_to_request(self, action: str) -> ActionRequest:
        c = self.config
        mapping = {
            "FORWARD":    ActionRequest("forward", c.forward_step_cm),
            "BACK":       ActionRequest("back", c.back_step_cm),
            "ROTATE_CW":  ActionRequest("rotate_cw", c.rotate_step_deg),
            "ROTATE_CCW": ActionRequest("rotate_ccw", c.rotate_step_deg),
            "UP":         ActionRequest("up", c.up_step_cm),
            "DOWN":       ActionRequest("down", c.down_step_cm),
            "HOVER":      ActionRequest("HOVER"),
        }
        return mapping.get(action.upper(), ActionRequest("HOVER"))


# Adapter helpers (talk to real Tello / Hue). Kept simple so unit tests can mock at the boundary.


def _execute_action(tello: Any, pose: Pose, req: ActionRequest, dry_run: bool, sleep: Callable[[float], None]) -> None:
    if req.kind == "HOVER":
        sleep(1.0)
        return
    if req.kind == "forward":
        if not dry_run:
            tello.move_forward(int(req.magnitude))
        pose.update_move("forward", req.magnitude)
        return
    if req.kind == "back":
        if not dry_run:
            tello.move_back(int(req.magnitude))
        pose.update_move("back", req.magnitude)
        return
    if req.kind == "up":
        if not dry_run:
            tello.move_up(int(req.magnitude))
        pose.update_move("up", req.magnitude)
        return
    if req.kind == "down":
        if not dry_run:
            tello.move_down(int(req.magnitude))
        pose.update_move("down", req.magnitude)
        return
    if req.kind == "rotate_cw":
        if not dry_run:
            tello.rotate_clockwise(int(req.magnitude))
        pose.update_move("rotate_cw", req.magnitude)
        return
    if req.kind == "rotate_ccw":
        if not dry_run:
            tello.rotate_counter_clockwise(int(req.magnitude))
        pose.update_move("rotate_ccw", req.magnitude)
        return
    if req.kind == "LAND":
        return
    if req.kind == "NONE":
        return


def _build_tello_default() -> Any:
    from djitellopy import Tello  # lazy: tests don't import djitellopy
    return Tello()


def _grab_frame(fr: Any) -> Any:
    try:
        return fr.frame
    except Exception:
        return None


def run_mission(
    config: MissionConfig,
    tello: Optional[Any] = None,
    hue: Optional[Any] = None,
    log: Optional[Any] = None,
    vlm_outbound: Optional[Any] = None,
    vlm_return: Optional[Any] = None,
    safety_monitor: Optional[Any] = None,
    frame_source: Optional[Callable[[], Any]] = None,
    sleep: Callable[[float], None] = time.sleep,
    now: Callable[[], float] = time.monotonic,
    battery_source: Optional[Callable[[], int]] = None,
    max_cycles: int = 500,
) -> dict:
    """Run the full state machine. Returns a result dict with phase + abort_reason + pose."""
    home_beacon_id = LIGHT_IDS.get(config.home_beacon_light_name, LIGHT_IDS["Polaris"])

    # Dependency wiring
    if tello is None:
        tello = _build_tello_default()
    if log is None:
        log = FlightLog()
    if safety_monitor is None:
        safety_monitor = SafetyMonitor(
            min_battery_takeoff=config.min_battery_takeoff,
            low_battery_land=config.low_battery_land,
            max_phase_seconds=config.outbound_budget_sec,
            max_frame_freeze_seconds=config.max_frame_freeze_sec,
            max_distance_cm=config.max_distance_cm,
        )
    if hue is None:
        try:
            hue = HueBeacon()
        except Exception as e:
            log.event("init", "hue_unavailable", err=str(e))
            hue = None
    if vlm_outbound is None:
        vlm_outbound = VlmPlanner(config.outbound_prompt_path, endpoint=config.vlm_endpoint)
    if vlm_return is None:
        vlm_return = VlmPlanner(config.return_prompt_path, endpoint=config.vlm_endpoint)

    state = MissionState(config=config)
    state.phase = Phase.INIT
    snap: Optional[dict] = None

    def get_battery() -> int:
        if battery_source is not None:
            return int(battery_source())
        try:
            return int(tello.get_battery())
        except Exception:
            return 0

    try:
        # INIT
        log.event("init", "begin", config=config.__dict__)
        if not config.dry_run:
            try:
                tello.connect()
            except Exception as e:
                log.event("init", "connect_failed", err=str(e))
                raise

        battery = get_battery()
        log.event("init", "battery", battery=battery)
        ok, reason = safety_monitor.check_pre_takeoff(battery)
        if not ok:
            state.abort_reason = reason
            state.phase = Phase.ABORTED
            log.event("init", "pre_takeoff_fail", reason=reason)
            return _finalize_result(state, battery)

        # PRE_FLIGHT (Hue setup)
        state.phase = Phase.PRE_FLIGHT
        if hue is not None:
            try:
                snap = hue.snapshot_scene()
                hue.set_kitchen_magenta()
                hue.set_home_cyan(home_beacon_id)
                hue.dim_others(except_ids=KITCHEN_LIGHTS + [home_beacon_id])
                log.event("pre_flight", "hue_set", home_beacon=config.home_beacon_light_name)
            except Exception as e:
                log.event("pre_flight", "hue_setup_failed", err=str(e))

        # VLM health
        try:
            healthy = vlm_outbound.health_check()
            log.event("pre_flight", "vlm_health", healthy=bool(healthy))
        except Exception as e:
            log.event("pre_flight", "vlm_health_failed", err=str(e))

        safety_monitor.install_landing_signal_handler(tello, log)

        if not config.dry_run:
            try:
                tello.set_speed(20)
            except Exception:
                pass
            tello.streamon()
            sleep(3.0)
            fr = tello.get_frame_read() if frame_source is None else None
            for _ in range(8):
                _ = _grab_frame(fr) if fr is not None else (frame_source() if frame_source else None)
        else:
            fr = None

        # ARMED + takeoff
        state.phase = Phase.ARMED
        log.event("armed", "takeoff")
        if not config.dry_run:
            tello.takeoff()
            sleep(2.0)

        # ASCEND
        state.phase = Phase.ASCEND_TO_CRUISE
        if not config.dry_run:
            tello.move_up(config.cruise_altitude_cm)
        state.pose.update_move("up", config.cruise_altitude_cm)
        log.event("ascend", "complete", z_cm=state.pose.z_cm)

        # OUTBOUND
        state.begin_phase(Phase.OUTBOUND, now())
        safety_monitor.max_phase_seconds = config.outbound_budget_sec
        _run_navigation_loop(
            state=state,
            vlm=vlm_outbound,
            goal_descriptor="magenta_kitchen_beacon",
            tello=tello,
            log=log,
            fr=fr,
            frame_source=frame_source,
            get_battery=get_battery,
            now=now,
            sleep=sleep,
            dry_run=config.dry_run,
            stop_phase=Phase.KITCHEN_CONFIRM,
            max_cycles=max_cycles,
        )

        if state.phase == Phase.KITCHEN_CONFIRM:
            log.event("kitchen_confirm", "begin")
            print("<!-- TTS: \"Kitchen reached.\" -->")
            sleep(config.kitchen_hover_sec)
            frame = _next_frame(fr, frame_source)
            if frame is not None:
                try:
                    log.frame("kitchen_confirm", 0, frame)
                except Exception as e:
                    log.event("kitchen_confirm", "frame_save_failed", err=str(e))
            log.event("kitchen_confirm", "complete")

            state.begin_phase(Phase.RETURN, now())
            safety_monitor.max_phase_seconds = config.return_budget_sec
            _run_navigation_loop(
                state=state,
                vlm=vlm_return,
                goal_descriptor="cyan_home_beacon",
                tello=tello,
                log=log,
                fr=fr,
                frame_source=frame_source,
                get_battery=get_battery,
                now=now,
                sleep=sleep,
                dry_run=config.dry_run,
                stop_phase=Phase.HOME_DETECTED,
                max_cycles=max_cycles,
            )

        if state.phase == Phase.HOME_DETECTED:
            log.event("home_detected", "begin", z_cm=state.pose.z_cm)
            while state.pose.z_cm > 30:
                if not config.dry_run:
                    try:
                        tello.move_down(config.descent_step_cm)
                    except Exception as e:
                        log.event("home_detected", "move_down_failed", err=str(e))
                        break
                state.pose.update_move("down", config.descent_step_cm)
                sleep(0.5)
            state.phase = Phase.LANDING

        # LANDING (covers normal end + aborted-to-landing)
        if state.phase in (Phase.LANDING, Phase.HOME_DETECTED, Phase.KITCHEN_CONFIRM, Phase.OUTBOUND, Phase.RETURN):
            state.phase = Phase.LANDING
            log.event("landing", "begin", abort_reason=state.abort_reason)
            if not config.dry_run:
                try:
                    tello.land()
                except Exception as e:
                    log.event("landing", "land_failed", err=str(e))

    except Exception as e:
        state.abort_reason = f"exception:{e}"
        log.event("error", "exception", err=str(e))
    finally:
        # Hard contract: restore Hue, safe-land, close log. ALWAYS.
        if hue is not None and snap is not None:
            try:
                hue.restore_scene(snap)
            except Exception as e:
                try:
                    log.event("post_flight", "hue_restore_failed", err=str(e))
                except Exception:
                    pass
        try:
            safe_land(tello, log, "finally_cleanup")
        except Exception:
            pass
        try:
            tello.streamoff()
        except Exception:
            pass
        try:
            state.final_battery = get_battery()
        except Exception:
            state.final_battery = None
        try:
            log.event(
                "post_flight",
                "complete",
                final_battery=state.final_battery,
                total_distance_cm=state.pose.distance_to_origin(),
                abort_reason=state.abort_reason,
                final_phase=state.phase.value,
            )
        except Exception:
            pass
        try:
            log.close()
        except Exception:
            pass

    return _finalize_result(state, state.final_battery)


def _next_frame(fr: Any, frame_source: Optional[Callable[[], Any]]) -> Any:
    if frame_source is not None:
        try:
            return frame_source()
        except Exception:
            return None
    if fr is None:
        return None
    return _grab_frame(fr)


def _run_navigation_loop(
    state: MissionState,
    vlm: Any,
    goal_descriptor: str,
    tello: Any,
    log: Any,
    fr: Any,
    frame_source: Optional[Callable[[], Any]],
    get_battery: Callable[[], int],
    now: Callable[[], float],
    sleep: Callable[[float], None],
    dry_run: bool,
    stop_phase: Phase,
    max_cycles: int,
) -> None:
    cycle = 0
    while state.phase not in (stop_phase, Phase.LANDING, Phase.ABORTED) and cycle < max_cycles:
        cycle += 1
        t_cycle = now()
        frame = _next_frame(fr, frame_source)

        if frame is None:
            log.event(state.phase.value.lower(), "frame_none", cycle=cycle)
            req = state.tick(False, get_battery(), None, t_cycle)
            _execute_action(tello, state.pose, req, dry_run, sleep)
            sleep(state.config.cycle_period_sec)
            continue

        battery = get_battery()
        elapsed = t_cycle - state.phase_start
        budget = state.phase_budget()

        try:
            decision = vlm.decide(
                frame_bgr=frame,
                goal_descriptor=goal_descriptor,
                history=state.history,
                pose=state.pose,
                battery_pct=battery,
                phase_elapsed_sec=elapsed,
                max_phase_sec=budget,
            )
        except Exception as e:
            decision = Decision(description="", action="HOVER", confidence=0.0, reason=f"vlm_exc:{e}", raw={"err": str(e)})

        try:
            log.frame(state.phase.value.lower(), cycle, frame)
        except Exception:
            pass
        log.event(
            state.phase.value.lower(),
            "decision",
            cycle=cycle,
            battery=battery,
            elapsed=elapsed,
            action=decision.action,
            description=decision.description,
            confidence=decision.confidence,
            reason=decision.reason,
            pose=state.pose.as_dict(),
        )

        req = state.tick(True, battery, decision, t_cycle)
        _execute_action(tello, state.pose, req, dry_run, sleep)

        if req.kind == "LAND":
            break

        dt = now() - t_cycle
        if dt < state.config.cycle_period_sec:
            sleep(state.config.cycle_period_sec - dt)


def _finalize_result(state: MissionState, battery: Optional[int]) -> dict:
    return {
        "phase": state.phase.value,
        "abort_reason": state.abort_reason,
        "final_battery": battery,
        "pose": state.pose.as_dict(),
        "total_distance_cm": state.pose.distance_to_origin(),
        "history_len": len(state.history),
    }


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Autonomous Tello kitchen-and-back mission.")
    p.add_argument("--home-beacon-light", default="Polaris", help="Star name from lib/hue_beacon.LIGHT_IDS")
    p.add_argument("--vlm-endpoint", default="http://100.94.176.110:11434/api/generate")
    p.add_argument("--dry-run", action="store_true", help="Skip takeoff/land/move primitives")
    return p.parse_args(argv)


def main() -> None:
    ns = parse_args()
    config = MissionConfig(
        home_beacon_light_name=ns.home_beacon_light,
        vlm_endpoint=ns.vlm_endpoint,
        dry_run=ns.dry_run,
    )
    if config.home_beacon_light_name not in LIGHT_IDS:
        print(f"unknown home beacon light {config.home_beacon_light_name!r}; pick one of {list(LIGHT_IDS.keys())}")
        sys.exit(2)
    result = run_mission(config)
    print(result)


if __name__ == "__main__":
    main()
