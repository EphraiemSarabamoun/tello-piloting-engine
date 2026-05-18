"""Phase 3: goal-directed mission within the same room. Short budgets, delegates to kitchen.run_mission.

Run on loki (joined to Tello AP):
    uv run python phase3.py [--home-beacon-light Polaris] [--vlm-endpoint URL] [--dry-run]
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from kitchen import MissionConfig, run_mission
from lib.hue_beacon import LIGHT_IDS


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Phase 3: short-budget kitchen-and-back, same room.")
    p.add_argument("--home-beacon-light", default="Polaris")
    p.add_argument("--vlm-endpoint", default="http://100.94.176.110:11434/api/generate")
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args(argv)


def run() -> int:
    ns = parse_args()
    if ns.home_beacon_light not in LIGHT_IDS:
        print(f"unknown home beacon {ns.home_beacon_light!r}; choose from {list(LIGHT_IDS.keys())}")
        return 2

    config = MissionConfig(
        home_beacon_light_name=ns.home_beacon_light,
        vlm_endpoint=ns.vlm_endpoint,
        dry_run=ns.dry_run,
        outbound_budget_sec=60.0,
        return_budget_sec=60.0,
        max_distance_cm=600.0,
    )
    result = run_mission(config)
    print(f"Phase 3 result: {result}")
    abort = result.get("abort_reason")
    final_phase = result.get("phase")
    if abort is None and final_phase in ("LANDING", "POST_FLIGHT"):
        print("PASS: phase 3 completed without abort.")
        return 0
    print(f"FAIL: final_phase={final_phase}, abort_reason={abort}")
    return 1


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
