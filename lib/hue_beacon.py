"""Direct-curl Hue v2 CLIP client. Beacon control for indoor drone nav."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Optional

import requests

CONFIG_PATH = Path.home() / ".hue-mcp" / "config.json"
DEFAULT_TIMEOUT = 5.0

LIGHT_IDS: dict[str, str] = {
    "Sirius":     "5ad28e26-06c8-4def-89ff-d5ea21c3dd50",
    "Betelgeuse": "28730992-4532-4972-8b7c-c6b5169dff41",
    "Rigel":      "78ec9d5e-c8b8-4b07-b594-aa7b7c057a38",
    "Altair":     "080216b6-d2bc-4212-ac6d-710c1b0a280f",
    "Antares":    "be2e43ef-8f14-48a3-b45d-7da71d96562b",
    "Deneb":      "63d6b91a-fb7d-4c97-a797-0ba6726e7e89",
    "Polaris":    "7c09d4eb-2921-4064-ac88-3751868949ad",
    "Vega":       "83eb1afe-b3cd-4361-b2b9-6aed1a737225",
    "Capella":    "a96ac8b1-bd8b-4a10-adc1-68d3dc214702",
}

BEDROOM_LIGHTS: list[str] = [
    LIGHT_IDS["Sirius"],
    LIGHT_IDS["Betelgeuse"],
    LIGHT_IDS["Rigel"],
    LIGHT_IDS["Altair"],
    LIGHT_IDS["Antares"],
    LIGHT_IDS["Deneb"],
    LIGHT_IDS["Polaris"],
]

KITCHEN_LIGHTS: list[str] = [
    LIGHT_IDS["Vega"],
    LIGHT_IDS["Capella"],
]

ALL_LIGHTS: list[str] = BEDROOM_LIGHTS + KITCHEN_LIGHTS

MAGENTA_XY: tuple[float, float] = (0.45, 0.20)
CYAN_XY: tuple[float, float] = (0.17, 0.30)
RED_XY: tuple[float, float] = (0.675, 0.322)
WARM_WHITE_MIREK: int = 350


class HueBeacon:
    """Bridge client. Loads ~/.hue-mcp/config.json at init."""

    def __init__(self, config_path: Path | str = CONFIG_PATH, timeout: float = DEFAULT_TIMEOUT) -> None:
        cfg_path = Path(config_path)
        cfg = json.loads(cfg_path.read_text())
        self.bridge_ip: str = cfg["bridge_ip"]
        self.app_key: str = cfg["app_key"]
        self.timeout = timeout
        self.base = f"https://{self.bridge_ip}/clip/v2/resource"
        self.headers = {
            "hue-application-key": self.app_key,
            "Content-Type": "application/json",
        }

    def _get(self, path: str) -> dict:
        r = requests.get(f"{self.base}/{path}", headers=self.headers, verify=False, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def _put(self, path: str, payload: dict) -> dict:
        r = requests.put(f"{self.base}/{path}", headers=self.headers, json=payload, verify=False, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def snapshot_scene(self) -> dict:
        """GET on/brightness/xy for every tracked light, keyed by service id."""
        snap: dict = {}
        for lid in ALL_LIGHTS:
            data = self._get(f"light/{lid}")
            item = data.get("data", [{}])[0]
            on = item.get("on", {}).get("on", False)
            bri = item.get("dimming", {}).get("brightness", 50.0)
            xy_obj = item.get("color", {}).get("xy")
            mirek = item.get("color_temperature", {}).get("mirek")
            snap[lid] = {"on": on, "brightness": bri, "xy": xy_obj, "mirek": mirek}
        return snap

    def restore_scene(self, snapshot: dict) -> None:
        """Write each captured state back via PUT."""
        for lid, state in snapshot.items():
            payload: dict = {"on": {"on": bool(state.get("on", False))}}
            bri = state.get("brightness")
            if bri is not None:
                payload["dimming"] = {"brightness": float(bri)}
            xy = state.get("xy")
            mirek = state.get("mirek")
            if xy is not None:
                payload["color"] = {"xy": {"x": float(xy["x"]), "y": float(xy["y"])}}
            elif mirek is not None:
                payload["color_temperature"] = {"mirek": int(mirek)}
            try:
                self._put(f"light/{lid}", payload)
            except Exception:
                pass

    def set_light(
        self,
        light_service_id: str,
        on: bool,
        brightness: float,
        xy: Optional[tuple[float, float]] = None,
        mirek: Optional[int] = None,
    ) -> None:
        """Single-light set. xy overrides mirek if both supplied."""
        payload: dict = {"on": {"on": bool(on)}, "dimming": {"brightness": float(brightness)}}
        if xy is not None:
            payload["color"] = {"xy": {"x": float(xy[0]), "y": float(xy[1])}}
        elif mirek is not None:
            payload["color_temperature"] = {"mirek": int(mirek)}
        self._put(f"light/{light_service_id}", payload)

    def set_kitchen_magenta(self) -> None:
        """Vega + Capella to bright magenta."""
        for lid in KITCHEN_LIGHTS:
            self.set_light(lid, True, 100.0, xy=MAGENTA_XY)

    def set_home_cyan(self, light_service_id: str) -> None:
        """Make the chosen bedroom light the cyan home beacon."""
        self.set_light(light_service_id, True, 100.0, xy=CYAN_XY)

    def dim_others(self, except_ids: list[str]) -> None:
        """Every non-beacon light to warm-white at low brightness."""
        ex = set(except_ids)
        for lid in ALL_LIGHTS:
            if lid in ex:
                continue
            try:
                self.set_light(lid, True, 20.0, mirek=WARM_WHITE_MIREK)
            except Exception:
                pass

    def panic_red_flash(self, flashes: int = 3, on_ms: float = 0.25, off_ms: float = 0.25) -> None:
        """Flash kitchen lights bright red. ABORT-state UX cue."""
        for _ in range(flashes):
            for lid in KITCHEN_LIGHTS:
                try:
                    self.set_light(lid, True, 100.0, xy=RED_XY)
                except Exception:
                    pass
            time.sleep(on_ms)
            for lid in KITCHEN_LIGHTS:
                try:
                    self.set_light(lid, False, 100.0)
                except Exception:
                    pass
            time.sleep(off_ms)
