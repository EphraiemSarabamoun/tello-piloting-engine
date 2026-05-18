"""Offline tests for kitchen_simple.ask_vlm_color (HSV-backed beacon detection)."""

import os
import sys

import cv2
import numpy as np
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from kitchen_simple import (  # noqa: E402
    COLOR_REACHED_PIXEL_THRESHOLD,
    OUTBOUND_PROMPT,
    RETURN_PROMPT,
    _count_color_pixels,
    ask_vlm_color,
)

W, H = 960, 720  # match Tello forward-cam resolution


def make_blob_frame(color_bgr, w=W, h=H, blob_w=600, blob_h=400):
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    cx, cy = w // 2, h // 2
    half_w, half_h = blob_w // 2, blob_h // 2
    frame[cy - half_h:cy + half_h, cx - half_w:cx + half_w] = color_bgr
    return frame


def make_dim_blob_frame(color_bgr, w=W, h=H):
    # Same blob size as the bright case (240000 px) but dim — fails the V>80 floor.
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    cy, cx = h // 2, w // 2
    dim = tuple(int(c * 0.20) for c in color_bgr)  # 20% brightness → V ~ 30-40
    frame[cy - 200:cy + 200, cx - 300:cx + 300] = dim
    return frame


def test_magenta_blob_triggers_reached():
    frame = make_blob_frame((255, 0, 255))  # BGR magenta
    out = ask_vlm_color(frame, OUTBOUND_PROMPT)
    assert out["reached"] is True
    assert "magenta_px=" in out["note"]
    n = _count_color_pixels(frame, "magenta")
    assert n >= COLOR_REACHED_PIXEL_THRESHOLD


def test_cyan_blob_triggers_reached_for_cyan_prompt():
    frame = make_blob_frame((255, 255, 0))  # BGR cyan
    out = ask_vlm_color(frame, RETURN_PROMPT)
    assert out["reached"] is True
    assert "cyan_px=" in out["note"]
    n = _count_color_pixels(frame, "cyan")
    assert n >= COLOR_REACHED_PIXEL_THRESHOLD


def test_dim_magenta_does_not_trigger():
    frame = make_dim_blob_frame((255, 0, 255))  # very dim magenta — fails V>80
    out = ask_vlm_color(frame, OUTBOUND_PROMPT)
    assert out["reached"] is False
    n = _count_color_pixels(frame, "magenta")
    assert n < COLOR_REACHED_PIXEL_THRESHOLD


def test_non_beacon_scene_does_not_trigger():
    """Simulate a typical indoor frame: warm-white walls + some saturated non-beacon colors.

    Uniform RGB noise has too much energy in the magenta/cyan hue bands to be a realistic
    'no beacon' frame, so we synthesize a scene composed of greens, yellows, oranges, and
    warm-white (the dominant indoor palette) instead.
    """
    rng = np.random.default_rng(seed=1)
    # Warm-white base (BGR roughly (200, 220, 240), low saturation in HSV)
    frame = np.full((H, W, 3), (210, 220, 235), dtype=np.uint8)
    # Add patches of green, yellow, orange — all outside the magenta/cyan hue bands.
    palette_bgr = [(0, 180, 0), (0, 220, 220), (0, 140, 230), (40, 90, 160)]
    for _ in range(40):
        cx = int(rng.integers(60, W - 60))
        cy = int(rng.integers(60, H - 60))
        bw = int(rng.integers(40, 160))
        bh = int(rng.integers(40, 160))
        col = palette_bgr[int(rng.integers(0, len(palette_bgr)))]
        frame[max(0, cy - bh):cy + bh, max(0, cx - bw):cx + bw] = col
    out_mg = ask_vlm_color(frame, OUTBOUND_PROMPT)
    out_cy = ask_vlm_color(frame, RETURN_PROMPT)
    assert out_mg["reached"] is False, f"unexpected magenta trigger: {out_mg['note']}"
    assert out_cy["reached"] is False, f"unexpected cyan trigger: {out_cy['note']}"


def test_threshold_override_is_respected():
    frame = make_blob_frame((255, 0, 255))
    n = _count_color_pixels(frame, "magenta")
    # Set threshold one above the actual count: should NOT trigger.
    out = ask_vlm_color(frame, OUTBOUND_PROMPT, threshold=n + 1)
    assert out["reached"] is False
    assert f"thresh={n + 1}" in out["note"]
    # Set threshold one below: SHOULD trigger.
    out = ask_vlm_color(frame, OUTBOUND_PROMPT, threshold=max(0, n - 1))
    assert out["reached"] is True


def test_debug_mode_includes_hsv_stats():
    frame = make_blob_frame((255, 0, 255))
    out = ask_vlm_color(frame, OUTBOUND_PROMPT, debug=True)
    assert out["reached"] is True
    assert " h=" in out["note"] and " s=" in out["note"] and " v=" in out["note"]


def test_hue_wrap_low_end_caught():
    # Hue near 0 should still register as magenta (red-pink wrap).
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    # OpenCV HSV: build a pixel with H=5, S=200, V=200 → convert back to BGR for the test frame
    hsv_color = np.array([[[5, 200, 200]]], dtype=np.uint8)
    bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
    cy, cx = H // 2, W // 2
    frame[cy - 200:cy + 200, cx - 300:cx + 300] = bgr_color
    n = _count_color_pixels(frame, "magenta")
    assert n >= COLOR_REACHED_PIXEL_THRESHOLD


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
