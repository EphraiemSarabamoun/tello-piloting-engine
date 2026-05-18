"""Unit tests for lib.pose. Stdlib only."""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from lib.pose import Pose  # noqa: E402


def test_forward_increases_x_when_yaw_zero():
    p = Pose()
    p.update_move("forward", 50)
    assert math.isclose(p.x_cm, 50.0, abs_tol=1e-6)
    assert math.isclose(p.y_cm, 0.0, abs_tol=1e-6)
    assert math.isclose(p.z_cm, 0.0, abs_tol=1e-6)
    assert math.isclose(p.yaw_deg, 0.0, abs_tol=1e-6)


def test_rotate_cw_30_then_forward_50_lands_in_correct_xy():
    p = Pose()
    p.update_move("rotate_cw", 30)
    # CW reduces yaw → yaw = -30 deg ≡ 330 deg
    assert math.isclose(p.yaw_deg, 330.0, abs_tol=1e-6)
    p.update_move("forward", 50)
    expected_x = 50.0 * math.cos(math.radians(330.0))
    expected_y = 50.0 * math.sin(math.radians(330.0))
    assert math.isclose(p.x_cm, expected_x, abs_tol=1e-6)
    assert math.isclose(p.y_cm, expected_y, abs_tol=1e-6)


def test_back_inverse_of_forward():
    p = Pose()
    p.update_move("forward", 75)
    p.update_move("back", 75)
    assert math.isclose(p.x_cm, 0.0, abs_tol=1e-6)
    assert math.isclose(p.y_cm, 0.0, abs_tol=1e-6)


def test_back_inverse_of_forward_after_rotation():
    p = Pose()
    p.update_move("rotate_ccw", 47)
    p.update_move("forward", 60)
    p.update_move("back", 60)
    assert math.isclose(p.x_cm, 0.0, abs_tol=1e-6)
    assert math.isclose(p.y_cm, 0.0, abs_tol=1e-6)


def test_inverse_heading_simple():
    p = Pose(yaw_deg=0.0)
    assert math.isclose(p.inverse_heading_deg(), 180.0, abs_tol=1e-6)
    p2 = Pose(yaw_deg=270.0)
    assert math.isclose(p2.inverse_heading_deg(), 90.0, abs_tol=1e-6)
    p3 = Pose(yaw_deg=45.0)
    assert math.isclose(p3.inverse_heading_deg(), 225.0, abs_tol=1e-6)


def test_distance_to_origin():
    p = Pose(x_cm=3.0, y_cm=4.0, z_cm=0.0)
    assert math.isclose(p.distance_to_origin(), 5.0, abs_tol=1e-6)
    p2 = Pose(x_cm=1.0, y_cm=2.0, z_cm=2.0)
    assert math.isclose(p2.distance_to_origin(), 3.0, abs_tol=1e-6)
    p3 = Pose()
    assert math.isclose(p3.distance_to_origin(), 0.0, abs_tol=1e-6)


def test_yaw_wraps_to_zero_to_360():
    p = Pose()
    p.update_move("rotate_ccw", 400)
    assert 0.0 <= p.yaw_deg < 360.0
    assert math.isclose(p.yaw_deg, 40.0, abs_tol=1e-6)
    p2 = Pose()
    p2.update_move("rotate_cw", 400)
    assert 0.0 <= p2.yaw_deg < 360.0
    assert math.isclose(p2.yaw_deg, 320.0, abs_tol=1e-6)
