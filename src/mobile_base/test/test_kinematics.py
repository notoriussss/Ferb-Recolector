"""Tests: mecanum kinematics (forward, strafe, turn)."""

import pytest

from mobile_base.kinematics import MecanumGeometry, scale_to_limit, wheel_speeds

GEO = MecanumGeometry(wheel_radius_m=0.0325, half_wheelbase_m=0.0605, half_track_m=0.0655)


def test_forward_all_wheels_same():
    s = wheel_speeds(0.1, 0.0, 0.0, GEO)
    assert all(v == pytest.approx(0.1 / 0.0325) for v in s.values())


def test_strafe_left_diagonal_pairs():
    s = wheel_speeds(0.0, 0.1, 0.0, GEO)
    assert s["fl"] < 0 and s["rr"] < 0
    assert s["fr"] > 0 and s["rl"] > 0
    assert s["fl"] == pytest.approx(-s["fr"])


def test_turn_left_right_side_forward():
    s = wheel_speeds(0.0, 0.0, 1.0, GEO)
    assert s["fr"] > 0 and s["rr"] > 0
    assert s["fl"] < 0 and s["rl"] < 0
    assert s["fr"] == pytest.approx((0.0605 + 0.0655) / 0.0325)


def test_scale_keeps_ratios():
    s = scale_to_limit({"fl": 10.0, "fr": -5.0, "rl": 2.0, "rr": 0.0}, 5.0)
    assert s == pytest.approx({"fl": 5.0, "fr": -2.5, "rl": 1.0, "rr": 0.0})


def test_scale_does_nothing_below_limit():
    s = {"fl": 1.0, "fr": -1.0, "rl": 0.5, "rr": 0.0}
    assert scale_to_limit(s, 5.0) == s
