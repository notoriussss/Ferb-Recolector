"""
Tests for the geometry of driving around the cubes to avoid (run with:
pytest tools).
"""

import math

import pytest

from teleop_web_lib.avoidance import (
    SLIDE_LEFT,
    SLIDE_RIGHT,
    ahead_and_left_m,
    cubes_in_path,
    dodge_direction,
    is_guarded,
    separation_m,
    slide_to_clear_m,
)

CLEARANCE_M = 0.12
REACH_M = 0.30


class Cube:
    """
    A cube seen at `distance_m` and `bearing_rad` (positive = left).
    """

    def __init__(self, distance_m, bearing_rad=0.0):
        self.distance_m = distance_m
        self.bearing_rad = bearing_rad


def at(ahead_m, left_m):
    """
    The cube at that spot of the robot frame.
    """
    return Cube(math.hypot(ahead_m, left_m), math.atan2(left_m, ahead_m))


def test_a_cube_is_placed_ahead_and_to_its_side():
    ahead_m, left_m = ahead_and_left_m(Cube(0.2, math.pi / 2))
    assert ahead_m == pytest.approx(0.0, abs=1e-9)
    assert left_m == pytest.approx(0.2)
    assert separation_m(at(0.2, 0.05), at(0.2, -0.05)) == pytest.approx(0.1)


def test_only_cubes_near_ahead_and_inside_the_band_block():
    ahead = at(0.2, 0.05)
    beside = at(0.2, 0.2)
    far = at(0.5, 0.0)
    assert cubes_in_path([ahead, beside, far], CLEARANCE_M, REACH_M) == [
        ahead,
    ]


def test_a_cube_behind_the_target_is_not_in_the_path():
    behind = at(0.25, 0.0)
    assert cubes_in_path([behind], CLEARANCE_M, REACH_M, goal_m=0.18) == []
    assert cubes_in_path([behind], CLEARANCE_M, REACH_M, goal_m=0.28) == [
        behind,
    ]


def test_a_target_next_to_a_cube_to_avoid_is_guarded():
    target = at(0.3, 0.0)
    assert is_guarded(target, [at(0.3, 0.05)], CLEARANCE_M)
    assert not is_guarded(target, [at(0.3, 0.2)], CLEARANCE_M)
    assert not is_guarded(target, [], CLEARANCE_M)


def test_the_slide_clears_the_cube_by_the_clearance():
    cube = at(0.2, 0.03)  # a bit to the left: short slide to the right
    assert slide_to_clear_m([cube], CLEARANCE_M, SLIDE_RIGHT) == (
        pytest.approx(0.09)
    )
    assert slide_to_clear_m([cube], CLEARANCE_M, SLIDE_LEFT) == (
        pytest.approx(0.15)
    )
    assert slide_to_clear_m([at(0.2, 0.3)], CLEARANCE_M, SLIDE_LEFT) == 0.0


def test_the_slide_also_passes_a_second_cube_on_that_side():
    # Sliding left past the first lands in front of the second.
    cubes = [at(0.2, 0.03), at(0.25, 0.2)]
    assert slide_to_clear_m(cubes, CLEARANCE_M, SLIDE_LEFT) == (
        pytest.approx(0.32)
    )


def test_it_slides_away_from_a_lone_cube():
    assert dodge_direction([at(0.2, 0.03)], CLEARANCE_M) == SLIDE_RIGHT
    assert dodge_direction([at(0.2, -0.03)], CLEARANCE_M) == SLIDE_LEFT
    assert dodge_direction([at(0.2, 0.0)], CLEARANCE_M) == SLIDE_RIGHT


def test_it_does_not_slide_into_another_cube():
    # Away from the nearest is right, but a second cube waits there.
    cubes = [at(0.2, 0.03), at(0.25, -0.16)]
    assert dodge_direction(cubes, CLEARANCE_M) == SLIDE_LEFT
