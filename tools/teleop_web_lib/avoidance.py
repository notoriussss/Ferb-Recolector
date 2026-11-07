"""
Geometry of driving around the cubes Auto must never push (pure logic: no
camera, no motors).

The robot slides sideways (Mecanum, the heading stays) until its straight
path is clear. Cubes are placed in the robot frame from their distance and
bearing: `ahead` along the heading, `left` across it (positive = left, like
the bearings). The path is the band `clearance_m` wide on each side of the
heading.
"""

import math

# Slide directions, same sign as the bearings.
SLIDE_LEFT = 1.0
SLIDE_RIGHT = -1.0


def ahead_and_left_m(cube) -> tuple[float, float]:
    """
    Where `cube` (distance_m, bearing_rad) is: metres ahead and to the left.
    """
    return (
        cube.distance_m * math.cos(cube.bearing_rad),
        cube.distance_m * math.sin(cube.bearing_rad),
    )


def separation_m(a, b) -> float:
    """
    Distance on the floor between two cubes seen by the camera.
    """
    ax, ay = ahead_and_left_m(a)
    bx, by = ahead_and_left_m(b)
    return math.hypot(ax - bx, ay - by)


def is_guarded(target, avoid: list, guard_m: float) -> bool:
    """
    True when a cube of `avoid` lies within `guard_m` of `target`: catching
    the target would push it too, from whatever side the robot comes.
    """
    return any(separation_m(target, cube) < guard_m for cube in avoid)


def cubes_in_path(
    cubes: list,
    clearance_m: float,
    reach_m: float,
    goal_m: float = math.inf,
) -> list:
    """
    The `cubes` a straight forward move would push: nearer than `reach_m`,
    within `clearance_m` of the heading, and not past the goal the robot
    stops at, `goal_m` ahead (a cube behind the target is not reached).
    """
    blocking = []
    for cube in cubes:
        ahead_m, left_m = ahead_and_left_m(cube)
        if cube.distance_m > reach_m or ahead_m > goal_m:
            continue
        if abs(left_m) <= clearance_m:
            blocking.append(cube)
    return blocking


def slide_to_clear_m(
    cubes: list,
    clearance_m: float,
    direction: float,
) -> float:
    """
    How far (m) to slide toward `direction` (SLIDE_LEFT or SLIDE_RIGHT)
    until none of `cubes` is in the path, passing the ones on that side
    too.
    """
    # Each cube blocks the slide over [u - c, u + c], u its offset toward
    # `direction`. Equal widths: sorted by start, one pass finds the gap.
    offsets = sorted(direction * ahead_and_left_m(cube)[1] for cube in cubes)
    slide_m = 0.0
    for offset_m in offsets:
        if offset_m - clearance_m <= slide_m <= offset_m + clearance_m:
            slide_m = offset_m + clearance_m
    return slide_m


def dodge_direction(cubes: list, clearance_m: float) -> float:
    """
    The side to slide toward to get past `cubes` (nearest first): the one
    that needs the shorter slide; on a tie, away from the nearest cube.
    """
    left_m = slide_to_clear_m(cubes, clearance_m, SLIDE_LEFT)
    right_m = slide_to_clear_m(cubes, clearance_m, SLIDE_RIGHT)
    if left_m != right_m:
        return SLIDE_LEFT if left_m < right_m else SLIDE_RIGHT
    nearest_left_m = ahead_and_left_m(cubes[0])[1] if cubes else 0.0
    return SLIDE_RIGHT if nearest_left_m >= 0 else SLIDE_LEFT
