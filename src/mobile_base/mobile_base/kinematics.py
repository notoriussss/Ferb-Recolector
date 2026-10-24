"""
Mecanum inverse kinematics (X-roller layout). Pure logic, no rclpy.

REP 103 convention: vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise.
Wheel order: fl, fr, rl, rr.
"""

from dataclasses import dataclass

WHEELS = ("fl", "fr", "rl", "rr")


@dataclass(frozen=True)
class MecanumGeometry:
    """
    Chassis dimensions needed by the inverse kinematics.
    """

    wheel_radius_m: float
    half_wheelbase_m: float  # half the wheelbase (front-back)
    half_track_m: float  # half the track width (left-right)


def wheel_speeds(
    vx: float,
    vy: float,
    wz: float,
    geo: MecanumGeometry,
) -> dict[str, float]:
    """
    Angular speed of each wheel (rad/s) for a motion (m/s, m/s, rad/s).
    """
    k = geo.half_wheelbase_m + geo.half_track_m
    r = geo.wheel_radius_m
    return {
        "fl": (vx - vy - k * wz) / r,
        "fr": (vx + vy + k * wz) / r,
        "rl": (vx + vy - k * wz) / r,
        "rr": (vx - vy + k * wz) / r,
    }


def scale_to_limit(
    speeds: dict[str, float],
    max_abs: float,
) -> dict[str, float]:
    """
    If any wheel exceeds `max_abs`, scale all of them equally so the
    direction of motion is preserved.
    """
    peak = max(abs(v) for v in speeds.values())
    if peak <= max_abs or peak == 0.0:
        return dict(speeds)
    factor = max_abs / peak
    return {name: v * factor for name, v in speeds.items()}
