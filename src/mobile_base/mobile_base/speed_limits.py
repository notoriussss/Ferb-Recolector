"""Speed limit based on the ultrasonic distance. Pure logic, no rclpy.

Levels from the plan: beyond `slow_distance_m` normal, between `stop_distance_m` and
`slow_distance_m` half speed, below `stop_distance_m` stopped. Only forward motion is limited
(vx > 0): reversing, strafing and turning are allowed so the robot can get out.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class GuardParams:
    slow_distance_m: float = 0.30
    stop_distance_m: float = 0.15
    slow_factor: float = 0.5


def limit_forward(vx: float, range_m: float | None, p: GuardParams) -> float:
    """Allowed forward speed. `range_m` None = no recent reading: no forward motion."""
    if vx <= 0.0:
        return vx
    if range_m is None or range_m < p.stop_distance_m:
        return 0.0
    if range_m < p.slow_distance_m:
        return vx * p.slow_factor
    return vx
