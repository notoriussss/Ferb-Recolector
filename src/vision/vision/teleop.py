"""Manual control from the web viewer: keys -> velocities. Pure logic, no rclpy.

The page sends each axis in [-1, 1] (W/S, A/D, Q/E) and a speed in [0, 1];
here they are converted to m/s and rad/s using the `teleop_max_*` caps.
REP 103 convention: vx > 0 forward, vy > 0 left, wz > 0 counter-clockwise.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TeleopLimits:
    max_linear_m_s: float
    max_angular_rad_s: float


def _unit(x, lo: float = -1.0) -> float:
    x = float(x)
    if not math.isfinite(x):
        raise ValueError("non-finite value")
    return max(lo, min(1.0, x))


def teleop_velocity(data: dict, limits: TeleopLimits) -> tuple[float, float, float]:
    """(vx m/s, vy m/s, wz rad/s) from the page's {"vx", "vy", "wz", "speed"}."""
    speed = _unit(data.get("speed", 0.0), lo=0.0)
    vx = _unit(data.get("vx", 0.0)) * speed * limits.max_linear_m_s
    vy = _unit(data.get("vy", 0.0)) * speed * limits.max_linear_m_s
    wz = _unit(data.get("wz", 0.0)) * speed * limits.max_angular_rad_s
    return vx, vy, wz
