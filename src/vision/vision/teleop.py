"""Control manual desde el visor web: teclas → velocidades. Lógica pura, sin rclpy.

La página manda cada eje en [-1, 1] (W/S, A/D, Q/E) y una velocidad en [0, 1];
aquí se pasan a m/s y rad/s con los topes de `teleop_max_*`.
Convención REP 103: vx > 0 adelante, vy > 0 a la izquierda, wz > 0 antihorario.
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
        raise ValueError("valor no finito")
    return max(lo, min(1.0, x))


def teleop_velocity(data: dict, limits: TeleopLimits) -> tuple[float, float, float]:
    """(vx m/s, vy m/s, wz rad/s) a partir de {"vx", "vy", "wz", "speed"} de la página."""
    speed = _unit(data.get("speed", 0.0), lo=0.0)
    vx = _unit(data.get("vx", 0.0)) * speed * limits.max_linear_m_s
    vy = _unit(data.get("vy", 0.0)) * speed * limits.max_linear_m_s
    wz = _unit(data.get("wz", 0.0)) * speed * limits.max_angular_rad_s
    return vx, vy, wz
