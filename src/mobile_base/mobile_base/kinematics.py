"""Cinemática inversa mecanum (rodillos en X). Lógica pura, sin rclpy.

Convención REP 103: vx > 0 adelante, vy > 0 a la izquierda, wz > 0 antihorario.
Orden de ruedas: fl, fr, rl, rr.
"""

from dataclasses import dataclass

WHEELS = ("fl", "fr", "rl", "rr")


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius_m: float
    half_wheelbase_m: float  # mitad de la distancia entre ejes (adelante-atrás)
    half_track_m: float  # mitad de la trocha (izquierda-derecha)


def wheel_speeds(vx: float, vy: float, wz: float, geo: MecanumGeometry) -> dict[str, float]:
    """Velocidad angular de cada rueda (rad/s) para un movimiento (m/s, m/s, rad/s)."""
    k = geo.half_wheelbase_m + geo.half_track_m
    r = geo.wheel_radius_m
    return {
        "fl": (vx - vy - k * wz) / r,
        "fr": (vx + vy + k * wz) / r,
        "rl": (vx + vy - k * wz) / r,
        "rr": (vx - vy + k * wz) / r,
    }


def scale_to_limit(speeds: dict[str, float], max_abs: float) -> dict[str, float]:
    """Si alguna rueda pasa de `max_abs`, escala todas por igual (se conserva la dirección)."""
    peak = max(abs(v) for v in speeds.values())
    if peak <= max_abs or peak == 0.0:
        return dict(speeds)
    f = max_abs / peak
    return {n: v * f for n, v in speeds.items()}
