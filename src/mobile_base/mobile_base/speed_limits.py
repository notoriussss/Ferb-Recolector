"""Límite de velocidad según la distancia del ultrasonido. Lógica pura, sin rclpy.

Niveles del plan: más de `slow_distance_m` normal, entre `stop_distance_m` y
`slow_distance_m` a la mitad, menos de `stop_distance_m` parado. Solo limita el avance
(vx > 0): retroceder, ir de lado y girar se permiten para poder salir.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class GuardParams:
    slow_distance_m: float = 0.30
    stop_distance_m: float = 0.15
    slow_factor: float = 0.5


def limit_forward(vx: float, range_m: float | None, p: GuardParams) -> float:
    """Velocidad de avance permitida. `range_m` None = no hay lectura reciente: no avanza."""
    if vx <= 0.0:
        return vx
    if range_m is None or range_m < p.stop_distance_m:
        return 0.0
    if range_m < p.slow_distance_m:
        return vx * p.slow_factor
    return vx
