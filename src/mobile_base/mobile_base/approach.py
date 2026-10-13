"""Acercarse a un cubo de los colores objetivo y detenerse a cierta distancia.

Es el primer pedazo de la máquina de estados (BUSCAR -> ACERCARSE -> LLEGUÉ), sin la pala.
Lógica pura, sin rclpy: el nodo behavior_fsm le pasa los cubos vistos y publica el comando.
"""

import math
from dataclasses import dataclass, field
from enum import StrEnum


class State(StrEnum):
    PAUSED = "PAUSA"
    SEARCH = "BUSCAR"
    APPROACH = "ACERCARSE"
    ARRIVED = "LLEGUE"


@dataclass(frozen=True)
class Cube:
    color: str
    bearing_rad: float  # positivo = a la izquierda
    distance_m: float


@dataclass
class ApproachParams:
    target_colors: list[str] = field(default_factory=lambda: ["rojo", "naranja", "amarillo"])
    stop_distance_m: float = 0.17  # se detiene cuando el cubo está a esta distancia de la cámara
    resume_margin_m: float = 0.04  # si el cubo se aleja más que esto, vuelve a acercarse
    slow_down_distance_m: float = 0.45  # desde aquí empieza a frenar
    max_forward_m_s: float = 0.12
    min_forward_m_s: float = 0.03
    turn_gain: float = 1.5  # rad/s por rad de error
    max_turn_rad_s: float = 0.8
    align_tolerance_rad: float = 0.35  # con más error que esto, gira sin avanzar
    search_turn_rad_s: float = 0.5
    lost_timeout_s: float = 1.0  # tiempo sin ver el cubo antes de volver a buscar


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    wz: float = 0.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ApproachController:
    def __init__(self, params: ApproachParams | None = None):
        self.params = params or ApproachParams()
        self.state = State.SEARCH
        self.enabled = True
        self.target: Cube | None = None
        self._last_seen_s: float | None = None
        self._search_dir = 1.0

    def pick_target(self, cubes: list[Cube]) -> Cube | None:
        """El cubo más cercano de los colores objetivo."""
        wanted = set(self.params.target_colors)
        candidates = [c for c in cubes if c.color in wanted]
        return min(candidates, key=lambda c: c.distance_m, default=None)

    def update(self, cubes: list[Cube], now_s: float) -> Command:
        p = self.params
        if not self.enabled:
            self.state = State.PAUSED
            self.target = None
            return Command()
        if self.state == State.PAUSED:
            self.state = State.SEARCH

        cube = self.pick_target(cubes)
        if cube is not None:
            self.target = cube
            self._last_seen_s = now_s
            self._search_dir = 1.0 if cube.bearing_rad >= 0 else -1.0
            if self.state == State.ARRIVED:
                if cube.distance_m <= p.stop_distance_m + p.resume_margin_m:
                    return Command()
            elif cube.distance_m <= p.stop_distance_m:
                self.state = State.ARRIVED
                return Command()
            self.state = State.APPROACH
            return self._drive_to(cube)

        # no se ve ningún cubo objetivo
        lost_for = math.inf if self._last_seen_s is None else now_s - self._last_seen_s
        if lost_for < p.lost_timeout_s and self.state in (State.APPROACH, State.ARRIVED):
            # puede ser un cuadro malo: espera quieto un momento
            return Command()
        self.state = State.SEARCH
        self.target = None
        return Command(0.0, self._search_dir * p.search_turn_rad_s)

    def _drive_to(self, cube: Cube) -> Command:
        p = self.params
        wz = _clamp(p.turn_gain * cube.bearing_rad, -p.max_turn_rad_s, p.max_turn_rad_s)
        if abs(cube.bearing_rad) > p.align_tolerance_rad:
            return Command(0.0, wz)
        span = max(1e-6, p.slow_down_distance_m - p.stop_distance_m)
        frac = _clamp((cube.distance_m - p.stop_distance_m) / span, 0.0, 1.0)
        vx = p.min_forward_m_s + frac * (p.max_forward_m_s - p.min_forward_m_s)
        # menos avance cuanto peor alineado
        vx *= 1.0 - abs(cube.bearing_rad) / p.align_tolerance_rad * 0.5
        return Command(vx, wz)
