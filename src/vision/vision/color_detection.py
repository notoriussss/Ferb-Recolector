"""Detección de cubos por color (HSV) y geometría de la cámara. Lógica pura, sin rclpy.

Flujo: imagen BGR -> máscara HSV por color -> contornos -> filtros -> lista de `Detection`
con el ángulo y la distancia aproximada sobre el piso (cámara inclinada hacia abajo).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# Color de cada caja en la imagen de depuración (BGR)
DRAW_COLORS_BGR = {
    "rojo": (40, 40, 230),
    "naranja": (0, 140, 255),
    "amarillo": (0, 230, 255),
    "verde": (60, 200, 60),
    "azul": (230, 120, 30),
    "morado": (160, 40, 140),
}


@dataclass(frozen=True)
class HsvRange:
    """Un rango HSV de OpenCV (H 0–179, S y V 0–255)."""

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


def parse_hsv_ranges(values: list[int] | tuple[int, ...]) -> list[HsvRange]:
    """Convierte una lista plana [h,s,v,h,s,v, ...] (múltiplo de 6) en rangos.

    Cada grupo de 6 números es un rango: mínimo (h, s, v) y máximo (h, s, v).
    El rojo usa dos rangos porque el tono da la vuelta (cerca de 0 y cerca de 179).
    """
    values = [int(x) for x in values]
    if not values or len(values) % 6 != 0:
        raise ValueError(f"se esperaban grupos de 6 números (h,s,v mín y máx), llegó {values}")
    ranges = []
    for i in range(0, len(values), 6):
        lo = tuple(values[i : i + 3])
        hi = tuple(values[i + 3 : i + 6])
        if not all(0 <= a <= b for a, b in zip(lo, hi, strict=True)):
            raise ValueError(f"rango HSV inválido: {lo} > {hi}")
        if hi[0] > 179 or hi[1] > 255 or hi[2] > 255:
            raise ValueError(f"rango HSV fuera de límites: {hi}")
        ranges.append(HsvRange(lo, hi))
    return ranges


@dataclass(frozen=True)
class CameraModel:
    """Cámara pinhole a cierta altura, inclinada hacia abajo, sin giro lateral.

    Ejes del robot: x adelante, y a la izquierda. El piso es el plano z = 0.
    """

    width_px: int
    height_px: int
    hfov_rad: float
    height_m: float
    tilt_rad: float  # positivo = mirando hacia abajo

    @property
    def focal_px(self) -> float:
        return (self.width_px / 2.0) / math.tan(self.hfov_rad / 2.0)

    def ground_point(self, u: float, v: float) -> tuple[float, float, float] | None:
        """Punto del piso que se ve en el píxel (u, v).

        Devuelve (x_m, y_m, depth_m): posición respecto a la cámara (proyectada en el piso)
        y la profundidad a lo largo del eje óptico. None si el píxel está sobre el horizonte.
        """
        f = self.focal_px
        left = (self.width_px / 2.0 - u) / f
        down = (v - self.height_px / 2.0) / f
        c, s = math.cos(self.tilt_rad), math.sin(self.tilt_rad)
        forward_w = c - s * down
        down_w = s + c * down
        if down_w <= 1e-6:
            return None
        scale = self.height_m / down_w
        return scale * forward_w, scale * left, scale

    def bearing_distance(self, u: float, v: float) -> tuple[float, float] | None:
        """(ángulo_rad, distancia_m) del punto del piso en (u, v). Ángulo positivo = izquierda."""
        p = self.ground_point(u, v)
        if p is None:
            return None
        x, y, _ = p
        return math.atan2(y, x), math.hypot(x, y)


@dataclass(frozen=True)
class DetectionFilter:
    """Filtros para descartar manchas que no son cubos."""

    min_area_px: int = 40
    cube_size_m: float = 0.025
    # ancho visto / ancho esperado a esa distancia: fuera de este rango no es un cubo
    min_size_ratio: float = 0.45
    max_size_ratio: float = 2.2
    # descarta manchas que tocan el borde de abajo (la pala se ve ahí)
    ignore_touching_bottom: bool = True
    max_distance_m: float = 2.5


@dataclass
class Detection:
    """Un cubo visto en la imagen."""

    color: str
    u: int  # columna del centro (px)
    v: int  # fila del borde de abajo (px), donde el cubo toca el piso
    width_px: int
    height_px: int
    bbox: tuple[int, int, int, int]  # x, y, w, h
    bearing_rad: float = 0.0
    distance_m: float = math.inf


@dataclass
class ColorDetector:
    """Detector de cubos de varios colores con su modelo de cámara."""

    ranges: dict[str, list[HsvRange]]
    camera: CameraModel
    filters: DetectionFilter = field(default_factory=DetectionFilter)
    morph_kernel_px: int = 3

    def mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """Máscara binaria (0/255) de un color en una imagen HSV."""
        total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in self.ranges[color]:
            total |= cv2.inRange(hsv, np.array(r.lower), np.array(r.upper))
        if self.morph_kernel_px > 1:
            k = np.ones((self.morph_kernel_px, self.morph_kernel_px), np.uint8)
            total = cv2.morphologyEx(total, cv2.MORPH_OPEN, k)
        return total

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """Busca cubos de todos los colores. Ordenados del más cercano al más lejano."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        img_h = bgr.shape[0]
        found: list[Detection] = []
        for color in self.ranges:
            contours, _ = cv2.findContours(
                self.mask(hsv, color), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                if cv2.contourArea(cnt) < self.filters.min_area_px:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if self.filters.ignore_touching_bottom and y + h >= img_h - 1:
                    continue
                det = Detection(color, x + w // 2, y + h - 1, w, h, (x, y, w, h))
                if self._locate(det):
                    found.append(det)
        found.sort(key=lambda d: d.distance_m)
        return found

    def _locate(self, det: Detection) -> bool:
        """Calcula ángulo y distancia; devuelve False si la mancha no parece un cubo."""
        cam = self.camera
        p = cam.ground_point(det.u, det.v)
        if p is None:
            return False
        x, y, depth = p
        det.bearing_rad = math.atan2(y, x)
        det.distance_m = math.hypot(x, y)
        if det.distance_m > self.filters.max_distance_m:
            return False
        expected_px = cam.focal_px * self.filters.cube_size_m / depth
        ratio = det.width_px / expected_px
        return self.filters.min_size_ratio <= ratio <= self.filters.max_size_ratio


def draw_detections(
    bgr: np.ndarray,
    detections: list[Detection],
    highlight: Detection | None = None,
) -> np.ndarray:
    """Copia de la imagen con una caja y una etiqueta por cubo. `highlight` va más grueso."""
    out = bgr.copy()
    for det in detections:
        x, y, w, h = det.bbox
        color = DRAW_COLORS_BGR.get(det.color, (255, 255, 255))
        thick = 3 if det is highlight else 1
        cv2.rectangle(out, (x, y), (x + w, y + h), color, thick)
        label = f"{det.color} {det.distance_m * 100:.0f}cm {math.degrees(det.bearing_rad):+.0f}"
        cv2.putText(out, label, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(out, label, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    if highlight is not None:
        cv2.drawMarker(out, (highlight.u, highlight.v), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    return out
