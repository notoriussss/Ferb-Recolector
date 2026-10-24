"""Cube detection by colour (HSV) and camera geometry. Pure logic, no rclpy.

Flow: BGR image -> HSV mask per colour -> contours -> filters -> list of `Detection`
with the bearing and approximate distance on the floor (camera tilted down).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# Box colour for each class in the debug image (BGR)
DRAW_COLORS_BGR = {
    "red": (40, 40, 230),
    "orange": (0, 140, 255),
    "yellow": (0, 230, 255),
    "green": (60, 200, 60),
    "blue": (230, 120, 30),
    "purple": (160, 40, 140),
}


@dataclass(frozen=True)
class HsvRange:
    """One OpenCV HSV range (H 0–179, S and V 0–255)."""

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


def parse_hsv_ranges(values: list[int] | tuple[int, ...]) -> list[HsvRange]:
    """Converts a flat list [h,s,v,h,s,v, ...] (multiple of 6) into ranges.

    Each group of 6 numbers is one range: minimum (h, s, v) and maximum (h, s, v).
    Red uses two ranges because the hue wraps around (near 0 and near 179).
    """
    values = [int(x) for x in values]
    if not values or len(values) % 6 != 0:
        raise ValueError(f"expected groups of 6 numbers (h,s,v min and max), got {values}")
    ranges = []
    for i in range(0, len(values), 6):
        lo = tuple(values[i : i + 3])
        hi = tuple(values[i + 3 : i + 6])
        if not all(0 <= a <= b for a, b in zip(lo, hi, strict=True)):
            raise ValueError(f"invalid HSV range: {lo} > {hi}")
        if hi[0] > 179 or hi[1] > 255 or hi[2] > 255:
            raise ValueError(f"HSV range out of bounds: {hi}")
        ranges.append(HsvRange(lo, hi))
    return ranges


@dataclass(frozen=True)
class CameraModel:
    """Pinhole camera at some height, tilted down, with no sideways rotation.

    Robot axes: x forward, y to the left. The floor is the plane z = 0.
    """

    width_px: int
    height_px: int
    hfov_rad: float
    height_m: float
    tilt_rad: float  # positive = looking down

    @property
    def focal_px(self) -> float:
        return (self.width_px / 2.0) / math.tan(self.hfov_rad / 2.0)

    def ground_point(self, u: float, v: float) -> tuple[float, float, float] | None:
        """Floor point seen at pixel (u, v).

        Returns (x_m, y_m, depth_m): position relative to the camera (projected on the floor)
        and the depth along the optical axis. None if the pixel is above the horizon.
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
        """(bearing_rad, distance_m) of the floor point at (u, v). Positive bearing = left."""
        p = self.ground_point(u, v)
        if p is None:
            return None
        x, y, _ = p
        return math.atan2(y, x), math.hypot(x, y)


@dataclass(frozen=True)
class DetectionFilter:
    """Filters that discard blobs that are not cubes."""

    min_area_px: int = 40
    cube_size_m: float = 0.025
    # seen width / expected width at that distance: outside this range it is not a cube
    min_size_ratio: float = 0.45
    max_size_ratio: float = 2.2
    # discard blobs touching the bottom edge (the scoop shows there)
    ignore_touching_bottom: bool = True
    max_distance_m: float = 2.5


@dataclass
class Detection:
    """A cube seen in the image."""

    color: str
    u: int  # centre column (px)
    v: int  # bottom edge row (px), where the cube touches the floor
    width_px: int
    height_px: int
    bbox: tuple[int, int, int, int]  # x, y, w, h
    bearing_rad: float = 0.0
    distance_m: float = math.inf


@dataclass(frozen=True)
class Region:
    """A large colour blob (the delivery line), not a cube."""

    color: str
    u: int  # column of the nearest point (px)
    v: int  # lowest row of the blob (px)
    area_px: int
    bbox: tuple[int, int, int, int]
    bearing_rad: float
    distance_m: float


@dataclass
class ColorDetector:
    """Multi-colour cube detector with its camera model."""

    ranges: dict[str, list[HsvRange]]
    camera: CameraModel
    filters: DetectionFilter = field(default_factory=DetectionFilter)
    morph_kernel_px: int = 3

    def mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """Binary mask (0/255) of one colour in an HSV image."""
        total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in self.ranges[color]:
            total |= cv2.inRange(hsv, np.array(r.lower), np.array(r.upper))
        if self.morph_kernel_px > 1:
            k = np.ones((self.morph_kernel_px, self.morph_kernel_px), np.uint8)
            total = cv2.morphologyEx(total, cv2.MORPH_OPEN, k)
        return total

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """Finds cubes of every colour, sorted from nearest to farthest."""
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

    def find_region(self, bgr: np.ndarray, min_area_px: int) -> tuple[Region | None, np.ndarray]:
        """The largest blob of any of the colours (e.g. the delivery line).

        Returns the region (with bearing and distance of its nearest point) and the mask of
        all colours combined, to tell which cubes are on the zone.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        best = None
        for color in self.ranges:
            m = self.mask(hsv, color)
            total |= m
            contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area >= min_area_px and (best is None or area > best[1]):
                    best = (color, area, cnt)
        if best is None:
            return None, total
        color, area, cnt = best
        pts = cnt.reshape(-1, 2)
        v = int(pts[:, 1].max())
        # mean column of the lowest part of the blob: that is where the robot aims
        u = int(pts[pts[:, 1] >= v - 3, 0].mean())
        bd = self.camera.bearing_distance(u, v)
        if bd is None:
            return None, total
        return Region(color, u, v, int(area), cv2.boundingRect(cnt), *bd), total

    def _locate(self, det: Detection) -> bool:
        """Computes bearing and distance; returns False if the blob does not look like a cube."""
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
    """Copy of the image with a box and label per cube. `highlight` is drawn thicker."""
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
