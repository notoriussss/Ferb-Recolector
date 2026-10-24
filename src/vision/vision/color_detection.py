"""
Cube detection by colour (HSV) and camera geometry. Pure logic, no rclpy.

Flow: BGR image -> HSV mask per colour -> contours -> filters -> list of
`Detection` with the bearing and approximate distance on the floor (camera
tilted down).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from vision.camera_model import CameraModel
from vision.hsv import HsvRange

# --- Detection filter defaults (real values: config/hsv_ranges.yaml) ----------
DEFAULT_MIN_AREA_PX = 40
DEFAULT_CUBE_SIZE_M = 0.025
# Seen width / expected width at that distance: outside the range it is not
# a cube.
DEFAULT_MIN_SIZE_RATIO = 0.45
DEFAULT_MAX_SIZE_RATIO = 2.2
DEFAULT_MAX_DISTANCE_M = 2.5

# --- Detector constants -------------------------------------------------------
DEFAULT_MORPH_KERNEL_PX = 3
# A line blob must be at least this many times longer than wide.
DEFAULT_MIN_ELONGATION = 3.0
# Narrowest blob width considered when measuring elongation (avoids dividing
# by a near-zero width).
MIN_BLOB_WIDTH_PX = 1.0
# A blob whose lowest row is within this many px of the image bottom touches it.
BOTTOM_EDGE_MARGIN_PX = 1
# Rows (counted up from the lowest one) averaged to find the aim column of
# a line blob.
REGION_NEAR_ROWS_PX = 3

# Binary mask values.
MASK_ON = 255
MASK_OFF = 0


@dataclass(frozen=True)
class DetectionFilter:
    """
    Filters that discard blobs that are not cubes.

    Field names double as the `filter.*` ROS parameter names.
    """

    min_area_px: int = DEFAULT_MIN_AREA_PX
    cube_size_m: float = DEFAULT_CUBE_SIZE_M
    min_size_ratio: float = DEFAULT_MIN_SIZE_RATIO
    max_size_ratio: float = DEFAULT_MAX_SIZE_RATIO
    # Discard blobs touching the bottom edge (the scoop shows there).
    ignore_touching_bottom: bool = True
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M


@dataclass
class Detection:
    """
    A cube seen in the image.
    """

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
    """
    A large colour blob (the delivery line), not a cube.
    """

    color: str
    u: int  # column of the nearest point (px)
    v: int  # lowest row of the blob (px)
    area_px: int
    bbox: tuple[int, int, int, int]
    bearing_rad: float
    distance_m: float


@dataclass
class ColorDetector:
    """
    Multi-colour cube detector with its camera model.
    """

    ranges: dict[str, list[HsvRange]]
    camera: CameraModel
    filters: DetectionFilter = field(default_factory=DetectionFilter)
    morph_kernel_px: int = DEFAULT_MORPH_KERNEL_PX

    def mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """
        Binary mask (0/255) of one colour in an HSV image.
        """
        total = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in self.ranges[color]:
            total |= cv2.inRange(hsv, np.array(r.lower), np.array(r.upper))
        if self.morph_kernel_px > 1:
            kernel = np.ones((self.morph_kernel_px,) * 2, np.uint8)
            total = cv2.morphologyEx(total, cv2.MORPH_OPEN, kernel)
        return total

    def detect(self, bgr: np.ndarray) -> list[Detection]:
        """
        Find cubes of every colour, sorted from nearest to farthest.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        image_height_px = bgr.shape[0]
        found: list[Detection] = []
        for color in self.ranges:
            for contour in self._contours_for(hsv, color):
                detection = self._detection_from_contour(
                    color,
                    contour,
                    image_height_px,
                )
                if detection is not None:
                    found.append(detection)
        found.sort(key=lambda d: d.distance_m)
        return found

    def find_region(
        self,
        bgr: np.ndarray,
        min_area_px: int,
        min_elongation: float = DEFAULT_MIN_ELONGATION,
    ) -> tuple[Region | None, np.ndarray]:
        """
        The largest elongated blob of any of the colours (e.g. the
        delivery line).

        Only blobs at least `min_elongation` times longer than wide count,
        so a cube of the same colour is not mistaken for the line. Returns
        the region (with bearing and distance of its nearest point) and the
        mask of those line blobs, to tell which cubes are on it.
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        line_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        best = None  # (color, area, contour) of the largest line blob
        for color in self.ranges:
            for contour in self._contours_for(hsv, color):
                area = cv2.contourArea(contour)
                if area < min_area_px:
                    continue
                if not _is_elongated(contour, min_elongation):
                    continue  # squarish: a cube, not the line
                cv2.drawContours(line_mask, [contour], -1, MASK_ON, cv2.FILLED)
                if best is None or area > best[1]:
                    best = (color, area, contour)
        if best is None:
            return None, line_mask
        return self._region_from_contour(*best), line_mask

    def _contours_for(self, hsv: np.ndarray, color: str) -> list:
        """
        External contours of the blobs of one colour.
        """
        contours, _ = cv2.findContours(
            self.mask(hsv, color),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        return contours

    def _detection_from_contour(
        self,
        color: str,
        contour: np.ndarray,
        image_height_px: int,
    ) -> Detection | None:
        """
        A located `Detection` if the blob passes every filter, else None.
        """
        if cv2.contourArea(contour) < self.filters.min_area_px:
            return None
        x, y, w, h = cv2.boundingRect(contour)
        touches_bottom = y + h >= image_height_px - BOTTOM_EDGE_MARGIN_PX
        if self.filters.ignore_touching_bottom and touches_bottom:
            return None
        detection = Detection(color, x + w // 2, y + h - 1, w, h, (x, y, w, h))
        return detection if self._locate(detection) else None

    def _region_from_contour(
        self,
        color: str,
        area: float,
        contour: np.ndarray,
    ) -> Region | None:
        """
        Region aimed at the lowest part of a line blob, or None if that point
        is above the horizon.
        """
        points = contour.reshape(-1, 2)
        v = int(points[:, 1].max())
        # Mean column of the lowest part of the blob: that is where the robot
        # aims.
        near_bottom = points[:, 1] >= v - REGION_NEAR_ROWS_PX
        u = int(points[near_bottom, 0].mean())
        bearing_distance = self.camera.bearing_distance(u, v)
        if bearing_distance is None:
            return None
        return Region(
            color,
            u,
            v,
            int(area),
            cv2.boundingRect(contour),
            *bearing_distance,
        )

    def _locate(self, det: Detection) -> bool:
        """
        Compute bearing and distance; False if the blob does not look like a
        cube.
        """
        cam = self.camera
        point = cam.ground_point(det.u, det.v)
        if point is None:
            return False
        x, y, depth = point
        det.bearing_rad = math.atan2(y, x)
        det.distance_m = math.hypot(x, y)
        if det.distance_m > self.filters.max_distance_m:
            return False
        expected_px = cam.focal_px * self.filters.cube_size_m / depth
        ratio = det.width_px / expected_px
        return (
            self.filters.min_size_ratio <= ratio <= self.filters.max_size_ratio
        )


def _is_elongated(contour: np.ndarray, min_elongation: float) -> bool:
    """
    True when the contour's minimum-area rectangle is at least
    `min_elongation` times longer than wide.
    """
    width, height = cv2.minAreaRect(contour)[1]
    short_side = max(MIN_BLOB_WIDTH_PX, min(width, height))
    return max(width, height) >= min_elongation * short_side
