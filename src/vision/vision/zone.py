"""
Delivery area location. Pure logic, no rclpy.

The area is blue and green paper in a gap of the black line, flanked by two
upright traffic-light landmarks (green on the left, red on the right, seen
from the arena). Two estimates, best one first:

- floor: the area paper itself, once it is in view at the bottom of the
  image (precise up close);
- gate: the point between the landmarks, which are tall and seen from
  afar.

Cubes already inside the area are dropped so they are not picked up again.
Used by the cube_detector node and by tools/teleop_web_lib.
"""

from dataclasses import dataclass, field

import numpy as np

from vision.camera_model import CameraModel
from vision.color_detection import (
    REGION_FLOOR,
    ColorDetector,
    Detection,
    Region,
)
from vision.hsv import HsvRange

# --- Defaults (real values: config/hsv_ranges.yaml, `zone.*`) -----------------
# [left, right] landmark colours, seen from the arena.
DEFAULT_LANDMARK_COLORS = ("green", "red")
DEFAULT_FLOOR_COLORS = ("blue", "green")
DEFAULT_LANDMARK_MIN_AREA_PX = 200
# Landmarks are upright: height at least this times the width.
DEFAULT_LANDMARK_MIN_HEIGHT_RATIO = 1.2
# Smallest area paper blob (TODO measure on the track).
DEFAULT_FLOOR_MIN_AREA_PX = 1500
# Paper on the floor looks at least this many times wider than tall.
DEFAULT_FLOOR_MIN_FLATNESS = 1.5
# Cubes up to this many px nearer than the landmarks' bases count as in the
# area when only the gate is known.
DEFAULT_IGNORE_MARGIN_PX = 80
# Largest area paper blob accepted (0 = no limit): under some white balances
# the whole pale table reads as paper and fills the lower image.
DEFAULT_FLOOR_MAX_AREA_PX = 0
# Drop an area blob that reaches the bottom of the image: the real paper has
# the floor in front of it, a blob of the table itself does not.
DEFAULT_FLOOR_IGNORE_TOUCHING_BOTTOM = False
# A blob whose lowest row is within this many px of the image bottom
# touches it.
FLOOR_BOTTOM_MARGIN_PX = 3

# Half width of the area around the gate aim column, in landmark heights.
PAPER_HALF_SPAN_HEIGHTS = 1.8
LANDMARK_COUNT = 2


@dataclass
class ZoneParams:
    """
    Parameters of the delivery area locator.

    Field names double as the `zone.*` ROS parameter names.
    """

    landmark_colors: list[str] = field(
        default_factory=lambda: list(DEFAULT_LANDMARK_COLORS),
    )
    floor_colors: list[str] = field(
        default_factory=lambda: list(DEFAULT_FLOOR_COLORS),
    )
    landmark_min_area_px: int = DEFAULT_LANDMARK_MIN_AREA_PX
    landmark_min_height_ratio: float = DEFAULT_LANDMARK_MIN_HEIGHT_RATIO
    floor_min_area_px: int = DEFAULT_FLOOR_MIN_AREA_PX
    floor_min_flatness: float = DEFAULT_FLOOR_MIN_FLATNESS
    ignore_margin_px: int = DEFAULT_IGNORE_MARGIN_PX
    floor_max_area_px: int = DEFAULT_FLOOR_MAX_AREA_PX
    floor_ignore_touching_bottom: bool = DEFAULT_FLOOR_IGNORE_TOUCHING_BOTTOM


def is_plausible_floor(
    floor: Region | None,
    frame_height_px: int,
    params: ZoneParams,
) -> bool:
    """
    True when `floor` can be the area paper: it exists, is not larger than
    `floor_max_area_px` and, if asked, does not reach the image bottom.
    """
    if floor is None:
        return False
    if params.floor_max_area_px and floor.area_px > params.floor_max_area_px:
        return False
    touches_bottom = floor.v >= frame_height_px - FLOOR_BOTTOM_MARGIN_PX
    return not (params.floor_ignore_touching_bottom and touches_bottom)


def choose_zone(gate: Region | None, floor: Region | None) -> Region | None:
    """
    The floor estimate when the area paper is in view, else the gate.
    """
    return floor if floor is not None else gate


def in_zone(det: Detection, zone: Region, margin_px: int) -> bool:
    """
    True when cube `det` already lies inside the delivery area.
    """
    x, y, w, h = zone.bbox
    if zone.color == REGION_FLOOR:
        return x <= det.u <= x + w and y <= det.v <= y + h
    half_span_px = PAPER_HALF_SPAN_HEIGHTS * h
    limit_v = y + h + margin_px
    return abs(det.u - zone.u) <= half_span_px and det.v <= limit_v


def without_cubes_in_zone(
    detections: list[Detection],
    zone: Region | None,
    margin_px: int,
) -> list[Detection]:
    """
    The detections that are not already inside the delivery area.
    """
    if zone is None:
        return list(detections)
    return [d for d in detections if not in_zone(d, zone, margin_px)]


class ZoneLocator:
    """
    Finds the delivery area in a frame from the landmarks and the paper.
    """

    def __init__(
        self,
        ranges: dict[str, list[HsvRange]],
        camera: CameraModel,
        params: ZoneParams,
    ):
        """
        Build one detector for the landmarks and one for the area paper
        from the HSV `ranges` (colours without a range are skipped).
        """
        self.params = params
        self._landmarks = None
        if len(params.landmark_colors) == LANDMARK_COUNT and all(
            c in ranges for c in params.landmark_colors
        ):
            self._landmarks = ColorDetector(
                {c: ranges[c] for c in params.landmark_colors},
                camera,
            )
        floor_ranges = {
            c: ranges[c] for c in params.floor_colors if c in ranges
        }
        self._floor = (
            ColorDetector(floor_ranges, camera) if floor_ranges else None
        )

    def locate(self, bgr: np.ndarray) -> Region | None:
        """
        The best estimate of the delivery area in `bgr`, or None.
        """
        return choose_zone(self._find_gate(bgr), self._find_floor(bgr))

    def _find_gate(self, bgr: np.ndarray) -> Region | None:
        """
        The area located from the left and/or right landmark.
        """
        if self._landmarks is None:
            return None
        p = self.params
        left_color, right_color = p.landmark_colors
        left, right = (
            self._landmarks.find_marker(
                bgr,
                color,
                p.landmark_min_area_px,
                p.landmark_min_height_ratio,
            )
            for color in (left_color, right_color)
        )
        return self._landmarks.gate_between(left, right)

    def _find_floor(self, bgr: np.ndarray) -> Region | None:
        """
        The area paper on the floor.
        """
        if self._floor is None:
            return None
        floor = self._floor.find_floor_area(
            bgr,
            self.params.floor_min_area_px,
            self.params.floor_min_flatness,
        )
        if is_plausible_floor(floor, bgr.shape[0], self.params):
            return floor
        return None
