"""
Cube and delivery-area detection for the browser tool, on top of the vision
package's pure logic (the same code the ROS cube_detector node uses,
including vision/zone.py for the area).

Needs OpenCV, NumPy and PyYAML. `camera.py` imports this module lazily, so
the robot still drives when they are missing.
"""

import logging
import math
import sys
import threading
import time

import numpy as np

from teleop_web_lib.avoidance import is_guarded
from teleop_web_lib.border import BorderParams, without_cubes_near_border
from teleop_web_lib.config import (
    DEFAULT_DETECTION_FILTER,
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    VISION_PACKAGE_DIR,
    read_node_params,
)
from teleop_web_lib.support import SupportParams, without_cubes_off_table

log = logging.getLogger("teleop_web")

# Section of hsv_ranges.yaml with the delivery area parameters.
ZONE_SECTION = "zone"
# Key of the camera section that overrides the camera geometry.
MODEL_SECTION = "model"
# Key of the camera section with the border filter settings.
BORDER_SECTION = "border"
# Key of the camera section with the cube-on-the-table filter settings.
SUPPORT_SECTION = "support"
# Key of the camera section with extra HSV ranges for the area paper.
AREA_HSV_KEY = "area_hsv"
# Key of the camera section with the colours of the other robot's cubes.
AVOID_COLORS_KEY = "avoid_colors"
# Key of the camera section with how close (m) to a cube to avoid a target
# can be and still be caught.
AVOID_GUARD_KEY = "avoid_guard_m"
# A target with a cube to avoid this close (m) cannot be caught without
# pushing it too: half the 186 mm chassis, a cube and a margin.
# TODO measure
DEFAULT_AVOID_GUARD_M = 0.13
# Key of the camera section with how long (s) a cube to avoid still guards
# the targets near it after it was last seen.
AVOID_GUARD_MEMORY_KEY = "avoid_guard_memory_s"
# A red cube 5 cm from a yellow one was found in some frames only: the
# yellow one turned target and not, and the robot backed up and came back to
# it for minutes. TODO measure
DEFAULT_AVOID_GUARD_MEMORY_S = 1.0


class DetectionPipeline:
    """
    Finds the best cube and the delivery area in each frame, remembers the
    latest of each, and returns the annotated frame for the video stream.
    """

    def __init__(
        self,
        detector,
        zone_locator,
        border: BorderParams | None = None,
        support: SupportParams | None = None,
        avoid_colors: frozenset[str] = frozenset(),
        avoid_guard_m: float = DEFAULT_AVOID_GUARD_M,
        avoid_guard_memory_s: float = DEFAULT_AVOID_GUARD_MEMORY_S,
    ):
        """
        Keep the cube detector, the area locator (vision/zone.py) and the
        border filter settings (off when not given). Cubes of `avoid_colors`
        are never targets: they are only reported by `foreign_cubes`, and a
        target closer than `avoid_guard_m` to one seen in the last
        `avoid_guard_memory_s` is skipped.
        """
        self._avoid_colors = avoid_colors
        self._avoid_guard_m = avoid_guard_m
        self._avoid_guard_memory_s = avoid_guard_memory_s
        # (time seen, cube) of the cubes to avoid that still guard targets;
        # only the capture thread uses it.
        self._guards: list = []
        self._detector = detector
        self._zone_locator = zone_locator
        self._border = border or BorderParams()
        self._support = support or SupportParams()
        self._lock = threading.Lock()
        self._det = None
        self._det_stamp_s = 0.0
        self._foreign: list = []
        self._foreign_stamp_s = 0.0
        # When a target was last skipped for a cube to avoid next to it.
        self._guarded_stamp_s = -math.inf
        self._zone = None
        self._zone_stamp_s = 0.0

    def latest_detection(self, now: float, timeout_s: float):
        """
        The best cube seen less than `timeout_s` ago, or None.
        """
        with self._lock:
            if self._det is not None and (now - self._det_stamp_s) <= timeout_s:
                return self._det
            return None

    def foreign_cubes(self, now: float, timeout_s: float) -> list:
        """
        The cubes to avoid seen less than `timeout_s` ago, nearest first;
        empty when none.
        """
        with self._lock:
            fresh = (now - self._foreign_stamp_s) <= timeout_s
            return list(self._foreign) if fresh else []

    def guarded_target_seen(self, now: float, timeout_s: float) -> bool:
        """
        True when a cube to go for was skipped less than `timeout_s` ago
        because a cube to avoid lies next to it.
        """
        with self._lock:
            return (now - self._guarded_stamp_s) <= timeout_s

    def latest_zone(self, now: float, timeout_s: float):
        """
        The delivery area seen less than `timeout_s` ago, or None.
        """
        with self._lock:
            if (
                self._zone is not None
                and (now - self._zone_stamp_s) <= timeout_s
            ):
                return self._zone
            return None

    def process(self, frame: np.ndarray) -> np.ndarray:
        """
        Detect in `frame`, remember the result and return the annotated
        frame.
        """
        from vision.drawing import draw_detections, draw_region
        from vision.zone import without_cubes_in_zone

        zone = self._zone_locator.locate(frame)
        zone_params = self._zone_locator.params
        detections = without_cubes_in_zone(
            self._detector.detect(frame),
            zone,
            zone_params.ignore_margin_px,
            zone_params.ignore_floor_margin_m,
            zone_params.ignore_gate_depth_m,
            zone_params.ignore_gate_half_width_m,
        )
        detections = without_cubes_near_border(frame, detections, self._border)
        detections = without_cubes_off_table(frame, detections, self._support)
        now = time.monotonic()
        self._guards = [
            (seen_s, cube)
            for seen_s, cube in self._guards
            if now - seen_s <= self._avoid_guard_memory_s
        ]
        remembered = [cube for _, cube in self._guards]
        best, foreign = split_cubes(
            detections,
            self._avoid_colors,
            self._avoid_guard_m,
            remembered,
        )
        self._guards += [(now, cube) for cube in foreign]
        guarded = any(
            d.color not in self._avoid_colors
            and is_guarded(d, foreign + remembered, self._avoid_guard_m)
            for d in detections
        )
        self._remember(best, foreign, zone, guarded)

        annotated = draw_detections(frame, detections, best)
        if zone is not None:
            draw_region(annotated, zone)
        return annotated

    def _remember(self, best, foreign, zone, guarded: bool) -> None:
        """
        Store the latest detections with the time they were seen.
        """
        now = time.monotonic()
        with self._lock:
            if guarded:
                self._guarded_stamp_s = now
            self._det = best
            if best is not None:
                self._det_stamp_s = now
            # A frame that missed them does not forget them: they flicker,
            # and one frame without them let the catch push start.
            if foreign:
                self._foreign = foreign
                self._foreign_stamp_s = now
            self._zone = zone
            if zone is not None:
                self._zone_stamp_s = now


def split_cubes(
    detections: list,
    avoid_colors: frozenset[str],
    guard_m: float = DEFAULT_AVOID_GUARD_M,
    remembered: list | None = None,
):
    """
    From detections sorted nearest first: the nearest cube to go for that
    no cube to avoid guards (None if there is none), and the cubes to
    avoid, nearest first. `remembered` cubes to avoid, seen in earlier
    frames, guard too.
    """
    foreign = [d for d in detections if d.color in avoid_colors]
    guards = foreign + list(remembered or [])
    ours = (
        d
        for d in detections
        if d.color not in avoid_colors and not is_guarded(d, guards, guard_m)
    )
    return next(ours, None), foreign


def build_pipeline(
    camera_cfg: dict,
    detection_overrides: dict | None = None,
) -> DetectionPipeline | None:
    """
    Build the pipeline from the camera section and the shared HSV file.

    Returns None (with a warning) when no target colours are configured or
    when anything needed to detect is missing: control keeps working
    without detection.
    """
    colors = camera_cfg.get("target_colors") or []
    if not colors:
        return None
    try:
        return _build_pipeline(
            camera_cfg,
            colors,
            detection_overrides or {},
        )
    except Exception as exc:
        log.warning("no colour detector: %s", exc)
        return None


def _build_pipeline(
    camera_cfg: dict,
    colors: list[str],
    detection_overrides: dict,
) -> DetectionPipeline:
    """
    The actual construction; may raise when files or colours are missing.
    """
    sys.path.insert(0, str(VISION_PACKAGE_DIR))
    from vision.camera_model import CameraModel
    from vision.color_detection import ColorDetector, DetectionFilter
    from vision.hsv import parse_hsv_ranges
    from vision.zone import ZoneLocator, ZoneParams

    hsv_cfg = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)
    # Colours of this tool only (the delivery area paper), added to the
    # shared ones without touching the file the ROS nodes read.
    hsv_values = {**hsv_cfg["hsv"], **camera_cfg.get(AREA_HSV_KEY, {})}
    ranges = {c: parse_hsv_ranges(v) for c, v in hsv_values.items()}

    # The tool's camera section overrides the shared geometry (its sensor
    # mode is not the one the shared values were measured in).
    geometry = {**hsv_cfg["camera"], **camera_cfg.get(MODEL_SECTION, {})}
    camera_model = CameraModel(
        width_px=camera_cfg["width_px"],
        height_px=camera_cfg["height_px"],
        hfov_rad=geometry["hfov_rad"],
        height_m=geometry["height_m"],
        tilt_rad=geometry["tilt_rad"],
    )
    filters = DetectionFilter(
        cube_size_m=hsv_cfg["filter"]["cube_size_m"],
        **{**DEFAULT_DETECTION_FILTER, **detection_overrides},
    )
    avoid = frozenset(camera_cfg.get(AVOID_COLORS_KEY) or [])
    if avoid & set(colors):
        both = sorted(avoid & set(colors))
        raise ValueError(f"colours both target and avoided: {both}")
    missing = sorted(avoid - set(ranges))
    if missing:
        raise ValueError(f"no HSV ranges for avoid_colors: {missing}")
    detector = ColorDetector(
        {c: ranges[c] for c in [*colors, *avoid] if c in ranges},
        camera_model,
        filters,
    )
    # The camera section may override the shared zone settings (e.g. turn the
    # area paper off: `zone: {floor_colors: []}`).
    zone_params = ZoneParams(
        **{**hsv_cfg.get(ZONE_SECTION, {}), **camera_cfg.get(ZONE_SECTION, {})}
    )
    return DetectionPipeline(
        detector,
        ZoneLocator(ranges, camera_model, zone_params),
        BorderParams(**camera_cfg.get(BORDER_SECTION, {})),
        SupportParams(**camera_cfg.get(SUPPORT_SECTION, {})),
        avoid,
        camera_cfg.get(AVOID_GUARD_KEY, DEFAULT_AVOID_GUARD_M),
        camera_cfg.get(AVOID_GUARD_MEMORY_KEY, DEFAULT_AVOID_GUARD_MEMORY_S),
    )
