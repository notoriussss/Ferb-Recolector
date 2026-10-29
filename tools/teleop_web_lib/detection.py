"""
Cube and delivery-line detection for the browser tool, on top of the vision
package's pure logic (the same code the ROS cube_detector node uses).

Needs OpenCV, NumPy and PyYAML. `camera.py` imports this module lazily, so
the robot still drives when they are missing.
"""

import logging
import sys
import threading
import time

import numpy as np

from teleop_web_lib.config import (
    DEFAULT_DETECTION_FILTER,
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    VISION_PACKAGE_DIR,
    read_node_params,
)

log = logging.getLogger("teleop_web")


# --- Delivery gate defaults (real values: tools/teleop_web.yaml, camera) -----
GATE_LANDMARK_COUNT = 2
DEFAULT_GATE_MIN_AREA_PX = 200
# Landmarks are upright pieces: height must be at least this times the width.
DEFAULT_GATE_MIN_HEIGHT_RATIO = 1.2
# Cubes up to this many px below the landmarks' bases count as on the paper.
DEFAULT_GATE_IGNORE_MARGIN_PX = 80
PAPER_HALF_SPAN_HEIGHTS = 1.8


class DetectionPipeline:
    """
    Finds the best cube and the delivery gate in each frame, remembers the
    latest of each, and returns the annotated frame for the video stream.

    The gate is the gap between two landmarks: the left one (e.g. green)
    and the right one (e.g. red) flanking the delivery paper.
    """

    def __init__(
        self,
        detector,
        gate_detector,
        gate_colors: tuple[str, str],
        gate_min_area_px: int,
        gate_min_height_ratio: float,
        gate_ignore_margin_px: int,
    ):
        """
        Keep the detectors and the delivery-gate parameters.
        """
        self._detector = detector
        self._gate_detector = gate_detector  # None = no gate colours
        self._gate_colors = gate_colors
        self._gate_min_area_px = gate_min_area_px
        self._gate_min_height_ratio = gate_min_height_ratio
        self._gate_ignore_margin_px = gate_ignore_margin_px
        self._lock = threading.Lock()
        self._det = None
        self._det_stamp_s = 0.0
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

    def latest_zone(self, now: float, timeout_s: float):
        """
        The delivery gate seen less than `timeout_s` ago, or None.
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

        detections = self._detector.detect(frame)
        gate = None
        if self._gate_detector is not None:
            gate = self._find_gate(frame)
            if gate is not None:
                detections = self._without_cubes_on_paper(detections, gate)
        best = detections[0] if detections else None
        self._remember(best, gate)

        annotated = draw_detections(frame, detections, best)
        if gate is not None:
            draw_region(annotated, gate)
        return annotated

    def _find_gate(self, frame: np.ndarray):
        """
        The delivery area located from the left and/or right landmarks.
        """
        left_color, right_color = self._gate_colors
        left = self._gate_detector.find_marker(
            frame,
            left_color,
            self._gate_min_area_px,
            self._gate_min_height_ratio,
        )
        right = self._gate_detector.find_marker(
            frame,
            right_color,
            self._gate_min_area_px,
            self._gate_min_height_ratio,
        )
        return self._gate_detector.gate_between(left, right)

    def _without_cubes_on_paper(self, detections: list, gate) -> list:
        """
        Drop cubes already on the graph paper (around the gate aim column and
        not nearer than the front edge of the paper).
        """
        _, y, _, h = gate.bbox
        half_span_px = PAPER_HALF_SPAN_HEIGHTS * h
        limit_v = y + h + self._gate_ignore_margin_px
        return [
            d
            for d in detections
            if not (abs(d.u - gate.u) <= half_span_px and d.v <= limit_v)
        ]

    def _remember(self, best, zone) -> None:
        """
        Store the latest detections with the time they were seen.
        """
        now = time.monotonic()
        with self._lock:
            self._det = best
            if best is not None:
                self._det_stamp_s = now
            self._zone = zone
            if zone is not None:
                self._zone_stamp_s = now


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
        return _build_pipeline(camera_cfg, colors, detection_overrides or {})
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

    hsv_cfg = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)
    hsv = hsv_cfg["hsv"]

    camera_model = CameraModel(
        width_px=camera_cfg["width_px"],
        height_px=camera_cfg["height_px"],
        hfov_rad=hsv_cfg["camera"]["hfov_rad"],
        height_m=hsv_cfg["camera"]["height_m"],
        tilt_rad=hsv_cfg["camera"]["tilt_rad"],
    )
    filters = DetectionFilter(
        cube_size_m=hsv_cfg["filter"]["cube_size_m"],
        **{**DEFAULT_DETECTION_FILTER, **detection_overrides},
    )
    detector = ColorDetector(
        {c: parse_hsv_ranges(hsv[c]) for c in colors if c in hsv},
        camera_model,
        filters,
    )
    gate_colors = camera_cfg.get("gate_colors") or []
    gate_detector = None
    if len(gate_colors) == GATE_LANDMARK_COUNT:
        gate_detector = ColorDetector(
            {c: parse_hsv_ranges(hsv[c]) for c in gate_colors},
            camera_model,
        )
    return DetectionPipeline(
        detector,
        gate_detector,
        tuple(gate_colors) if gate_detector else ("", ""),
        gate_min_area_px=int(
            camera_cfg.get("gate_min_area_px", DEFAULT_GATE_MIN_AREA_PX)
        ),
        gate_min_height_ratio=float(
            camera_cfg.get(
                "gate_min_height_ratio", DEFAULT_GATE_MIN_HEIGHT_RATIO
            )
        ),
        gate_ignore_margin_px=int(
            camera_cfg.get(
                "gate_ignore_margin_px", DEFAULT_GATE_IGNORE_MARGIN_PX
            )
        ),
    )
