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

import cv2
import numpy as np

from teleop_web_lib.config import (
    DEFAULT_DETECTION_FILTER,
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    VISION_PACKAGE_DIR,
    read_node_params,
)

log = logging.getLogger("teleop_web")


# --- Delivery line defaults (real values: tools/teleop_web.yaml, camera) ------
DEFAULT_ZONE_MIN_AREA_PX = 300
DEFAULT_ZONE_MIN_ELONGATION = 3.0
DEFAULT_ZONE_IGNORE_MARGIN_PX = 25


class DetectionPipeline:
    """
    Finds the best cube and the delivery line in each frame, remembers the
    latest of each, and returns the annotated frame for the video stream.
    """

    def __init__(
        self,
        detector,
        zone_detector,
        zone_min_area_px: int,
        zone_min_elongation: float,
        zone_ignore_margin_px: int,
    ):
        """
        Keep the detectors and the delivery-line parameters.
        """
        self._detector = detector
        self._zone_detector = zone_detector  # None = no delivery line colours
        self._zone_min_area_px = zone_min_area_px
        self._zone_min_elongation = zone_min_elongation
        side_px = 2 * zone_ignore_margin_px + 1
        self._line_margin_kernel = np.ones((side_px, side_px), np.uint8)
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
        The delivery line (green/blue) seen less than `timeout_s` ago, or
        None.
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
        zone = None
        if self._zone_detector is not None:
            zone, line_mask = self._zone_detector.find_region(
                frame,
                self._zone_min_area_px,
                self._zone_min_elongation,
            )
            detections = self._without_cubes_on_line(detections, line_mask)
        best = detections[0] if detections else None
        self._remember(best, zone)

        annotated = draw_detections(frame, detections, best)
        if zone is not None:
            draw_region(annotated, zone)
        return annotated

    def _without_cubes_on_line(self, detections: list, line_mask: np.ndarray):
        """
        Drop cubes already at the delivery line, so they are not picked up
        again.
        """
        near_line = cv2.dilate(line_mask, self._line_margin_kernel)
        height, width = near_line.shape
        return [
            d
            for d in detections
            if not near_line[min(d.v, height - 1), min(d.u, width - 1)]
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
    zone_colors = camera_cfg.get("zone_colors") or []
    zone_detector = None
    if zone_colors:
        zone_detector = ColorDetector(
            {c: parse_hsv_ranges(hsv[c]) for c in zone_colors},
            camera_model,
        )
    return DetectionPipeline(
        detector,
        zone_detector,
        zone_min_area_px=int(
            camera_cfg.get("zone_min_area_px", DEFAULT_ZONE_MIN_AREA_PX)
        ),
        zone_min_elongation=float(
            camera_cfg.get("zone_min_elongation", DEFAULT_ZONE_MIN_ELONGATION)
        ),
        zone_ignore_margin_px=int(
            camera_cfg.get(
                "zone_ignore_margin_px", DEFAULT_ZONE_IGNORE_MARGIN_PX
            )
        ),
    )
