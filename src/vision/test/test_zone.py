"""
Tests: delivery area location (landmarks far away, area paper up close)
and dropping cubes already inside it (needs OpenCV).
"""

import cv2
import numpy as np

from vision.camera_model import CameraModel
from vision.color_detection import Detection, Region
from vision.hsv import parse_hsv_ranges
from vision.zone import (
    ZoneLocator,
    ZoneParams,
    choose_zone,
    is_plausible_floor,
    without_cubes_in_zone,
)

CAM = CameraModel(
    width_px=640,
    height_px=480,
    hfov_rad=0.94,
    height_m=0.10,
    tilt_rad=0.20,
)
RANGES = {
    "red": parse_hsv_ranges(
        [0, 110, 80, 8, 255, 255, 165, 110, 80, 179, 255, 255]
    ),
    "green": parse_hsv_ranges([45, 80, 50, 92, 255, 255]),
    "blue": parse_hsv_ranges([103, 120, 70, 128, 255, 255]),
}
GREEN = (60, 200, 180)
RED = (2, 200, 180)
BLUE = (115, 200, 180)


def blank():
    return np.full((480, 640, 3), 235, dtype=np.uint8)


def paint_hsv(img, x, y, w, h, hsv):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    img[y : y + h, x : x + w] = bgr


def region(color, u, v, bbox):
    return Region(color, u, v, 1000, bbox, 0.0, 0.5)


def test_choose_zone_prefers_the_floor():
    gate = region("gate", 300, 200, (250, 150, 100, 50))
    floor = region("floor", 320, 400, (200, 380, 240, 60))
    assert choose_zone(gate, floor) is floor
    assert choose_zone(gate, None) is gate
    assert choose_zone(None, None) is None


def test_far_away_only_the_landmarks_locate_the_zone():
    img = blank()
    paint_hsv(img, 140, 150, 20, 50, GREEN)
    paint_hsv(img, 460, 150, 20, 50, RED)
    zone = ZoneLocator(RANGES, CAM, ZoneParams()).locate(img)
    assert zone is not None and zone.color == "gate"
    assert 150 < zone.u < 470


def test_up_close_the_area_paper_wins():
    img = blank()
    paint_hsv(img, 140, 150, 20, 50, GREEN)
    paint_hsv(img, 460, 150, 20, 50, RED)
    paint_hsv(img, 180, 400, 140, 70, BLUE)
    paint_hsv(img, 320, 400, 140, 70, GREEN)
    zone = ZoneLocator(RANGES, CAM, ZoneParams()).locate(img)
    assert zone is not None and zone.color == "floor"


def test_nothing_in_view_gives_no_zone():
    assert ZoneLocator(RANGES, CAM, ZoneParams()).locate(blank()) is None


def test_cubes_inside_the_floor_area_are_dropped():
    floor = region("floor", 320, 470, (180, 400, 280, 70))
    inside = Detection("yellow", 300, 440, 20, 20, (290, 420, 20, 20))
    outside = Detection("yellow", 100, 300, 20, 20, (90, 280, 20, 20))
    kept = without_cubes_in_zone([inside, outside], floor, margin_px=80)
    assert kept == [outside]
    assert without_cubes_in_zone([inside], None, margin_px=80) == [inside]


def floor_blob(area_px, v):
    return Region("floor", 320, v, area_px, (0, 0, 640, 40), 0.0, 0.5)


def test_an_area_blob_larger_than_the_limit_is_not_the_paper():
    params = ZoneParams(floor_max_area_px=50000)
    assert is_plausible_floor(floor_blob(14000, 250), 480, params)
    assert not is_plausible_floor(floor_blob(90000, 250), 480, params)


def test_an_area_blob_reaching_the_bottom_is_not_the_paper_when_asked():
    asked = ZoneParams(floor_ignore_touching_bottom=True)
    assert is_plausible_floor(floor_blob(14000, 445), 480, asked)
    assert not is_plausible_floor(floor_blob(14000, 479), 480, asked)
    assert is_plausible_floor(floor_blob(14000, 479), 480, ZoneParams())


def test_there_is_no_paper_without_a_blob():
    assert not is_plausible_floor(None, 480, ZoneParams())
