"""Tests: HSV colour classification and camera geometry."""

import math

import cv2
import numpy as np
import pytest

from vision.color_detection import (
    CameraModel,
    ColorDetector,
    DetectionFilter,
    HsvRange,
    parse_hsv_ranges,
)

CAM = CameraModel(width_px=640, height_px=480, hfov_rad=0.94, height_m=0.11, tilt_rad=0.42)

RANGES = {
    "red": parse_hsv_ranges([0, 110, 100, 6, 255, 255, 173, 110, 100, 179, 255, 255]),
    "orange": parse_hsv_ranges([8, 120, 100, 18, 255, 255]),
    "blue": parse_hsv_ranges([100, 110, 90, 130, 255, 255]),
}


def blank():
    return np.full((480, 640, 3), 235, dtype=np.uint8)  # white background like the field


def paint_hsv(img, x, y, w, h, hsv):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    img[y : y + h, x : x + w] = bgr


def test_parse_ranges_groups_of_six():
    r = parse_hsv_ranges([0, 1, 2, 10, 11, 12, 170, 1, 2, 179, 255, 255])
    assert r == [HsvRange((0, 1, 2), (10, 11, 12)), HsvRange((170, 1, 2), (179, 255, 255))]


@pytest.mark.parametrize("bad", [[], [1, 2, 3], [10, 0, 0, 5, 255, 255], [0, 0, 0, 200, 255, 255]])
def test_parse_ranges_rejects_bad_input(bad):
    with pytest.raises(ValueError):
        parse_hsv_ranges(bad)


def test_center_bottom_pixel_is_straight_ahead():
    bearing, dist = CAM.bearing_distance(320, 240)
    assert bearing == pytest.approx(0.0)
    assert dist == pytest.approx(0.11 / math.tan(0.42))


def test_lower_rows_are_closer_and_left_is_positive():
    _, near = CAM.bearing_distance(320, 400)
    _, far = CAM.bearing_distance(320, 260)
    assert near < far
    left, _ = CAM.bearing_distance(100, 300)
    right, _ = CAM.bearing_distance(540, 300)
    assert left > 0 > right


def test_above_horizon_has_no_ground_point():
    # the top row is just below the horizon: very far away
    assert CAM.bearing_distance(320, 0)[1] > 1.5
    # camera looking up: the image centre does not hit the floor
    cam_up = CameraModel(640, 480, 0.94, 0.11, tilt_rad=-0.5)
    assert cam_up.ground_point(320, 100) is None


def cube_box(u, v_bottom):
    """Box the expected size of a 2.5 cm cube with its bottom edge at (u, v)."""
    _, _, depth = CAM.ground_point(u, v_bottom)
    side = round(CAM.focal_px * 0.025 / depth)
    return u - side // 2, v_bottom - side + 1, side, side


def test_detects_each_color_with_its_position():
    img = blank()
    x, y, w, h = cube_box(200, 300)
    paint_hsv(img, x, y, w, h, (2, 200, 180))  # red near 0
    x2, y2, w2, h2 = cube_box(450, 260)
    paint_hsv(img, x2, y2, w2, h2, (114, 170, 160))  # blue
    det = ColorDetector(RANGES, CAM).detect(img)
    assert [d.color for d in det] == ["red", "blue"]  # sorted by distance
    assert det[0].bearing_rad > 0 > det[1].bearing_rad
    assert det[0].distance_m == pytest.approx(CAM.bearing_distance(det[0].u, det[0].v)[1])


def test_red_wraps_around_hue_179():
    img = blank()
    x, y, w, h = cube_box(320, 300)
    paint_hsv(img, x, y, w, h, (177, 190, 170))
    det = ColorDetector(RANGES, CAM).detect(img)
    assert [d.color for d in det] == ["red"]


def test_orange_is_not_red():
    img = blank()
    x, y, w, h = cube_box(320, 300)
    paint_hsv(img, x, y, w, h, (12, 190, 197))
    det = ColorDetector(RANGES, CAM).detect(img)
    assert [d.color for d in det] == ["orange"]


def test_ignores_white_black_and_gray():
    img = blank()
    img[100:140, 100:400] = 20  # black line
    img[300:340, 300:340] = 128  # gris
    assert ColorDetector(RANGES, CAM).detect(img) == []


def test_ignores_blob_touching_bottom_edge():
    img = blank()
    paint_hsv(img, 100, 380, 60, 100, (12, 190, 197))  # the orange scoop shows at the bottom
    assert ColorDetector(RANGES, CAM).detect(img) == []
    keep = ColorDetector(RANGES, CAM, DetectionFilter(ignore_touching_bottom=False))
    assert len(keep.detect(img)) <= 1


def test_rejects_blob_too_big_for_a_cube():
    img = blank()
    x, y, w, h = cube_box(320, 260)
    # a blob much larger than a cube, e.g. another robot's chassis
    paint_hsv(img, x - 3 * w, y - h, 7 * w, 2 * h, (114, 170, 160))
    assert ColorDetector(RANGES, CAM).detect(img) == []


def test_find_region_returns_nearest_point_of_largest_line():
    img = blank()
    paint_hsv(img, 100, 300, 400, 20, (110, 200, 200))  # blue line across the image
    paint_hsv(img, 300, 100, 10, 10, (110, 200, 200))  # small blob: not the line
    region, mask = ColorDetector(RANGES, CAM).find_region(img, min_area_px=300)
    assert region.color == "blue"
    assert 315 <= region.v <= 320 and 280 <= region.u <= 320
    assert abs(region.bearing_rad) < 0.05
    assert mask[105, 305] and not mask[200, 50]


def test_find_region_none_without_color():
    region, mask = ColorDetector(RANGES, CAM).find_region(blank(), min_area_px=300)
    assert region is None and not mask.any()
