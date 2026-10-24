"""
cube_detector node: finds cubes by colour (HSV) in image_raw and publishes
cubes.

It also publishes cubes/debug_image: the same image with one box per cube,
to view it in web_viewer or rqt_image_view.

HSV ranges and camera geometry come from config/hsv_ranges.yaml. They can be
changed at runtime with `ros2 param set /cube_detector ...`.

TODO: also locate the delivery square.
"""

from dataclasses import dataclass, field

from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from recolector_common.node_utils import spin_node
from recolector_common.params import declare_params, read_params
from recolector_interfaces.msg import CubeDetection, CubeDetectionArray
from vision.camera_model import CameraModel, CameraParams
from vision.color_detection import ColorDetector, DetectionFilter
from vision.colors import COLOR_NAMES
from vision.drawing import draw_detections
from vision.hsv import parse_hsv_ranges
from vision.ros_image import bgr_to_image, image_to_bgr
from vision.topics import (
    DEFAULT_QOS_DEPTH,
    TOPIC_CUBES,
    TOPIC_CUBES_DEBUG_IMAGE,
    TOPIC_IMAGE_RAW,
)

NODE_NAME = "cube_detector"

# Prefixes of the grouped parameters (hsv.red, camera.hfov_rad, ...).
HSV_PREFIX = "hsv."
CAMERA_PREFIX = "camera."
FILTER_PREFIX = "filter."

# Minimum time between two "bad image" errors.
IMAGE_ERROR_THROTTLE_S = 5.0


@dataclass
class CubeDetectorParams:
    """
    Node-level parameters; the HSV ranges (`hsv.<colour>`), `camera.*` and
    `filter.*` groups are declared separately.

    Field names double as ROS parameter names (see recolector_common.params).
    """

    color_names: list[str] = field(default_factory=lambda: list(COLOR_NAMES))
    publish_debug_image: bool = True


class CubeDetectorNode(Node):
    """
    Runs the colour detector on every incoming image.
    """

    def __init__(self):
        """
        Declare the parameters and create the detector's topics.
        """
        super().__init__(NODE_NAME)
        declare_params(self, CubeDetectorParams())
        self._params = read_params(self, CubeDetectorParams())
        for name in self._params.color_names:
            self.declare_parameter(
                f"{HSV_PREFIX}{name}",
                Parameter.Type.INTEGER_ARRAY,
            )
        declare_params(self, CameraParams(), prefix=CAMERA_PREFIX)
        declare_params(self, DetectionFilter(), prefix=FILTER_PREFIX)

        self._detector = None
        self._image_size = None
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_changed)

        self._pub_cubes = self.create_publisher(
            CubeDetectionArray,
            TOPIC_CUBES,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_debug = self.create_publisher(
            Image,
            TOPIC_CUBES_DEBUG_IMAGE,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            TOPIC_IMAGE_RAW,
            self._on_image,
            qos_profile_sensor_data,
        )
        colors = ", ".join(self._params.color_names)
        self.get_logger().info(f"looking for cubes of colours: {colors}")

    # ---------- parameters ----------
    def _validate_params(self, params):
        """
        Reject malformed HSV ranges before they are applied.
        """
        for p in params:
            if p.name.startswith(HSV_PREFIX):
                try:
                    parse_hsv_ranges(p.value)
                except (ValueError, TypeError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _on_params_changed(self, _params):
        """
        Drop the detector; it is rebuilt on the next frame.
        """
        self._params = read_params(self, CubeDetectorParams())
        self._detector = None

    def _build_detector(self, width: int, height: int) -> ColorDetector:
        """
        Build a detector for images of the given size from the parameters.
        """
        ranges = {}
        for name in self._params.color_names:
            values = self.get_parameter(f"{HSV_PREFIX}{name}").value
            if not values:
                self.get_logger().warn(
                    f"colour '{name}' has no {HSV_PREFIX}{name} range; ignored"
                )
                continue
            ranges[name] = parse_hsv_ranges(values)
        camera_params = read_params(self, CameraParams(), prefix=CAMERA_PREFIX)
        camera = CameraModel(
            width_px=width,
            height_px=height,
            hfov_rad=camera_params.hfov_rad,
            height_m=camera_params.height_m,
            tilt_rad=camera_params.tilt_rad,
        )
        filters = read_params(self, DetectionFilter(), prefix=FILTER_PREFIX)
        return ColorDetector(ranges, camera, filters)

    # ---------- image ----------
    def _on_image(self, msg: Image):
        """
        Detect cubes in one image and publish them (and the debug image).
        """
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(
                str(e),
                throttle_duration_sec=IMAGE_ERROR_THROTTLE_S,
            )
            return
        size = (msg.width, msg.height)
        if self._detector is None or self._image_size != size:
            self._detector = self._build_detector(*size)
            self._image_size = size

        detections = self._detector.detect(bgr)
        self._pub_cubes.publish(_to_message(detections, msg.header))

        wants_debug = self._params.publish_debug_image
        if wants_debug and self._pub_debug.get_subscription_count() > 0:
            debug_image = draw_detections(bgr, detections)
            self._pub_debug.publish(bgr_to_image(debug_image, msg.header))


def _to_message(detections, header) -> CubeDetectionArray:
    """
    Convert detector output into the CubeDetectionArray message.
    """
    out = CubeDetectionArray()
    out.header = header
    for d in detections:
        cube = CubeDetection()
        cube.color = d.color
        cube.bearing_rad = float(d.bearing_rad)
        cube.distance_m = float(d.distance_m)
        cube.u, cube.v = int(d.u), int(d.v)
        cube.width_px, cube.height_px = int(d.width_px), int(d.height_px)
        out.cubes.append(cube)
    return out


def main(args=None):
    """
    Entry point of the cube_detector node.
    """
    spin_node(CubeDetectorNode, args)


if __name__ == "__main__":
    main()
