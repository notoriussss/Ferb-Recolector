"""Nodo cube_detector: busca cubos por color (HSV) en image_raw y publica cubes.

También publica cubes/debug_image: la misma imagen con una caja por cubo, para verla en
web_viewer o en rqt_image_view.

Los rangos HSV y la geometría de la cámara vienen de config/hsv_ranges.yaml. Se pueden
cambiar en caliente con `ros2 param set /cube_detector ...`.

TODO: ubicar también el cuadrado de entrega.
"""

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from recolector_interfaces.msg import CubeDetection, CubeDetectionArray
from vision.color_detection import (
    CameraModel,
    ColorDetector,
    DetectionFilter,
    draw_detections,
    parse_hsv_ranges,
)
from vision.ros_image import bgr_to_image, image_to_bgr

DEFAULT_COLORS = ["rojo", "naranja", "amarillo", "verde", "azul", "morado"]


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__("cube_detector")
        self.declare_parameter("color_names", DEFAULT_COLORS)
        for name in self.get_parameter("color_names").value:
            self.declare_parameter(f"hsv.{name}", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("camera.hfov_rad", 0.94)
        self.declare_parameter("camera.height_m", 0.11)
        self.declare_parameter("camera.tilt_rad", 0.42)
        self.declare_parameter("filter.min_area_px", 40)
        self.declare_parameter("filter.cube_size_m", 0.025)
        self.declare_parameter("filter.min_size_ratio", 0.45)
        self.declare_parameter("filter.max_size_ratio", 2.2)
        self.declare_parameter("filter.ignore_touching_bottom", True)
        self.declare_parameter("filter.max_distance_m", 2.5)
        self.declare_parameter("publish_debug_image", True)

        self._detector = None
        self._image_size = None
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_changed)

        self._pub_cubes = self.create_publisher(CubeDetectionArray, "cubes", 10)
        self._pub_debug = self.create_publisher(Image, "cubes/debug_image", qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"buscando cubos de colores: {', '.join(self.get_parameter('color_names').value)}"
        )

    # ---------- parámetros ----------
    def _validate_params(self, params):
        for p in params:
            if p.name.startswith("hsv."):
                try:
                    parse_hsv_ranges(p.value)
                except (ValueError, TypeError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _on_params_changed(self, _params):
        # se reconstruye el detector con el próximo cuadro
        self._detector = None

    def _build_detector(self, width: int, height: int) -> ColorDetector:
        ranges = {}
        for name in self.get_parameter("color_names").value:
            values = self.get_parameter(f"hsv.{name}").value
            if not values:
                self.get_logger().warn(f"el color '{name}' no tiene rango hsv.{name}; se ignora")
                continue
            ranges[name] = parse_hsv_ranges(values)
        camera = CameraModel(
            width_px=width,
            height_px=height,
            hfov_rad=self.get_parameter("camera.hfov_rad").value,
            height_m=self.get_parameter("camera.height_m").value,
            tilt_rad=self.get_parameter("camera.tilt_rad").value,
        )
        filters = DetectionFilter(
            min_area_px=self.get_parameter("filter.min_area_px").value,
            cube_size_m=self.get_parameter("filter.cube_size_m").value,
            min_size_ratio=self.get_parameter("filter.min_size_ratio").value,
            max_size_ratio=self.get_parameter("filter.max_size_ratio").value,
            ignore_touching_bottom=self.get_parameter("filter.ignore_touching_bottom").value,
            max_distance_m=self.get_parameter("filter.max_distance_m").value,
        )
        return ColorDetector(ranges, camera, filters)

    # ---------- imagen ----------
    def _on_image(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        size = (msg.width, msg.height)
        if self._detector is None or self._image_size != size:
            self._detector = self._build_detector(*size)
            self._image_size = size

        detections = self._detector.detect(bgr)

        out = CubeDetectionArray()
        out.header = msg.header
        for d in detections:
            c = CubeDetection()
            c.color = d.color
            c.bearing_rad = float(d.bearing_rad)
            c.distance_m = float(d.distance_m)
            c.u, c.v = int(d.u), int(d.v)
            c.width_px, c.height_px = int(d.width_px), int(d.height_px)
            out.cubes.append(c)
        self._pub_cubes.publish(out)

        if self.get_parameter("publish_debug_image").value and (
            self._pub_debug.get_subscription_count() > 0
        ):
            self._pub_debug.publish(bgr_to_image(draw_detections(bgr, detections), msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
