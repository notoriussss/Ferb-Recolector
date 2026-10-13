"""Nodo camera_node: abre la cámara y publica image_raw.

Usa OpenCV (V4L2): sirve para una webcam USB o para la cámara de la Pi si el sistema la
expone como /dev/videoN. En la simulación no se usa: webots_bridge publica image_raw.

TODO (rol 2): probar con la OV5647 en la Pi 5 (puede necesitar libcamera / un pipeline
de GStreamer en `device`).
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vision.ros_image import bgr_to_image


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        # número de /dev/videoN o un pipeline de GStreamer
        self.declare_parameter("device", "0")
        self.declare_parameter("width_px", 640)
        self.declare_parameter("height_px", 480)
        self.declare_parameter("fps_hz", 15.0)
        self.declare_parameter("frame_id", "camara")

        device = self.get_parameter("device").value
        self._frame_id = self.get_parameter("frame_id").value
        if device.isdigit():
            self._cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
        else:
            self._cap = cv2.VideoCapture(device, cv2.CAP_GSTREAMER)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter("width_px").value)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter("height_px").value)
        if not self._cap.isOpened():
            self.get_logger().error(f"no se pudo abrir la cámara '{device}'")

        self._pub = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter("fps_hz").value, self._tick)

    def _tick(self):
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn("no llegó imagen de la cámara", throttle_duration_sec=5.0)
            return
        msg = bgr_to_image(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

    def destroy_node(self):
        self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
