"""camera_node node: opens the camera and publishes image_raw.

Uses OpenCV (V4L2): works with a USB webcam or with the Pi camera if the system exposes it
as /dev/videoN. Not used in simulation: webots_bridge publishes image_raw.

TODO (role 2): test with the OV5647 on the Pi 5 (may need libcamera / a GStreamer
pipeline in `device`).
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
        # /dev/videoN number or a GStreamer pipeline
        self.declare_parameter("device", "0")
        self.declare_parameter("width_px", 640)
        self.declare_parameter("height_px", 480)
        self.declare_parameter("fps_hz", 15.0)
        self.declare_parameter("frame_id", "camera")

        device = self.get_parameter("device").value
        self._frame_id = self.get_parameter("frame_id").value
        if device.isdigit():
            self._cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
        else:
            self._cap = cv2.VideoCapture(device, cv2.CAP_GSTREAMER)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter("width_px").value)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter("height_px").value)
        if not self._cap.isOpened():
            self.get_logger().error(f"could not open camera '{device}'")

        self._pub = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter("fps_hz").value, self._tick)

    def _tick(self):
        ok, frame = self._cap.read()
        if not ok:
            self.get_logger().warn("no image from the camera", throttle_duration_sec=5.0)
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
