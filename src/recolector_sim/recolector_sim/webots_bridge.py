"""webots_bridge node: Webots <extern> controller for the simulated robot.

In simulation it does what camera_node, ultrasonic and motor_driver do on the real robot,
with the same topics:
  publishes   image_raw (sensor_msgs/Image, bgr8) from the `camara` device
  publishes   range (sensor_msgs/Range) from `ultrasonido`
  subscribes  cmd_vel (geometry_msgs/Twist) and drives the 4 mecanum motors

If cmd_vel stops arriving for more than `cmd_timeout_s`, it stops (same as motor_driver).

It needs the Webots controller library: simulation.launch.py sets up
PYTHONPATH, LD_LIBRARY_PATH and WEBOTS_CONTROLLER_URL from WEBOTS_HOME.
"""

import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range

from mobile_base.kinematics import WHEELS, MecanumGeometry, scale_to_limit, wheel_speeds
from vision.ros_image import bgr_to_image

try:
    from controller import Robot
except ImportError:  # reported in main()
    Robot = None


class WebotsBridge(Node):
    def __init__(self, robot):
        super().__init__("webots_bridge")
        # RecolectorUCAB PROTO measurements (simulation)
        self.declare_parameter("wheel_radius_m", 0.0325)
        self.declare_parameter("half_wheelbase_m", 0.0605)
        self.declare_parameter("half_track_m", 0.0655)
        self.declare_parameter("max_wheel_speed_rad_s", 7.35)  # 10.5 rad/s x 70 % (2S battery)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("camera_period_ms", 64)
        self.declare_parameter("range_period_ms", 32)

        self._robot = robot
        self._step_ms = int(robot.getBasicTimeStep())
        self._geo = MecanumGeometry(
            self.get_parameter("wheel_radius_m").value,
            self.get_parameter("half_wheelbase_m").value,
            self.get_parameter("half_track_m").value,
        )
        self._max_wheel = self.get_parameter("max_wheel_speed_rad_s").value
        self._cmd_timeout_s = self.get_parameter("cmd_timeout_s").value

        self._motors = {}
        for n in WHEELS:
            m = robot.getDevice(f"motor_{n}")
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            self._motors[n] = m
        robot.getDevice("servo_pala").setPosition(0.0)

        self._cam_period_ms = max(self._step_ms, self.get_parameter("camera_period_ms").value)
        self._camera = robot.getDevice("camara")
        self._camera.enable(self._cam_period_ms)
        self._range_period_ms = max(self._step_ms, self.get_parameter("range_period_ms").value)
        self._sonar = robot.getDevice("ultrasonido")
        self._sonar.enable(self._range_period_ms)

        self._pub_image = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self._pub_range = self.create_publisher(Range, "range", qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)

        self._cmd = Twist()
        self._cmd_time_s: float | None = None
        self._last_cam_ms = -1e9
        self._last_range_ms = -1e9
        self.get_logger().info(f"connected to Webots robot '{robot.getName()}'")

    def _on_cmd(self, msg: Twist):
        self._cmd = msg
        self._cmd_time_s = self._robot.getTime()

    def step(self) -> bool:
        """One simulation step. False when Webots closes or resets."""
        if self._robot.step(self._step_ms) == -1:
            return False
        t_ms = self._robot.getTime() * 1000.0
        self._apply_cmd()
        if t_ms - self._last_cam_ms >= self._cam_period_ms:
            self._last_cam_ms = t_ms
            self._publish_image()
        if t_ms - self._last_range_ms >= self._range_period_ms:
            self._last_range_ms = t_ms
            self._publish_range()
        return True

    def _apply_cmd(self):
        fresh = (
            self._cmd_time_s is not None
            and self._robot.getTime() - self._cmd_time_s <= self._cmd_timeout_s
        )
        if fresh:
            c = self._cmd
            speeds = wheel_speeds(c.linear.x, c.linear.y, c.angular.z, self._geo)
            speeds = scale_to_limit(speeds, self._max_wheel)
        else:
            speeds = dict.fromkeys(WHEELS, 0.0)
        for n, v in speeds.items():
            self._motors[n].setVelocity(v)

    def _publish_image(self):
        try:
            raw = self._camera.getImage()
        except ValueError:  # over TCP, the first steps arrive without an image
            return
        if not raw:
            return
        h, w = self._camera.getHeight(), self._camera.getWidth()
        bgr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        msg = bgr_to_image(bgr)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self._pub_image.publish(msg)

    def _publish_range(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = float(self._sonar.getValue())
        self._pub_range.publish(msg)


def main(args=None):
    if Robot is None:
        print(
            "[webots_bridge] Webots 'controller' module not found. "
            "Launch it with simulation.launch.py or add $WEBOTS_HOME/lib/controller/python "
            "to PYTHONPATH.",
            file=sys.stderr,
        )
        sys.exit(1)
    rclpy.init(args=args)
    robot = Robot()  # waits for Webots to accept the connection (WEBOTS_CONTROLLER_URL)
    node = WebotsBridge(robot)
    try:
        while rclpy.ok() and node.step():
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
