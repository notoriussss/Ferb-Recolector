"""obstacle_guard node: limits the cmd_vel_raw speed based on the ultrasonic distance
and publishes cmd_vel.

Levels are in speed_limits.py. Without a recent `range`, forward motion is blocked (turning,
strafing and reversing are still allowed). If cmd_vel_raw stops arriving, it publishes zero.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from mobile_base.speed_limits import GuardParams, limit_forward


class ObstacleGuardNode(Node):
    def __init__(self):
        super().__init__("obstacle_guard")
        d = GuardParams()
        self.declare_parameter("slow_distance_m", d.slow_distance_m)
        self.declare_parameter("stop_distance_m", d.stop_distance_m)
        self.declare_parameter("slow_factor", d.slow_factor)
        self.declare_parameter("range_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("rate_hz", 20.0)

        self._range_m: float | None = None
        self._range_stamp_s: float | None = None
        self._cmd = Twist()
        self._cmd_stamp_s: float | None = None

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Twist, "cmd_vel_raw", self._on_cmd, 10)
        self.create_subscription(Range, "range", self._on_range, qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist):
        self._cmd = msg
        self._cmd_stamp_s = self._now_s()

    def _on_range(self, msg: Range):
        if math.isfinite(msg.range):
            self._range_m = msg.range
            self._range_stamp_s = self._now_s()

    def _tick(self):
        now = self._now_s()
        out = Twist()
        cmd_fresh = (
            self._cmd_stamp_s is not None
            and now - self._cmd_stamp_s <= self.get_parameter("cmd_timeout_s").value
        )
        if cmd_fresh:
            range_fresh = (
                self._range_stamp_s is not None
                and now - self._range_stamp_s <= self.get_parameter("range_timeout_s").value
            )
            params = GuardParams(
                slow_distance_m=self.get_parameter("slow_distance_m").value,
                stop_distance_m=self.get_parameter("stop_distance_m").value,
                slow_factor=self.get_parameter("slow_factor").value,
            )
            out.linear.x = limit_forward(
                self._cmd.linear.x, self._range_m if range_fresh else None, params
            )
            out.linear.y = self._cmd.linear.y
            out.angular.z = self._cmd.angular.z
            if out.linear.x == 0.0 and self._cmd.linear.x > 0.0:
                self.get_logger().warn(
                    "forward blocked: obstacle nearby or no ultrasonic reading",
                    throttle_duration_sec=2.0,
                )
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
