"""
behavior_fsm node: the collector's state machine (search, approach, catch,
carry to the square, release) and count of delivered cubes.

For now it only does SEARCH -> APPROACH -> ARRIVED (see approach.py): it
turns until it sees a cube of the target colours, approaches it and stops at
`stop_distance_m`. Publishes cmd_vel_raw (goes through obstacle_guard) and
behavior/state.

Manual control: if cmd_vel_teleop arrives (WASD from web_viewer), it is
passed as-is to cmd_vel_raw and the state is MANUAL, even when the behaviour
is paused. When it stops arriving for `teleop_timeout_s`, automatic
behaviour resumes.

Parameters that can be changed at runtime (for example from web_viewer):
  enabled        true / false
  target_colors  list of colours, e.g. ["red", "orange"]

TODO (role 3): CATCH, CARRY, RELEASE and the count once the scoop exists.
TODO (delivery): request the zone (`zone/want=true`) before crossing into
the square, enter only when `zone/granted` is true, publish the colour on
`zone/delivered` after the drop, and then set `zone/want=false`.
"""

import json
from dataclasses import dataclass, fields

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mobile_base.approach import ApproachController, ApproachParams, Cube
from mobile_base.node_utils import now_s, spin_node
from mobile_base.params import declare_params, read_params
from mobile_base.states import State
from mobile_base.topics import (
    DEFAULT_QOS_DEPTH,
    TOPIC_BEHAVIOR_STATE,
    TOPIC_CMD_VEL_RAW,
    TOPIC_CMD_VEL_TELEOP,
    TOPIC_CUBES,
    TOPIC_ZONE_DELIVERED,
    TOPIC_ZONE_WANT,
)
from recolector_interfaces.msg import CubeDetectionArray

NODE_NAME = "behavior_fsm"

# Decimals kept when the state report is rounded for the web viewer.
REPORT_DECIMALS = 3

# Names of the ApproachParams fields that must not be negative.
NUMERIC_PARAM_NAMES = tuple(
    f.name for f in fields(ApproachParams) if f.name != "target_colors"
)


@dataclass
class BehaviorNodeParams:
    """
    Node-level parameters (the controller's own live in ApproachParams).

    Field names double as ROS parameter names (see params.py).
    """

    enabled: bool = True
    control_rate_hz: float = 20.0
    # Without detections newer than this, stay still.
    cubes_timeout_s: float = 1.0
    # Without WASD commands newer than this, automatic mode resumes.
    teleop_timeout_s: float = 0.3


class BehaviorFsmNode(Node):
    """
    Runs the approach controller and arbitrates manual vs automatic control.
    """

    def __init__(self):
        super().__init__(NODE_NAME)
        declare_params(self, BehaviorNodeParams())
        declare_params(self, ApproachParams())
        self._node_params = read_params(self, BehaviorNodeParams())
        self._ctrl = ApproachController(read_params(self, ApproachParams()))
        self._ctrl.enabled = self._node_params.enabled
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_changed)

        self._cubes: list[Cube] = []
        self._cubes_stamp_s: float | None = None
        self._last_state: str | None = None
        self._teleop = Twist()
        self._teleop_stamp_s: float | None = None

        self._pub_cmd = self.create_publisher(
            Twist,
            TOPIC_CMD_VEL_RAW,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_state = self.create_publisher(
            String,
            TOPIC_BEHAVIOR_STATE,
            DEFAULT_QOS_DEPTH,
        )
        # Reserved for the delivery flow (see the module TODO).
        self._pub_zone_want = self.create_publisher(
            Bool,
            TOPIC_ZONE_WANT,
            DEFAULT_QOS_DEPTH,
        )
        self._pub_zone_delivered = self.create_publisher(
            String,
            TOPIC_ZONE_DELIVERED,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            CubeDetectionArray,
            TOPIC_CUBES,
            self._on_cubes,
            DEFAULT_QOS_DEPTH,
        )
        self.create_subscription(
            Twist,
            TOPIC_CMD_VEL_TELEOP,
            self._on_teleop,
            DEFAULT_QOS_DEPTH,
        )
        self.create_timer(1.0 / self._node_params.control_rate_hz, self._tick)
        self.get_logger().info(
            f"target colours: {self._ctrl.params.target_colors}"
        )

    # ---------- parameters ----------
    def _validate_params(self, params):
        """
        Reject negative values for the numeric controller parameters.
        """
        for p in params:
            if p.name in NUMERIC_PARAM_NAMES and p.value < 0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} cannot be < 0",
                )
        return SetParametersResult(successful=True)

    def _on_params_changed(self, _params):
        """
        Reload the parameters into the controller after a runtime change.
        """
        self._node_params = read_params(self, BehaviorNodeParams())
        self._ctrl.params = read_params(self, ApproachParams())
        self._ctrl.enabled = self._node_params.enabled
        self.get_logger().info(
            f"enabled={self._ctrl.enabled} "
            f"target colours={self._ctrl.params.target_colors} "
            f"stop at {self._ctrl.params.stop_distance_m:.2f} m"
        )

    # ---------- subscriptions ----------
    def _on_cubes(self, msg: CubeDetectionArray):
        """
        Store the latest detections with their arrival time.
        """
        self._cubes = [
            Cube(c.color, c.bearing_rad, c.distance_m) for c in msg.cubes
        ]
        self._cubes_stamp_s = now_s(self)

    def _on_teleop(self, msg: Twist):
        """
        Store the latest manual command with its arrival time.
        """
        self._teleop = msg
        self._teleop_stamp_s = now_s(self)

    # ---------- control loop ----------
    def _tick(self):
        """
        One control step: decide the source of the command, publish it and
        report the state.
        """
        now = now_s(self)
        manual = self._is_manual(now)
        stale = self._is_vision_stale(now)
        if stale:
            # No vision, no movement.
            cmd = None
            self._ctrl.target = None
        else:
            cmd = self._ctrl.update(self._cubes, now)

        twist = self._select_twist(manual, cmd)
        self._pub_cmd.publish(twist)
        self._publish_state(self._current_state(manual, stale), twist)

    def _is_manual(self, now: float) -> bool:
        """
        True while WASD commands keep arriving.
        """
        if self._teleop_stamp_s is None:
            return False
        age_s = now - self._teleop_stamp_s
        return age_s <= self._node_params.teleop_timeout_s

    def _is_vision_stale(self, now: float) -> bool:
        """
        True when there are no recent detections.
        """
        if self._cubes_stamp_s is None:
            return True
        age_s = now - self._cubes_stamp_s
        return age_s > self._node_params.cubes_timeout_s

    def _select_twist(self, manual: bool, cmd) -> Twist:
        """
        Manual command if teleop is active, else the controller's command,
        else zero.
        """
        twist = Twist()
        if manual:
            twist.linear.x = self._teleop.linear.x
            twist.linear.y = self._teleop.linear.y
            twist.angular.z = self._teleop.angular.z
        elif cmd is not None:
            twist.linear.x = cmd.vx
            twist.angular.z = cmd.wz
        return twist

    def _current_state(self, manual: bool, stale: bool) -> State:
        """
        State to report: manual and missing-camera override the controller.
        """
        if manual:
            return State.MANUAL
        if stale:
            return State.NO_CAMERA
        return self._ctrl.state

    # ---------- reporting ----------
    def _publish_state(self, state: State, twist: Twist):
        """
        Publish the JSON state report and log state changes.
        """
        report = self._state_report(state, twist)
        self._pub_state.publish(String(data=json.dumps(report)))
        if state != self._last_state:
            self.get_logger().info(f"state: {state.value}")
            self._last_state = state

    def _state_report(self, state: State, twist: Twist) -> dict:
        """
        Dictionary published on behavior/state (read by web_viewer and
        fleet_link).
        """
        target = self._ctrl.target
        target_info = None
        if target is not None:
            target_info = {
                "color": target.color,
                "distance_m": round(target.distance_m, REPORT_DECIMALS),
                "bearing_rad": round(target.bearing_rad, REPORT_DECIMALS),
            }
        return {
            "state": state.value,
            "enabled": self._ctrl.enabled,
            "target_colors": self._ctrl.params.target_colors,
            "stop_distance_m": self._ctrl.params.stop_distance_m,
            "target": target_info,
            "cmd": {
                "vx": round(twist.linear.x, REPORT_DECIMALS),
                "vy": round(twist.linear.y, REPORT_DECIMALS),
                "wz": round(twist.angular.z, REPORT_DECIMALS),
            },
        }


def main(args=None):
    """
    Entry point of the behavior_fsm node.
    """
    spin_node(BehaviorFsmNode, args)


if __name__ == "__main__":
    main()
