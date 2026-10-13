"""Nodo behavior_fsm: máquina de estados del recolector (buscar, acercarse, atrapar,
llevar al cuadrado, soltar) y conteo de cubos entregados.

Por ahora solo hace BUSCAR -> ACERCARSE -> LLEGUE (ver approach.py): gira hasta ver un cubo
de los colores objetivo, se acerca y se detiene a `stop_distance_m`. Publica cmd_vel_raw
(pasa por obstacle_guard) y behavior/state.

Parámetros que se pueden cambiar en caliente (por ejemplo desde web_viewer):
  enabled        true / false
  target_colors  lista de colores, p. ej. ["rojo", "naranja"]

TODO (rol 3): ATRAPAR, LLEVAR, SOLTAR y el conteo cuando exista la pala.
"""

import json

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String

from mobile_base.approach import ApproachController, ApproachParams, Cube
from recolector_interfaces.msg import CubeDetectionArray

_NUMERIC_PARAMS = (
    "stop_distance_m",
    "resume_margin_m",
    "slow_down_distance_m",
    "max_forward_m_s",
    "min_forward_m_s",
    "turn_gain",
    "max_turn_rad_s",
    "align_tolerance_rad",
    "search_turn_rad_s",
    "lost_timeout_s",
)


class BehaviorFsmNode(Node):
    def __init__(self):
        super().__init__("behavior_fsm")
        defaults = ApproachParams()
        self.declare_parameter("enabled", True)
        self.declare_parameter("target_colors", defaults.target_colors)
        for name in _NUMERIC_PARAMS:
            self.declare_parameter(name, float(getattr(defaults, name)))
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("cubes_timeout_s", 1.0)

        self._ctrl = ApproachController(self._read_params())
        self._ctrl.enabled = self.get_parameter("enabled").value
        self.add_on_set_parameters_callback(self._validate_params)
        self.add_post_set_parameters_callback(self._on_params_changed)

        self._cubes: list[Cube] = []
        self._cubes_stamp_s: float | None = None
        self._last_state = None

        self._pub_cmd = self.create_publisher(Twist, "cmd_vel_raw", 10)
        self._pub_state = self.create_publisher(String, "behavior/state", 10)
        self.create_subscription(CubeDetectionArray, "cubes", self._on_cubes, 10)
        self.create_timer(1.0 / self.get_parameter("control_rate_hz").value, self._tick)
        self.get_logger().info(f"colores objetivo: {self._ctrl.params.target_colors}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _read_params(self) -> ApproachParams:
        p = ApproachParams(target_colors=list(self.get_parameter("target_colors").value))
        for name in _NUMERIC_PARAMS:
            setattr(p, name, float(self.get_parameter(name).value))
        return p

    def _validate_params(self, params):
        for p in params:
            if p.name in _NUMERIC_PARAMS and p.value < 0:
                return SetParametersResult(successful=False, reason=f"{p.name} no puede ser < 0")
        return SetParametersResult(successful=True)

    def _on_params_changed(self, _params):
        self._ctrl.params = self._read_params()
        self._ctrl.enabled = self.get_parameter("enabled").value
        self.get_logger().info(
            f"enabled={self._ctrl.enabled} colores objetivo={self._ctrl.params.target_colors} "
            f"parar a {self._ctrl.params.stop_distance_m:.2f} m"
        )

    def _on_cubes(self, msg: CubeDetectionArray):
        self._cubes = [Cube(c.color, c.bearing_rad, c.distance_m) for c in msg.cubes]
        self._cubes_stamp_s = self._now_s()

    def _tick(self):
        now = self._now_s()
        stale = (
            self._cubes_stamp_s is None
            or now - self._cubes_stamp_s > self.get_parameter("cubes_timeout_s").value
        )
        if stale:
            # sin visión no se mueve
            cmd = None
            self._ctrl.target = None
        else:
            cmd = self._ctrl.update(self._cubes, now)

        twist = Twist()
        if cmd is not None:
            twist.linear.x = cmd.vx
            twist.angular.z = cmd.wz
        self._pub_cmd.publish(twist)

        state = "SIN_CAMARA" if stale else self._ctrl.state.value
        target = self._ctrl.target
        info = {
            "state": state,
            "enabled": self._ctrl.enabled,
            "target_colors": self._ctrl.params.target_colors,
            "stop_distance_m": self._ctrl.params.stop_distance_m,
            "target": None
            if target is None
            else {
                "color": target.color,
                "distance_m": round(target.distance_m, 3),
                "bearing_rad": round(target.bearing_rad, 3),
            },
            "cmd": {"vx": round(twist.linear.x, 3), "wz": round(twist.angular.z, 3)},
        }
        self._pub_state.publish(String(data=json.dumps(info)))
        if state != self._last_state:
            self.get_logger().info(f"estado: {state}")
            self._last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
