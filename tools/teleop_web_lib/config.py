"""
YAML loading and typed settings for the browser tool. No hardware.

The real values live in tools/teleop_web.yaml. The defaults below are only
the fallback when a key is missing, so a typo cannot silently change the
robot's behaviour: unknown keys raise at startup.
"""

from dataclasses import dataclass, fields
from pathlib import Path

# --- Auto mode defaults (real values: tools/teleop_web.yaml, section auto) ----
DEFAULT_STOP_DISTANCE_M = 0.185
DEFAULT_SLOW_DOWN_DISTANCE_M = 0.45
DEFAULT_FORWARD_SPEED = 0.42
DEFAULT_ROAM_FORWARD_SPEED = 0.35
DEFAULT_TURN_SPEED = 0.38
DEFAULT_SEARCH_TURN_SPEED = 0.32
DEFAULT_ALIGN_TOLERANCE_RAD = 0.22
DEFAULT_LOST_TIMEOUT_S = 0.6
DEFAULT_NO_CUBES_STOP_S = 7.0
DEFAULT_SCOOP_PUSH_S = 0.65
DEFAULT_LINE_BACKUP_S = 0.6
DEFAULT_LINE_TURN_S = 1.2
DEFAULT_LINE_SPEED = 0.35
DEFAULT_ZONE_STOP_DISTANCE_M = 0.20
DEFAULT_ZONE_ARRIVE_V_PX = 440
DEFAULT_DELIVER_PUSH_S = 0.5
DEFAULT_DELIVER_BACKUP_S = 0.8
DEFAULT_DELIVER_TURN_S = 2.0
# Image row (px) from which a cube counts as at the scoop threshold.
DEFAULT_SCOOP_ARRIVE_V_PX = 445
# The catch / drop alignment tolerance is align_tolerance_rad times this.
DEFAULT_CATCH_ALIGN_FACTOR = 1.4
# Roaming: drive forward for `roam_forward_s` out of every `roam_period_s`,
# and turn to scan the rest of the time.
DEFAULT_ROAM_PERIOD_S = 3.0
DEFAULT_ROAM_FORWARD_S = 1.3
# Largest sideways correction while driving toward a target (0-1).
DEFAULT_STEER_LIMIT = 0.7
# Slowest approach speed, as a fraction of forward_speed.
DEFAULT_MIN_SPEED_FRACTION = 0.35
# Smallest slow-down span (m), avoids dividing by a near-zero distance.
DEFAULT_MIN_SLOW_SPAN_M = 0.05

# --- Vision filter used by this tool (section `detection` overrides) ----------
# Looser than the ROS cube_detector because this tool's camera is not
# calibrated as tightly and the scoop is allowed in view.
DEFAULT_DETECTION_FILTER = {
    "min_area_px": 60,
    "min_size_ratio": 0.15,
    "max_size_ratio": 6.0,
    "ignore_touching_bottom": False,
    "max_distance_m": 3.0,
}

# The vision package is plain Python: it is imported straight from the repo,
# without building the ROS workspace.
REPO_ROOT = Path(__file__).resolve().parents[2]
VISION_PACKAGE_DIR = REPO_ROOT / "src" / "vision"
HSV_CONFIG_PATH = VISION_PACKAGE_DIR / "config" / "hsv_ranges.yaml"
HSV_NODE_NAME = "cube_detector"

# Key of the parameters of a node inside a ROS parameter file.
ROS_PARAMS_KEY = "ros__parameters"

# Section names of tools/teleop_web.yaml.
SECTION_MOTORS = "motors"
SECTION_CAMERA = "camera"
SECTION_SERVER = "server"
SECTION_LINE_SENSOR = "line_sensor"
SECTION_AUTO = "auto"
SECTION_DETECTION = "detection"


@dataclass(frozen=True)
class AutoConfig:
    """
    Settings of the Auto mode; field names are the YAML keys.
    """

    enabled: bool = False
    stop_distance_m: float = DEFAULT_STOP_DISTANCE_M
    slow_down_distance_m: float = DEFAULT_SLOW_DOWN_DISTANCE_M
    forward_speed: float = DEFAULT_FORWARD_SPEED
    roam_forward_speed: float = DEFAULT_ROAM_FORWARD_SPEED
    turn_speed: float = DEFAULT_TURN_SPEED
    search_turn_speed: float = DEFAULT_SEARCH_TURN_SPEED
    align_tolerance_rad: float = DEFAULT_ALIGN_TOLERANCE_RAD
    lost_timeout_s: float = DEFAULT_LOST_TIMEOUT_S
    no_cubes_stop_s: float = DEFAULT_NO_CUBES_STOP_S
    scoop_push_s: float = DEFAULT_SCOOP_PUSH_S
    line_backup_s: float = DEFAULT_LINE_BACKUP_S
    line_turn_s: float = DEFAULT_LINE_TURN_S
    line_speed: float = DEFAULT_LINE_SPEED
    zone_stop_distance_m: float = DEFAULT_ZONE_STOP_DISTANCE_M
    zone_arrive_v_px: int = DEFAULT_ZONE_ARRIVE_V_PX
    deliver_push_s: float = DEFAULT_DELIVER_PUSH_S
    deliver_backup_s: float = DEFAULT_DELIVER_BACKUP_S
    deliver_turn_s: float = DEFAULT_DELIVER_TURN_S
    scoop_arrive_v_px: int = DEFAULT_SCOOP_ARRIVE_V_PX
    catch_align_factor: float = DEFAULT_CATCH_ALIGN_FACTOR
    roam_period_s: float = DEFAULT_ROAM_PERIOD_S
    roam_forward_s: float = DEFAULT_ROAM_FORWARD_S
    steer_limit: float = DEFAULT_STEER_LIMIT
    min_speed_fraction: float = DEFAULT_MIN_SPEED_FRACTION
    min_slow_span_m: float = DEFAULT_MIN_SLOW_SPAN_M

    @classmethod
    def from_dict(cls, values: dict | None) -> "AutoConfig":
        """
        Build the settings from the YAML `auto` section.

        Missing keys keep their default; unknown keys raise ValueError.
        """
        values = values or {}
        defaults = cls()
        known = {field.name for field in fields(cls)}
        unknown = sorted(set(values) - known)
        if unknown:
            raise ValueError(f"unknown settings in section auto: {unknown}")
        return cls(
            **{
                name: _coerce_like(getattr(defaults, name), value)
                for name, value in values.items()
            }
        )


def _coerce_like(default, value):
    """
    Convert `value` to the type of `default` (bool, int or float).
    """
    return type(default)(value)


def load_config(path: Path) -> dict:
    """
    Read the YAML file into a dictionary.
    """
    # Imported here: PyYAML is only needed when the tool actually runs.
    import yaml

    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def read_node_params(path: Path, node_name: str) -> dict:
    """
    The `ros__parameters` of `node_name` in a ROS parameter YAML.

    The node key may carry a namespace wildcard (`/**/cube_detector`), so it
    is matched by its last path segment.
    """
    document = load_config(path)
    for key, value in document.items():
        if key.rsplit("/", 1)[-1] == node_name:
            return value[ROS_PARAMS_KEY]
    raise KeyError(f"no section for node {node_name!r} in {path}")
