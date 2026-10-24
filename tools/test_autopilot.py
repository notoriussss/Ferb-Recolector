"""
Tests for the Auto mode state machine and its settings (run with:
pytest tools). The autopilot is pure logic, so no thread or hardware is
involved.
"""

import pytest

from teleop_web_lib.autopilot import AutoPilot
from teleop_web_lib.config import (
    HSV_CONFIG_PATH,
    HSV_NODE_NAME,
    AutoConfig,
    read_node_params,
)

FAST_PHASES = {
    "enabled": True,
    "scoop_push_s": 0.5,
    "deliver_push_s": 0.5,
    "deliver_backup_s": 0.5,
    "deliver_turn_s": 0.5,
    "line_backup_s": 0.5,
    "line_turn_s": 0.5,
}


class StubVision:
    """
    Returns whatever cube and zone it is given.
    """

    def __init__(self):
        self.det = None
        self.zone = None

    def latest_detection(self, now, timeout_s):
        return self.det

    def latest_zone(self, now, timeout_s):
        return self.zone


class Seen:
    """
    A detection or region: colour, distance, bearing and bottom row.
    """

    def __init__(self, color, distance_m, bearing_rad=0.0, v=300):
        self.color = color
        self.distance_m = distance_m
        self.bearing_rad = bearing_rad
        self.v = v


def make_pilot():
    vision = StubVision()
    pilot = AutoPilot(AutoConfig(**FAST_PHASES), vision)
    pilot.state = "SEARCH"
    return pilot, vision


def test_pick_carry_deliver_and_count():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.15)
    assert pilot.step(0.0).vy == 1.0 and pilot.state == "CATCH"
    vision.det = None
    pilot.step(1.0)
    assert pilot.carrying and pilot.state == "CARRY"
    pilot.step(1.1)
    assert pilot.state.startswith("CARRY")  # no zone: looks for it
    vision.zone = Seen("green", 0.6, bearing_rad=0.5)
    pilot.step(1.2)
    assert pilot.state == "CARRY (align)"
    vision.zone = Seen("blue", 0.18)
    pilot.step(1.3)
    assert pilot.state == "DROP"
    assert pilot.step(2.0).vy == -1.0  # backs up leaving the cube
    assert pilot.delivered_count == 1 and not pilot.carrying
    pilot.step(2.4)
    assert pilot.state == "DROP (turn)"
    pilot.step(3.0)
    assert pilot.state.startswith("EXPLORE")


def test_black_line_backs_up_then_resumes_carrying():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    command = pilot.step(0.0, line_black=True)
    assert command.vy == -1.0 and pilot.state == "LINE (back up)"
    pilot.step(0.7)
    assert pilot.state == "LINE (turn)"
    pilot.step(1.2)
    assert pilot.state.startswith("CARRY")


def test_approach_aligns_before_driving():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.5)
    command = pilot.step(0.0)
    assert pilot.state == "ALIGN" and command.vy == 0.0 and command.vx != 0.0
    vision.det = Seen("red", 0.8, bearing_rad=0.0)
    command = pilot.step(0.1)
    assert pilot.state == "APPROACH" and command.vy == 1.0


def test_gives_up_when_no_cube_is_ever_seen():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # search clock at 0
    pilot.step(0.0)  # starts roaming
    assert pilot.step(pilot.cfg.no_cubes_stop_s + 1.0).speed == 0.0
    assert pilot.state == "NO CUBES (stopped)"
    # Stays stopped until Auto is restarted.
    assert pilot.step(pilot.cfg.no_cubes_stop_s + 2.0).speed == 0.0


def test_reset_clears_counters_and_restarts_search():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.delivered_count = 3
    pilot.collected_count = 4
    pilot.configure(now=5.0, enabled=True, reset=True)
    assert pilot.state == "SEARCH"
    assert (pilot.carrying, pilot.delivered_count, pilot.collected_count) == (
        False,
        0,
        0,
    )


def test_disabling_pauses_and_enabling_resumes_carrying():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.configure(now=0.0, enabled=False)
    assert pilot.state == "PAUSED"
    pilot.configure(now=1.0, enabled=True)
    assert pilot.state == "CARRY"


def test_auto_config_defaults_and_overrides():
    assert AutoConfig.from_dict(None) == AutoConfig()
    config = AutoConfig.from_dict({"forward_speed": 1, "enabled": True})
    assert config.forward_speed == 1.0 and isinstance(
        config.forward_speed, float
    )
    assert config.enabled is True


def test_auto_config_rejects_unknown_keys():
    with pytest.raises(ValueError, match="forwrad_speed"):
        AutoConfig.from_dict({"forwrad_speed": 0.4})


def test_reads_the_shared_hsv_file_with_or_without_namespace_wildcard(tmp_path):
    body = "{node}:\n  ros__parameters:\n    a: 1\n"
    plain = tmp_path / "plain.yaml"
    plain.write_text(body.format(node="cube_detector"))
    wild = tmp_path / "wild.yaml"
    wild.write_text(body.format(node="/**/cube_detector"))
    assert read_node_params(plain, "cube_detector") == {"a": 1}
    assert read_node_params(wild, "cube_detector") == {"a": 1}
    with pytest.raises(KeyError):
        read_node_params(plain, "web_viewer")


def test_the_real_vision_config_has_what_the_tool_needs():
    params = read_node_params(HSV_CONFIG_PATH, HSV_NODE_NAME)
    assert {"hsv", "camera", "filter"} <= set(params)
    assert {"hfov_rad", "height_m", "tilt_rad"} <= set(params["camera"])
