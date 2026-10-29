"""
Tests for the Auto mode state machine and its settings (run with:
pytest tools). The autopilot is pure logic, so no thread or hardware is
involved.
"""

import pytest

from teleop_web_lib.autopilot import AutoPilot, DriveCommand
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


def test_line_hit_during_the_delivery_push_counts_the_delivery():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    vision.zone = Seen("blue", 0.18)
    pilot.step(0.0)
    assert pilot.state == "DROP"
    pilot.step(0.2, line_black=True)
    assert pilot.delivered_count == 1 and not pilot.carrying
    vision.zone = None
    for step in range(1, 100):
        pilot.step(step * 0.1)
    assert not pilot.state.startswith("CARRY")


def test_line_hit_while_carrying_leaves_the_cube_and_searches_again():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    pilot.step(0.0, line_black=True)
    assert not pilot.carrying and pilot.delivered_count == 0
    for step in range(1, 100):
        pilot.step(step * 0.1)
    assert not pilot.state.startswith("CARRY")


def test_gives_up_carrying_when_the_gate_is_never_found():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # search clock at 0
    pilot.carrying = True
    pilot.state = "CARRY"
    timeout_s = pilot.cfg.carry_zone_timeout_s
    pilot.step(timeout_s - 1.0)
    assert pilot.carrying
    assert pilot.step(timeout_s + 0.1).speed == 0.0
    assert not pilot.carrying and pilot.state == "SEARCH"


def test_a_line_hit_while_going_for_a_cube_ignores_cubes_for_a_while():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.5)
    pilot.step(0.0)
    assert pilot.state == "APPROACH"
    pilot.step(0.1, line_black=True)  # reached the border line
    ban_s = pilot.cfg.line_cube_ban_s
    for step in range(3, 30):
        pilot.step(step * 0.1)
    assert pilot.state.startswith("EXPLORE")  # the cube is still in view
    pilot.step(0.1 + ban_s + 1.0)
    assert pilot.state in ("APPROACH", "ALIGN")  # ban over: tries again


def test_close_to_the_cube_a_small_bearing_error_does_not_realign():
    pilot, vision = make_pilot()
    tolerance_rad = pilot.cfg.align_tolerance_rad
    # Past the normal tolerance but within the widened one.
    bearing_rad = tolerance_rad * 1.2
    vision.det = Seen("red", 0.8, bearing_rad=bearing_rad)
    pilot.step(0.0)
    assert pilot.state == "ALIGN"  # far: turns to face it
    vision.det = Seen("red", 0.25, bearing_rad=bearing_rad)
    assert pilot.step(0.1).vy == 1.0 and pilot.state == "APPROACH"


def test_a_far_cube_that_flickers_out_keeps_the_heading_without_backing_up():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.5)  # far, to the left
    pilot.step(0.0)
    vision.det = None
    assert pilot.step(0.1).speed == 0.0  # waits briefly first
    command = pilot.step(0.7)
    assert pilot.state == "EXPLORE (reacquire)"
    assert command.vy == 0.0 and command.wz == 1.0  # turns toward it
    after_s = pilot.cfg.reacquire_s + 0.1
    pilot.step(after_s)
    assert pilot.state != "EXPLORE (reacquire)"  # memory expired: roams


def test_a_lost_cube_straight_ahead_is_approached_not_forgotten():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.0)
    pilot.step(0.0)
    vision.det = None
    pilot.step(0.1)
    command = pilot.step(0.7)
    assert pilot.state == "EXPLORE (reacquire)" and command.vy == 1.0


def test_backs_up_once_to_uncover_a_cube_that_vanished_from_view():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.25)  # close: may be under the scoop
    pilot.step(0.0)
    vision.det = None
    assert pilot.step(0.1).speed == 0.0  # waits briefly first
    command = pilot.step(0.7)
    assert command.vy == -1.0 and pilot.state == "EXPLORE (back up)"
    later_s = 0.7 + pilot.cfg.reveal_backup_s + 0.1
    command = pilot.step(later_s)
    assert command.vy != -1.0 and pilot.state != "EXPLORE (back up)"


def test_never_roams_forward_toward_a_close_gate_without_a_cube():
    pilot, vision = make_pilot()
    near_m = pilot.cfg.gate_avoid_distance_m - 0.1
    vision.zone = Seen("green", near_m, bearing_rad=0.3)  # to the left
    command = pilot.step(0.0)
    assert pilot.state == "EXPLORE (avoid gate)"
    assert command.vy == 0.0 and command.wz == -1.0  # turns right, away


def test_roams_normally_while_the_gate_is_far():
    pilot, vision = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)  # search clock at 0
    far_m = pilot.cfg.gate_avoid_distance_m + 0.5
    vision.zone = Seen("green", far_m)
    pilot.step(0.0)  # the first move is the back-up that uncovers cubes
    after_backup_s = pilot.cfg.reveal_backup_s + 0.1
    assert pilot.step(after_backup_s).vy == 1.0
    assert pilot.state == "EXPLORE (forward)"


def test_backs_up_first_when_auto_restarts_without_a_cube():
    pilot, _ = make_pilot()
    pilot.configure(now=0.0, enabled=True, reset=True)
    assert pilot.step(0.0).vy == -1.0
    pilot.mark_manual()  # a nudge by hand, then Auto takes over again
    assert pilot.step(5.0).vy == -1.0
    assert pilot.state == "EXPLORE (back up)"


def test_black_line_moves_away_opposite_to_the_last_motion():
    cases = [
        (
            DriveCommand(0.0, 1.0, 0.0, 0.4),
            0.0,
            -1.0,
        ),  # forward (W) -> back (S)
        (
            DriveCommand(0.0, -1.0, 0.0, 0.4),
            0.0,
            1.0,
        ),  # reverse (S) -> forward (W)
        (DriveCommand(-1.0, 0.0, 0.0, 0.4), 1.0, 0.0),  # left (A) -> right (D)
        (DriveCommand(1.0, 0.0, 0.0, 0.4), -1.0, 0.0),  # right (D) -> left (A)
    ]
    for moved, expect_vx, expect_vy in cases:
        pilot, _ = make_pilot()
        pilot.note_motion(0.0, moved)
        pilot.step(0.1, line_black=True)  # brake
        command = pilot.step(0.4, line_black=True)
        assert (command.vx, command.vy, command.wz) == (
            expect_vx,
            expect_vy,
            0.0,
        )


def test_black_line_backs_up_when_there_is_no_recent_motion():
    pilot, _ = make_pilot()
    pilot.note_motion(0.0, DriveCommand(0.0, -1.0, 0.0, 0.4))
    late = pilot.cfg.line_motion_memory_s + 1.0
    pilot.step(late, line_black=True)
    assert pilot.step(late + 0.3, line_black=True).vy == -1.0


def test_recovery_does_not_overwrite_the_direction_it_escapes_from():
    pilot, _ = make_pilot()
    pilot.note_motion(0.0, DriveCommand(0.0, -1.0, 0.0, 0.4))
    pilot.step(0.1, line_black=True)
    pilot.note_motion(0.2, DriveCommand(0.0, 1.0, 0.0, 0.4))  # escape move
    assert pilot.step(0.4, line_black=True).vy == 1.0


def test_black_line_stops_backs_up_then_searches_without_the_lost_cube():
    pilot, _ = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    command = pilot.step(0.0, line_black=True)
    assert command.speed == 0.0 and pilot.state == "LINE (stop)"
    command = pilot.step(0.3, line_black=True)
    assert command.vy == -1.0 and pilot.state == "LINE (back up)"
    pilot.step(1.0)  # white again: backs up the margin, then turns
    pilot.step(1.2)
    assert pilot.state == "LINE (turn)"
    pilot.step(2.0)
    # The cube stayed on the border: nothing left to carry.
    assert not pilot.carrying and pilot.state.startswith("EXPLORE")


def test_black_line_keeps_backing_up_while_still_on_black():
    pilot, _ = make_pilot()
    pilot.step(0.0, line_black=True)
    for t in (0.3, 0.6, 0.9):  # still black: never reaches the turn yet
        command = pilot.step(t, line_black=True)
        assert command.vy == -1.0 and pilot.state == "LINE (back up)"
    # Gives up backing after line_backup_max_s even if the sensor is stuck.
    states = set()
    for step in range(10, 30):
        pilot.step(step * 0.1, line_black=True)
        states.add(pilot.state)
    assert "LINE (turn)" in states


def test_reversing_onto_black_line_escapes_forward_without_turning_back():
    pilot, _ = make_pilot()
    pilot.note_motion(0.0, DriveCommand(0.0, -1.0, 0.0, 0.4))
    assert pilot.step(0.1, line_black=True).speed == 0.0
    assert pilot.step(0.4, line_black=True).vy == 1.0  # drives forward
    pilot.step(1.0, line_black=False)  # resumes without spinning 180 deg back
    assert not pilot.state.startswith("LINE")


def test_approach_aligns_before_driving():
    pilot, vision = make_pilot()
    vision.det = Seen("red", 0.8, bearing_rad=0.5)
    command = pilot.step(0.0)
    assert pilot.state == "ALIGN" and command.vy == 0.0 and command.wz != 0.0
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


def test_area_paper_arrives_closer_than_the_landmarks():
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    # 30 cm from the landmarks: arrived.
    vision.zone = Seen("gate", 0.30)
    pilot.step(0.0)
    assert pilot.state == "DROP"
    pilot, vision = make_pilot()
    pilot.carrying = True
    pilot.state = "CARRY"
    # 30 cm from the near edge of the paper: keep driving.
    vision.zone = Seen("floor", 0.30)
    pilot.step(0.0)
    assert pilot.state == "CARRY"
