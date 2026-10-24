"""Tests for the kinematics, timeout brake and Auto mode of tools/teleop_web.py (pytest tools)."""

import time

from teleop_web import Drive, mecanum_wheel_ratios


def test_forward_all_wheels_same():
    assert mecanum_wheel_ratios(1, 0, 0) == {"fl": 1, "fr": 1, "rl": 1, "rr": 1}


def test_strafe_left():
    r = mecanum_wheel_ratios(0, 1, 0)
    assert r == {"fl": -1, "fr": 1, "rl": 1, "rr": -1}


def test_turn_ccw():
    r = mecanum_wheel_ratios(0, 0, 1)
    assert r == {"fl": -1, "fr": 1, "rl": -1, "rr": 1}


def test_combined_is_normalized():
    r = mecanum_wheel_ratios(1, 1, 1)
    assert max(abs(v) for v in r.values()) == 1


class RecordingMotors:
    def __init__(self):
        self.last = None

    def set(self, ratios):
        self.last = ratios

    def close(self):
        pass


def test_brakes_without_commands():
    motors = RecordingMotors()
    drive = Drive(motors, max_duty_ratio=0.5, cmd_timeout_s=0.1)
    drive.command(1, 0, 0, 1)
    time.sleep(0.05)
    assert motors.last["fl"] == 0.5
    time.sleep(0.2)
    assert motors.last == {"fl": 0, "fr": 0, "rl": 0, "rr": 0}
    drive.close()


class StubCamera:
    """Stub camera: returns whatever cube and zone it is given."""

    def __init__(self):
        self.det = None
        self.zone = None

    def latest_detection(self, now, timeout=0.6):
        return (self.det, 1) if self.det else (None, 0)

    def latest_zone(self, now, timeout=0.6):
        return self.zone


class Seen:
    def __init__(self, color, distance_m, bearing_rad=0.0, v=300):
        self.color, self.distance_m, self.bearing_rad, self.v = color, distance_m, bearing_rad, v


class StubLine:
    black = False

    def is_black(self):
        return self.black


def make_auto():
    cfg = {
        "enabled": True,
        "scoop_push_s": 0.5,
        "deliver_push_s": 0.5,
        "deliver_backup_s": 0.5,
        "deliver_turn_s": 0.5,
        "line_backup_s": 0.5,
        "line_turn_s": 0.5,
    }
    drive = Drive(RecordingMotors(), 0.5, 0.1, cfg, line_sensor=StubLine())
    drive.close()  # no thread: _auto_step is called by hand
    cam = StubCamera()
    drive.set_camera(cam)
    drive.auto_state = "SEARCH"
    return drive, cam


def test_pick_carry_deliver_and_count():
    drive, cam = make_auto()
    cam.det = Seen("red", 0.15)
    assert drive._auto_step(0.0)[1] == 1.0 and drive.auto_state == "CATCH"
    cam.det = None
    drive._auto_step(1.0)
    assert drive.carrying and drive.auto_state == "CARRY"
    drive._auto_step(1.1)
    assert drive.auto_state.startswith("CARRY")  # no zone: looks for it
    cam.zone = Seen("green", 0.6, bearing_rad=0.5)
    drive._auto_step(1.2)
    assert drive.auto_state == "CARRY (align)"
    cam.zone = Seen("blue", 0.18)
    drive._auto_step(1.3)
    assert drive.auto_state == "DROP"
    assert drive._auto_step(2.0)[1] == -1.0  # backs up leaving the cube
    assert drive.delivered_count == 1 and not drive.carrying
    drive._auto_step(2.4)
    assert drive.auto_state == "DROP (turn)"
    drive._auto_step(3.0)
    assert drive.auto_state.startswith("EXPLORE")


def test_black_line_backs_up_then_resumes_carrying():
    drive, cam = make_auto()
    drive.carrying = True
    drive.auto_state = "CARRY"
    drive.line_black = True
    assert drive._auto_step(0.0)[1] == -1.0 and drive.auto_state == "LINE (back up)"
    drive.line_black = False
    drive._auto_step(0.7)
    assert drive.auto_state == "LINE (turn)"
    drive._auto_step(1.2)
    assert drive.auto_state.startswith("CARRY")
