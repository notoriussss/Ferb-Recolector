"""
Tests for the kinematics, timeout brake and control thread of the browser
tool (run with: pytest tools).
"""

import time

from teleop_web_lib.autopilot import STATE_MANUAL, STATE_PAUSED
from teleop_web_lib.drive import Drive
from teleop_web_lib.kinematics import mecanum_wheel_ratios

STOPPED = {"fl": 0, "fr": 0, "rl": 0, "rr": 0}


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
    """
    Motors that remember the last ratios they were given.
    """

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
    assert motors.last == STOPPED
    drive.close()


def test_manual_command_is_reported_as_manual_then_paused():
    drive = Drive(RecordingMotors(), max_duty_ratio=0.5, cmd_timeout_s=0.1)
    drive.command(1, 0, 0, 1)
    time.sleep(0.05)
    assert drive.status()["state"] == STATE_MANUAL
    time.sleep(0.2)
    assert drive.status()["state"] == STATE_PAUSED
    drive.close()


def test_status_has_the_keys_the_page_reads():
    drive = Drive(RecordingMotors(), max_duty_ratio=0.5, cmd_timeout_s=0.1)
    assert set(drive.status()) == {
        "auto",
        "state",
        "target",
        "collected",
        "delivered",
        "line",
    }
    drive.close()


def test_set_auto_switches_the_mode():
    drive = Drive(RecordingMotors(), max_duty_ratio=0.5, cmd_timeout_s=0.1)
    assert drive.status()["auto"] is False
    drive.set_auto(True)
    assert drive.status()["auto"] is True
    drive.set_auto(False)
    assert drive.status()["auto"] is False
    drive.close()
