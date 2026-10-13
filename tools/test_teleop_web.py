"""Pruebas de la cinemática y el freno por tiempo de tools/teleop_web.py (pytest tools)."""

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
