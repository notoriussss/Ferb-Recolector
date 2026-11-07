"""
Tests for the Auto state-change log line (run with: pytest tools).
"""

from teleop_web_lib.autopilot import AutoPilot
from teleop_web_lib.config import AutoConfig
from teleop_web_lib.state_log import StateLog, auto_snapshot


class GrantingFleet:
    """
    A fleet link that always grants the turn.
    """

    def update(self, state, want, carrying):
        pass

    def granted(self):
        return True

    def delivered(self, color):
        pass


def test_a_line_only_when_something_changes():
    pilot = AutoPilot(AutoConfig())
    log = StateLog()
    first = log.line(auto_snapshot(pilot), pilot.target_info)
    assert first is not None and "state=PAUSED" in first
    assert log.line(auto_snapshot(pilot), pilot.target_info) is None
    pilot.state = "CARRY (wait zone)"
    line = log.line(auto_snapshot(pilot), "zone floor 28cm")
    assert "state=CARRY (wait zone)" in line and "zone floor 28cm" in line


def test_the_line_says_what_the_fleet_was_told_and_answered():
    pilot = AutoPilot(AutoConfig())
    log = StateLog()
    line = log.line(auto_snapshot(pilot), None)
    assert "want=False granted=-" in line  # no fleet: no turns
    pilot.fleet = GrantingFleet()
    pilot.carrying = True
    pilot.state = "DROP"
    pilot.mark_paused()  # syncs the fleet: no longer wants the turn
    line = log.line(auto_snapshot(pilot), None)
    assert "carrying=True want=False granted=True" in line


def test_the_target_alone_does_not_make_a_line():
    pilot = AutoPilot(AutoConfig())
    log = StateLog()
    log.line(auto_snapshot(pilot), "yellow 24cm")
    assert log.line(auto_snapshot(pilot), "yellow 23cm") is None
