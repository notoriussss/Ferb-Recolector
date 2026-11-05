"""
Tests for the peer-robot guard of the Auto mode (run with: pytest tools).
"""

import math

from teleop_web_lib.autopilot import AutoPilot, DriveCommand
from teleop_web_lib.config import AutoConfig, PeerConfig
from teleop_web_lib.peer import PeerGuard, Pose2D, from_velocity, to_velocity

FORWARD = DriveCommand(0.0, 1.0, 0.0, 0.36)
OWN = Pose2D(0.0, 0.0, 0.0, 10.0)  # at the origin, facing +x


def peer_at(x, y, stamp=10.0):
    return Pose2D(x, y, 0.0, stamp)


def test_commands_go_both_ways_unchanged():
    for cmd in (FORWARD, DriveCommand(-1.0, 0.0, 0.0, 0.3)):
        back = from_velocity(to_velocity(cmd))
        for got, want in zip(back, cmd, strict=True):
            assert math.isclose(got, want, abs_tol=1e-9)


def test_a_peer_close_ahead_stops_the_forward_move_and_turns_away():
    guard = PeerGuard(PeerConfig())
    cmd, reason = guard.filter(FORWARD, (OWN, peer_at(0.3, 0.05)), 10.0)
    assert cmd.vy * cmd.speed <= 0.0  # no forward motion
    assert cmd.wz < 0  # peer slightly left: turn right
    assert "stop" in reason


def test_a_peer_in_the_slow_band_only_slows_down():
    guard = PeerGuard(PeerConfig())
    cmd, _ = guard.filter(FORWARD, (OWN, peer_at(0.5, -0.05)), 10.0)
    forward = cmd.vy * cmd.speed
    assert 0.0 < forward < FORWARD.vy * FORWARD.speed
    assert cmd.wz > 0  # peer slightly right: turn left


def test_a_peer_behind_stale_or_unknown_changes_nothing():
    guard = PeerGuard(PeerConfig())
    assert guard.filter(FORWARD, (OWN, peer_at(-0.3, 0.0)), 10.0)[0] == FORWARD
    stale = peer_at(0.3, 0.0, stamp=9.0)  # older than pose_max_age_s
    assert guard.filter(FORWARD, (OWN, stale), 10.0)[0] == FORWARD
    assert guard.filter(FORWARD, (None, None), 10.0)[0] == FORWARD


def test_blocked_too_long_ferb_yields_turning_in_place():
    cfg = PeerConfig()
    guard = PeerGuard(cfg)
    peer = (OWN, peer_at(0.3, 0.05))
    guard.filter(FORWARD, peer, 10.0)
    now = 10.0 + cfg.deadlock_time_s
    own = Pose2D(0.0, 0.0, 0.0, now)
    cmd, reason = guard.filter(FORWARD, (own, peer_at(0.3, 0.05, now)), now)
    assert cmd.vx == 0.0 and cmd.vy == 0.0 and cmd.wz < 0
    assert math.isclose(cmd.speed, cfg.escape_turn_speed)
    assert "deadlock" in reason


class PoseFleet:
    """
    A fleet link that only reports poses.
    """

    def __init__(self, poses):
        self._poses = poses

    def update(self, state, want, carrying):
        pass

    def granted(self):
        return False

    def delivered(self, color):
        pass

    def poses(self):
        return self._poses


def test_auto_does_not_drive_into_the_peer():
    pilot = AutoPilot(AutoConfig())
    pilot.enabled = True
    now = 10.0
    pilot.fleet = PoseFleet((OWN, peer_at(0.3, 0.05, now)))
    pilot.peer_guard = PeerGuard(PeerConfig())
    pilot.state = "EXPLORE (forward)"
    command = pilot.step(now)
    assert command.vy * command.speed <= 0.0
    assert pilot.target_info.startswith("peer")


def test_auto_and_mobile_base_agree_on_the_ok_reason():
    from teleop_web_lib.autopilot import REASON_PEER_OK
    from teleop_web_lib.peer import REASON_OK

    assert REASON_PEER_OK == REASON_OK


def test_peer_settings_reject_unknown_keys():
    import pytest

    assert not PeerConfig.from_dict(None).enabled
    with pytest.raises(ValueError):
        PeerConfig.from_dict({"stop_distance": 0.4})


def test_without_a_peer_guard_auto_ignores_poses():
    pilot = AutoPilot(AutoConfig())
    pilot.fleet = PoseFleet((OWN, peer_at(0.3, 0.05)))
    assert pilot.peer_guard is None
    pilot.step(10.0)  # no error, poses are not even read
