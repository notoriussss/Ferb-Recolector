"""
Guard of the Auto mode against the other collector (Phineas), from the
overhead poses Perry publishes. The rule is mobile_base's PeerAvoidance, the
one agreed with Phineas (docs/external-interface.md): this module only turns
an Auto DriveCommand into its (forward, left, ccw) velocity and back.

The front camera only sees cubes, the area and the border: without this, the
two robots drove into each other more than once (11.2026).
"""

import logging
import sys

from teleop_web_lib.autopilot import STOP, DriveCommand
from teleop_web_lib.config import MOBILE_BASE_PACKAGE_DIR, PeerConfig

# mobile_base is a ROS package, but peer_avoidance is pure logic: no ROS 2
# needed, the laptop runs it too.
sys.path.insert(0, str(MOBILE_BASE_PACKAGE_DIR))
from mobile_base.peer_avoidance import (  # noqa: E402
    REASON_OK,
    PeerAvoidance,
    PeerAvoidanceParams,
    Pose2D,
)

__all__ = ["REASON_OK", "PeerGuard", "Pose2D", "from_velocity", "to_velocity"]

log = logging.getLogger("teleop_web")


def to_velocity(cmd: DriveCommand) -> tuple[float, float, float]:
    """
    The (forward, left, ccw) velocity of an Auto command (axes times speed).
    """
    return (cmd.vy * cmd.speed, -cmd.vx * cmd.speed, cmd.wz * cmd.speed)


def from_velocity(velocity: tuple[float, float, float]) -> DriveCommand:
    """
    The Auto command for a (forward, left, ccw) velocity: the largest
    component becomes the speed and the axes stay in [-1, 1].
    """
    forward, left, ccw = velocity
    speed = max(abs(forward), abs(left), abs(ccw))
    if speed == 0.0:
        return STOP
    return DriveCommand(-left / speed, forward / speed, ccw / speed, speed)


class PeerGuard:
    """
    Filters Auto commands so the robot slows, stops and turns away from the
    other collector ahead, and yields in a deadlock.
    """

    def __init__(self, cfg: PeerConfig):
        """
        Build the shared avoidance rule with this tool's settings.
        """
        self._avoidance = PeerAvoidance(
            PeerAvoidanceParams(
                pose_max_age_s=cfg.pose_max_age_s,
                front_cone_deg=cfg.front_cone_deg,
                peer_slow_distance_m=cfg.slow_distance_m,
                peer_stop_distance_m=cfg.stop_distance_m,
                avoid_turn_rad_s=cfg.avoid_turn_speed,
                deadlock_time_s=cfg.deadlock_time_s,
                escape_turn_rad_s=cfg.escape_turn_speed,
            )
        )
        self._last_reason = REASON_OK

    def filter(self, cmd: DriveCommand, poses, now: float):
        """
        The command to apply and the reason, given (own, peer) poses stamped
        with the monotonic time they arrived (None: not received yet).
        """
        own, peer = poses
        velocity, reason = self._avoidance.filter(
            to_velocity(cmd), own, peer, now
        )
        if reason != self._last_reason:
            log.info("peer guard: %s", reason)
            self._last_reason = reason
        if reason == REASON_OK:
            return cmd, reason
        return from_velocity(velocity), reason
