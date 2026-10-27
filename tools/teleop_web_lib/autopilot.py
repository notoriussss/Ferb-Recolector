"""
Auto mode state machine for the browser tool. Pure logic: no hardware, no
threads. Time is passed in as a float in seconds.

Auto mode: find a target-colour cube, push it into the scoop, carry it to the
green/blue delivery line, leave it there and count it. The bottom line sensor
stops the robot at the black border (it backs up and turns inward).

Axes: on the physical robot, `DriveCommand` maps to:
- `vy > 0` = forward (W), `vy < 0` = backward/reverse (S)
- `vx < 0` = strafe left (A), `vx > 0` = strafe right (D)
- `wz > 0` = spin counter-clockwise (Q), `wz < 0` = spin clockwise (E)
`_forward`, `_backward`, `_strafe` and `_turn` hide that so the state machine
can think in plain directions.
"""

import time
from typing import NamedTuple, Protocol

from teleop_web_lib.config import AutoConfig
from teleop_web_lib.kinematics import clamp

# --- Machine states -----------------------------------------------------------
STATE_PAUSED = "PAUSED"
STATE_MANUAL = "MANUAL"
STATE_SEARCH = "SEARCH"
STATE_APPROACH = "APPROACH"
STATE_ALIGN = "ALIGN"
STATE_CATCH = "CATCH"
STATE_CARRY = "CARRY"
STATE_DROP = "DROP"
STATE_EXPLORE = "EXPLORE"
STATE_LINE_STOP = "LINE (stop)"
STATE_LINE_BACKUP = "LINE (back up)"
STATE_LINE_TURN = "LINE (turn)"
STATE_DROP_BACKUP = "DROP (back up)"
STATE_DROP_TURN = "DROP (turn)"
# Prefixes used to recognise a family of states.
PREFIX_LINE = "LINE"
PREFIX_DROP = "DROP"
PREFIX_DONE = "DONE"
PREFIX_NO_CUBES = "NO CUBES"
STATE_NO_CUBES = f"{PREFIX_NO_CUBES} (stopped)"
# Suffixes appended to a base state while it has sub-phases.
SUFFIX_FORWARD = " (forward)"
SUFFIX_TURN = " (turn)"
SUFFIX_ALIGN = " (align)"

# --- Direction helpers --------------------------------------------------------
# Direction of a turn: +1 toward the left, -1 toward the right.
DIR_LEFT = 1.0
DIR_RIGHT = -1.0
FULL_AXIS = 1.0

# --- Target descriptions shown in the page ------------------------------------
TARGET_LOOKING_FOR_ZONE = "looking for zone"
CM_PER_M = 100


class DriveCommand(NamedTuple):
    """
    What the drive thread applies: three axes in [-1, 1] and a speed in
    [0, 1] (see the module docstring for the axis meaning).
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    speed: float = 0.0


STOP = DriveCommand()


class Vision(Protocol):
    """
    What the autopilot needs from the camera pipeline.
    """

    def latest_detection(self, now: float, timeout_s: float):
        """
        The best cube seen less than `timeout_s` ago, or None.
        """

    def latest_zone(self, now: float, timeout_s: float):
        """
        The delivery line seen less than `timeout_s` ago, or None.
        """


def _forward(speed: float) -> DriveCommand:
    """
    Drive straight ahead.
    """
    return DriveCommand(0.0, FULL_AXIS, 0.0, speed)


def _backward(speed: float) -> DriveCommand:
    """
    Drive straight back.
    """
    return DriveCommand(0.0, -FULL_AXIS, 0.0, speed)


def _turn(direction: float, speed: float) -> DriveCommand:
    """
    Spin in place; `direction` is DIR_LEFT (+1, Q/CCW) or DIR_RIGHT (-1, E/CW).
    """
    return DriveCommand(0.0, 0.0, direction, speed)


def _strafe(direction: float, speed: float) -> DriveCommand:
    """
    Move sideways; `direction` is DIR_LEFT (+1, A) or DIR_RIGHT (-1, D).
    """
    return DriveCommand(-direction, 0.0, 0.0, speed)


def _side_of(bearing_rad: float) -> float:
    """
    Turn direction toward something at `bearing_rad` (positive = left).
    """
    return DIR_LEFT if bearing_rad >= 0 else DIR_RIGHT


class AutoPilot:
    """
    Decides the next `DriveCommand` while Auto mode is on.

    Not thread-safe by itself: the caller (Drive) serialises access.
    """

    def __init__(self, cfg: AutoConfig, vision: Vision | None = None):
        """
        Start paused with `cfg`; `vision` supplies the detections.
        """
        self.cfg = cfg
        self.vision = vision
        self.enabled = cfg.enabled
        self.state = STATE_PAUSED
        self.target_info: str | None = None
        self.collected_count = 0  # cubes pushed into the scoop
        self.delivered_count = 0  # cubes left at the green/blue line
        self.carrying = False
        self._phase_until = 0.0
        self._line_turn_until = 0.0
        self._line_stop_until = 0.0
        self._line_black = False
        self._escape = STOP
        self._last_motion: DriveCommand | None = None
        self._last_motion_s = 0.0
        self._resume_state = STATE_SEARCH
        self._last_seen_s: float | None = None
        self._search_start_s = time.monotonic()
        self._search_dir = DIR_LEFT

    # ---------- mode control ----------
    def configure(
        self,
        now: float,
        enabled: bool | None = None,
        reset: bool = False,
    ) -> None:
        """
        Switch Auto on or off and/or reset the counters.
        """
        if enabled is not None:
            self.enabled = bool(enabled)
        restartable = self.state.startswith(
            (PREFIX_DONE, PREFIX_NO_CUBES, STATE_PAUSED)
        )
        if (reset or self.enabled) and (restartable or reset):
            self.state = (
                STATE_CARRY if self.carrying and not reset else STATE_SEARCH
            )
            self._search_start_s = now
            if reset:
                self.collected_count = 0
                self.delivered_count = 0
                self.carrying = False
        if not self.enabled:
            self.state = STATE_PAUSED

    def mark_manual(self) -> None:
        """
        The operator is driving by hand.
        """
        self.state = STATE_MANUAL

    def mark_paused(self) -> None:
        """
        Auto is off and nobody is driving.
        """
        self.state = STATE_PAUSED

    # ---------- main step ----------
    def step(self, now: float, line_black: bool = False) -> DriveCommand:
        """
        The command for time `now`; `line_black` is the line sensor reading.
        """
        if self.state.startswith((PREFIX_DONE, PREFIX_NO_CUBES)):
            return STOP
        if self.enabled and self.state in (STATE_MANUAL, STATE_PAUSED):
            # Back from WASD or pause: if it had a cube, keep carrying it.
            self._resume_after_pause(now)
        self._line_black = line_black
        if line_black and not self.state.startswith(PREFIX_LINE):
            self._start_line_recovery(now)
        for phase in (self._line_phase, self._catch_phase, self._drop_phase):
            command = phase(now)
            if command is not None:
                return command
        if not self.enabled:
            self.state = STATE_PAUSED
            return STOP
        if self.state.startswith(STATE_CARRY):
            return self._carry_step(now)
        return self._search_step(now)

    def _resume_after_pause(self, now: float) -> None:
        """
        Continue carrying if a cube is in the scoop, else search.
        """
        self.state = STATE_CARRY if self.carrying else STATE_SEARCH
        self._search_start_s = now

    # ---------- timed phases (return None when not active) ----------
    def _start_line_recovery(self, now: float) -> None:
        """
        Black line underneath: brake, move away from the line (opposite to
        the way it was going) until the sensor sees white again plus a
        margin, turn inward (unless it backed into the line and escaped
        forward, already facing the arena), then resume what it was doing.
        """
        cfg = self.cfg
        if not self.enabled:
            self._resume_state = STATE_PAUSED
        elif self.carrying:
            self._resume_state = STATE_CARRY
        else:
            self._resume_state = STATE_SEARCH
        self._escape = self._escape_command(now)
        self.state = STATE_LINE_STOP
        self._line_stop_until = now + cfg.line_stop_s
        self._phase_until = self._line_stop_until + cfg.line_backup_s
        # Spin inward only when we hit the line going forward in Auto mode.
        needs_turn = (
            self.enabled and self._escape.vy < 0 and self._escape.vx == 0
        )
        turn_s = cfg.line_turn_s if needs_turn else 0.0
        self._line_turn_until = self._phase_until + turn_s
        self.target_info = None

    def note_motion(self, now: float, command: DriveCommand) -> None:
        """
        Remember the translation the robot is doing (manual or automatic:
        forward/back on `vy` or lateral strafe on `vx`), so the line recovery
        can undo it. Ignored while recovering, so the escape itself does not
        overwrite the direction it escapes from.
        """
        if self.state.startswith(PREFIX_LINE) or command.speed <= 0.0:
            return
        if command.vx != 0.0 or command.vy != 0.0:
            self._last_motion = command
            self._last_motion_s = now

    def _escape_command(self, now: float) -> DriveCommand:
        """
        Straight move away from the line: opposite to the main axis of the
        last translation (forward W -> back S, back S -> forward W, lateral
        left A -> right D, lateral right D -> left A). Backs up when there
        is no recent motion to undo.
        """
        cfg = self.cfg
        last = self._last_motion
        if last is None or now - self._last_motion_s > cfg.line_motion_memory_s:
            return _backward(cfg.line_speed)
        if abs(last.vx) > abs(last.vy):
            direction = DIR_RIGHT if last.vx < 0 else DIR_LEFT
            return _strafe(direction, cfg.line_speed)
        if last.vy < 0:
            return _forward(cfg.line_speed)
        return _backward(cfg.line_speed)

    def _line_phase(self, now: float) -> DriveCommand | None:
        """
        Stop, move away from the line, then turn, then hand control back to
        the resumed state.
        """
        if not self.state.startswith(PREFIX_LINE):
            return None
        cfg = self.cfg
        if now < self._line_stop_until:
            self.state = STATE_LINE_STOP
            return STOP
        if now < self._phase_until:
            self.state = STATE_LINE_BACKUP
            # Still on black: keep moving away (up to a limit), so it always
            # leaves the border before turning.
            backing_s = now - self._line_stop_until
            if self._line_black and backing_s < cfg.line_backup_max_s:
                needs_turn = (
                    self.enabled
                    and self._escape.vy < 0
                    and self._escape.vx == 0
                )
                turn_s = cfg.line_turn_s if needs_turn else 0.0
                self._phase_until = now + cfg.line_backup_s
                self._line_turn_until = self._phase_until + turn_s
            return self._escape
        if now < self._line_turn_until:
            self.state = STATE_LINE_TURN
            return _turn(self._search_dir, cfg.turn_speed)
        self._last_motion = None
        self.state = self._resume_state
        self._search_start_s = now
        return None

    def _catch_phase(self, now: float) -> DriveCommand | None:
        """
        Blind push through the last ~16.5 cm, where the scoop hides the cube
        from the camera; then count the cube as picked.
        """
        if self.state != STATE_CATCH:
            return None
        if now < self._phase_until:
            return _forward(self.cfg.forward_speed)
        self.collected_count += 1
        self.carrying = True
        self.state = STATE_CARRY
        self._search_start_s = now
        return STOP

    def _drop_phase(self, now: float) -> DriveCommand | None:
        """
        Delivery: push the cube onto the line, back up leaving it there and
        turn around.
        """
        if not self.state.startswith(PREFIX_DROP):
            return None
        cfg = self.cfg
        if now < self._phase_until:
            return _forward(cfg.forward_speed)
        if self.carrying:
            self.carrying = False
            self.delivered_count += 1
        backup_until = self._phase_until + cfg.deliver_backup_s
        if now < backup_until:
            self.state = STATE_DROP_BACKUP
            return _backward(cfg.line_speed)
        if now < backup_until + cfg.deliver_turn_s:
            self.state = STATE_DROP_TURN
            return _turn(self._search_dir, cfg.turn_speed)
        self.state = STATE_SEARCH
        self._search_start_s = now
        return None

    # ---------- looking for cubes ----------
    def _search_step(self, now: float) -> DriveCommand:
        """
        Look for a cube: approach it, catch it, or roam when none is seen.
        """
        cfg = self.cfg
        det = None
        if self.vision is not None:
            det = self.vision.latest_detection(now, cfg.lost_timeout_s)
        if det is None:
            return self._no_cube_step(now)

        self.target_info = f"{det.color} {det.distance_m * CM_PER_M:.0f}cm"
        self._last_seen_s = now
        self._search_start_s = now
        self._search_dir = _side_of(det.bearing_rad)

        # Cube at the scoop threshold (~18.5 cm or image bottom) and aligned.
        at_scoop = (
            det.distance_m <= cfg.stop_distance_m
            or det.v >= cfg.scoop_arrive_v_px
        )
        if at_scoop and self._is_aligned(det.bearing_rad):
            self.state = STATE_CATCH
            self._phase_until = now + cfg.scoop_push_s
            return _forward(cfg.forward_speed)
        return self._steer_to(
            det.bearing_rad,
            det.distance_m,
            cfg.stop_distance_m,
            STATE_APPROACH,
        )

    def _no_cube_step(self, now: float) -> DriveCommand:
        """
        No cube in view: wait briefly, give up after a while, else roam.
        """
        cfg = self.cfg
        self.target_info = None
        # If it lost sight of a cube a moment ago, wait briefly before turning.
        recently_seen = (
            self._last_seen_s is not None
            and (now - self._last_seen_s) < cfg.lost_timeout_s
        )
        if recently_seen:
            return STOP
        # After searching too long without seeing any cube, stop.
        if (now - self._search_start_s) >= cfg.no_cubes_stop_s:
            if self.delivered_count > 0:
                self.state = f"{PREFIX_DONE} ({self.delivered_count} delivered)"
            else:
                self.state = STATE_NO_CUBES
            return STOP
        return self._roam(now, STATE_EXPLORE)

    # ---------- carrying to the delivery line ----------
    def _carry_step(self, now: float) -> DriveCommand:
        """
        With a cube in the scoop: find the green/blue line and drive to it.
        """
        cfg = self.cfg
        zone = None
        if self.vision is not None:
            zone = self.vision.latest_zone(now, cfg.lost_timeout_s)
        if zone is None:
            self.target_info = TARGET_LOOKING_FOR_ZONE
            return self._roam(now, STATE_CARRY)

        self.target_info = (
            f"zone {zone.color} {zone.distance_m * CM_PER_M:.0f}cm"
        )
        self._search_start_s = now
        self._search_dir = _side_of(zone.bearing_rad)
        at_zone = (
            zone.distance_m <= cfg.zone_stop_distance_m
            or zone.v >= cfg.zone_arrive_v_px
        )
        if at_zone and self._is_aligned(zone.bearing_rad):
            self.state = STATE_DROP
            self._phase_until = now + cfg.deliver_push_s
            return _forward(cfg.forward_speed)
        return self._steer_to(
            zone.bearing_rad,
            zone.distance_m,
            cfg.zone_stop_distance_m,
            STATE_CARRY,
        )

    # ---------- motion primitives ----------
    def _is_aligned(self, bearing_rad: float) -> bool:
        """
        True when the target is straight enough ahead to push into it.
        """
        cfg = self.cfg
        limit_rad = cfg.align_tolerance_rad * cfg.catch_align_factor
        return abs(bearing_rad) <= limit_rad

    def _roam(self, now: float, label: str) -> DriveCommand:
        """
        Free roaming: alternate driving forward and turning to scan.
        """
        cfg = self.cfg
        elapsed_s = (now - self._search_start_s) % cfg.roam_period_s
        if elapsed_s < cfg.roam_forward_s:
            self.state = f"{label}{SUFFIX_FORWARD}"
            return _forward(cfg.roam_forward_speed)
        self.state = f"{label}{SUFFIX_TURN}"
        return _turn(self._search_dir, cfg.search_turn_speed)

    def _steer_to(
        self,
        bearing: float,
        distance: float,
        stop_distance: float,
        label: str,
    ) -> DriveCommand:
        """
        Turn in place if off-heading; otherwise drive straight forward
        (star motion: never mix rotation and translation on Mecanum wheels,
        which causes sideways drift).
        """
        cfg = self.cfg
        if abs(bearing) > cfg.align_tolerance_rad:
            if label == STATE_APPROACH:
                self.state = STATE_ALIGN
            else:
                self.state = f"{label}{SUFFIX_ALIGN}"
            return _turn(_side_of(bearing), cfg.turn_speed)
        self.state = label
        span_m = max(
            cfg.min_slow_span_m,
            cfg.slow_down_distance_m - stop_distance,
        )
        fraction = clamp(
            (distance - stop_distance) / span_m,
            cfg.min_speed_fraction,
            FULL_AXIS,
        )
        return _forward(cfg.forward_speed * fraction)
