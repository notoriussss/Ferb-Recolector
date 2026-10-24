"""
Shared state vocabulary of the collector. Pure logic, no rclpy.

`State` is the robot's own behaviour state (what `behavior_fsm` publishes).
`FleetState` is the vocabulary shared with the other robots on the track
(what `fleet_link` publishes).
"""

from enum import StrEnum


class State(StrEnum):
    """
    Local behaviour states.

    SEARCH, APPROACH and ARRIVED are driven by `ApproachController`.
    MANUAL and NO_CAMERA are overlays decided by the `behavior_fsm` node.
    CATCH, CARRY and RELEASE are reserved for the scoop work (role 3).
    """

    PAUSED = "PAUSED"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ARRIVED = "ARRIVED"
    CATCH = "CATCH"
    CARRY = "CARRY"
    RELEASE = "RELEASE"
    MANUAL = "MANUAL"
    NO_CAMERA = "NO_CAMERA"


class FleetState(StrEnum):
    """
    Multi-robot state vocabulary (see multirobot_interfaces/RobotState).
    """

    EXPLORE = "EXPLORE"
    APPROACH = "APPROACH"
    CAPTURE = "CAPTURE"
    CARRY = "CARRY"
    DELIVER = "DELIVER"
    STOP = "STOP"
