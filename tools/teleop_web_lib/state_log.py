"""
One log line each time the Auto state changes, with what the fleet was told
(want) and answered (granted): without it a robot standing still at the area
could not be told apart from one waiting for its turn.
"""

from typing import NamedTuple

# Shown for `granted` when there is no fleet link (Auto takes no turns).
NO_FLEET = "-"


class AutoSnapshot(NamedTuple):
    """
    What a state-change line reports; the target alone changes every frame
    and is not part of it.
    """

    auto: bool
    state: str
    carrying: bool
    want: bool
    granted: bool | None
    collected: int
    delivered: int


def auto_snapshot(pilot) -> AutoSnapshot:
    """
    The snapshot of an AutoPilot (call it under the drive lock).
    """
    fleet = pilot.fleet
    return AutoSnapshot(
        auto=pilot.enabled,
        state=pilot.state,
        carrying=pilot.carrying,
        want=pilot.fleet_want,
        granted=None if fleet is None else fleet.granted(),
        collected=pilot.collected_count,
        delivered=pilot.delivered_count,
    )


class StateLog:
    """
    Remembers the last snapshot logged and formats the next different one.
    """

    def __init__(self):
        """
        Start with nothing logged, so the first snapshot makes a line.
        """
        self._last: AutoSnapshot | None = None

    def line(self, snap: AutoSnapshot, target: str | None) -> str | None:
        """
        The log line for `snap`, or None when it did not change.
        """
        if snap == self._last:
            return None
        self._last = snap
        granted = NO_FLEET if snap.granted is None else snap.granted
        return (
            f"auto={snap.auto} state={snap.state} carrying={snap.carrying} "
            f"want={snap.want} granted={granted} "
            f"collected={snap.collected} delivered={snap.delivered} "
            f"target={target}"
        )
