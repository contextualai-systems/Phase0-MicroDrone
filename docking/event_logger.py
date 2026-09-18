"""Docking event logging for the Phase-0 MicroDrone.

This module records the ordered sequence of high-level docking events
(alignment, descent, and landing) emitted while the drone docks with its
roof-mounted docking station. The accepted event list is small and strictly
validated so the docking history stays meaningful for later modules and
Phase-1 autonomy (e.g. replaying a docking run or diagnosing an aborted
approach).
"""

from dataclasses import dataclass


# Sequence of docking events the drone passes through, in expected order.
DOCKING_EVENTS = (
    "alignment_start",
    "alignment_complete",
    "descent_start",
    "landing",
)


# TODO (Phase-1): Add timestamp to DockingEvent for chronological docking analytics.
@dataclass(frozen=True)
class DockingEvent:
    """A single, immutable docking event recorded by `DockingEventLogger`."""

    name: str


class DockingEventLogger:
    """Logs validated docking events in the order they occur.

    Only the event names listed in `DOCKING_EVENTS` are accepted, so the
    event history remains meaningful across runs. Recorded events are
    stored in `self.events` as immutable `DockingEvent` instances and can
    be replayed or inspected for docking analytics.
    """

    def __init__(self) -> None:
        self.events: list[DockingEvent] = []

    def record(self, event_name: str) -> DockingEvent:
        """Validate and record a docking event.

        Args:
            event_name: Name of the docking event to record. Must be one of
                the values in `DOCKING_EVENTS`.

        Returns:
            DockingEvent: The event that was created and appended to
            `self.events`.

        Raises:
            ValueError: If `event_name` is not a known docking event.
        """
        if event_name not in DOCKING_EVENTS:
            raise ValueError(f"Unknown docking event: {event_name}")

        event = DockingEvent(name=event_name)
        self.events.append(event)
        return event
