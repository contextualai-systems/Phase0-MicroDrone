from dataclasses import dataclass


DOCKING_EVENTS = (
    "alignment_start",
    "alignment_complete",
    "descent_start",
    "landing",
)


@dataclass(frozen=True)
class DockingEvent:
    name: str


class DockingEventLogger:
    def __init__(self):
        self.events = []

    def record(self, event_name: str):
        if event_name not in DOCKING_EVENTS:
            raise ValueError(f"Unknown docking event: {event_name}")

        event = DockingEvent(name=event_name)
        self.events.append(event)
        return event
