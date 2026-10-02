"""Phase 0 CV interface: synthetic camera input and prescribed observations.

No hardware capture, image-based detection, or mission decisions occur here.
The simulation calls read() once per tick and owns timing and scheduling.
"""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Callable

import cv2
import numpy as np

if __package__:
    from .synthetic_frames import DEFAULT_SCENARIO, SyntheticSource
    from .universal_logging import log_entry
else:
    from synthetic_frames import DEFAULT_SCENARIO, SyntheticSource
    from universal_logging import log_entry


@dataclass(frozen=True)
class CVObservation:
    """CV's answer for one frame; None detection means input was unavailable.

    Time is on the source's fixture clock, not wall time. Type and distance
    are fixture values, not measurements or classifications from the image.
    """

    detected: bool | None
    bird_type: str | None
    distance_m: float | None
    camera_ok: bool
    timestamp_ns: int
    frame_index: int
    camera_id: str
    source_id: str
    reason: str | None
    clock: str = "fixture"
    synthetic: bool = True
    confidence: float | None = None
    simulation_timestamp_ns: int | None = None

    @property
    def valid(self) -> bool:
        """Whether a camera sample was available, including an empty frame."""
        return self.camera_ok and self.detected is not None

    @property
    def status(self) -> str:
        if not self.valid:
            return "unavailable"
        return "detected" if self.detected else "absent"

    def to_dict(self) -> dict:
        """Return CV-owned fields for JSON logging or snapshot assembly.

        The coordinator supplies battery, safety decisions, docking and state.
        Missing input has bird=null; empty input has bird.detected=false.
        """
        result = {
            "timestamp_ns": self.timestamp_ns,
            "clock": self.clock,
            "frame_index": self.frame_index,
            "camera_id": self.camera_id,
            "source_id": self.source_id,
            "synthetic": self.synthetic,
            "camera_ok": self.camera_ok,
            "valid": self.valid,
            "status": self.status,
            "reason": self.reason,
            "confidence": self.confidence,
            "bird": None if self.detected is None else {
                "detected": self.detected,
                "type": self.bird_type,
                "distance_m": self.distance_m,
            },
        }
        if self.simulation_timestamp_ns is not None:
            result["mapped_time"] = {
                "clock": "simulation",
                "timestamp_ns": self.simulation_timestamp_ns,
            }
        return result


class ComputerVision:
    """Read synthetic frames and log observations through UniversalLog.

    The coordinator supplies mission-state context and an optional mapping from
    fixture nanoseconds to simulation nanoseconds. CV never chooses either.
    The initial logging context is IDLE; update it before each coordinator tick.
    """

    def __init__(
        self,
        *,
        log_sink: Callable[[dict], object] = log_entry,
        timestamp_mapper: Callable[[int], int] | None = None,
        state: str = "IDLE",
    ) -> None:
        self._log_sink = log_sink
        self._timestamp_mapper = timestamp_mapper
        self.set_log_context(state=state)
        self._source: SyntheticSource | None = None
        self._frame_index = 0
        self._last_observation: CVObservation | None = None
        self.frame: np.ndarray | None = None
        self.gray_frame: np.ndarray | None = None

    def set_log_context(self, *, state: str) -> None:
        """Attach the coordinator's mission-state string to subsequent logs."""
        if not isinstance(state, str) or not state:
            raise ValueError("state must be a nonempty mission-state string")
        self._log_state = state

    def _log(self, event: str, **details) -> None:
        # UniversalLog's outer Timestamp is wall-clock emission time. Source
        # and mapped sample times live in Details and are never replaced by it.
        self._log_sink({
            "State": self._log_state,
            "Details": {"module": "CV", "event": event, **details},
        })

    def _source_details(self) -> dict:
        assert self._source is not None
        return {
            "source_id": self._source.scenario["scenario_id"],
            "camera_id": self._source.camera_id,
            "clock": "fixture",
            "synthetic": True,
            "next_frame_index": self._frame_index,
            "last_observation": (
                None if self._last_observation is None
                else self._last_observation.to_dict()
            ),
        }

    def start_camera(
        self,
        scenario_path: str | Path = DEFAULT_SCENARIO,
        camera_id: str = "front",
        image_path: str | Path | None = None,
    ) -> None:
        """Load a synthetic source; the first read starts at fixture time zero.

        Stop before starting another source so resets are explicit to consumers.
        This prepares capture; it does not create a background thread.
        """
        if self._source is not None:
            raise RuntimeError("Camera already started; call stop_camera() first")
        scenario = json.loads(Path(scenario_path).read_text(encoding="utf-8"))
        self._source = SyntheticSource(
            scenario, camera_id, None if image_path is None else Path(image_path)
        )
        self._frame_index = 0
        self._last_observation = None
        self.frame = self.gray_frame = None
        self._log("camera_started", **self._source_details())

    def read(self) -> CVObservation:
        """Consume one scheduled frame and return its stub observation.

        Missing frames return an invalid observation and clear image buffers.
        A finite scenario ends with StopIteration, distinct from camera failure.
        """
        if self._source is None:
            raise RuntimeError("Call start_camera() before read()")
        try:
            frame, data = self._source.sample(self._frame_index)
        except StopIteration:
            self._log("source_exhausted", **self._source_details())
            self.stop_camera()
            raise
        mapped_timestamp = None
        if self._timestamp_mapper is not None:
            mapped_timestamp = self._timestamp_mapper(data["timestamp_ns"])
            if type(mapped_timestamp) is not int or mapped_timestamp < 0:
                raise ValueError("timestamp_mapper must return nonnegative integer nanoseconds")
        self.frame = frame
        self.gray_frame = None if frame is None else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bird = data["bird"]
        observation = CVObservation(
            detected=None if bird is None else bird["detected"],
            bird_type=None if bird is None else bird["type"],
            distance_m=None if bird is None else bird["distance_m"],
            camera_ok=data["camera_ok"], timestamp_ns=data["timestamp_ns"],
            frame_index=data["frame_index"], camera_id=data["camera_id"],
            source_id=data["source_id"], reason=data["reason"],
            clock=data["clock"], synthetic=data["synthetic"],
            simulation_timestamp_ns=mapped_timestamp,
        )
        self._frame_index += 1
        self._last_observation = observation
        self._log("observation", **observation.to_dict())
        return observation

    def stop_camera(self) -> None:
        """Stop receiving input and clear cached images; safe to call repeatedly."""
        details = None if self._source is None else self._source_details()
        self._source = None
        self.frame = self.gray_frame = None
        self._last_observation = None
        if details is not None:
            self._log("camera_stopped", **details)
