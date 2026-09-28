"""Phase 0 CV interface: synthetic camera input and prescribed observations.

No hardware capture, image-based detection, or mission decisions occur here.
The simulation calls read() once per tick and owns timing and scheduling.
"""

from dataclasses import dataclass
import json
import logging
from pathlib import Path

import cv2
import numpy as np

if __package__:
    from .synthetic_frames import DEFAULT_SCENARIO, SyntheticSource
else:
    from synthetic_frames import DEFAULT_SCENARIO, SyntheticSource

logger = logging.getLogger(__name__)


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

    def to_dict(self) -> dict:
        """Return CV-owned fields for JSON logging or snapshot assembly.

        The coordinator supplies battery, safety decisions, docking and state.
        Missing input has bird=null; empty input has bird.detected=false.
        """
        return {
            "timestamp_ns": self.timestamp_ns,
            "clock": self.clock,
            "frame_index": self.frame_index,
            "camera_id": self.camera_id,
            "source_id": self.source_id,
            "synthetic": self.synthetic,
            "camera_ok": self.camera_ok,
            "valid": self.camera_ok,
            "reason": self.reason,
            "confidence": self.confidence,
            "bird": None if self.detected is None else {
                "detected": self.detected,
                "type": self.bird_type,
                "distance_m": self.distance_m,
            },
        }


class ComputerVision:
    """Read the synthetic camera without opening windows or writing files."""

    def __init__(self) -> None:
        self._source: SyntheticSource | None = None
        self._frame_index = 0
        self.frame: np.ndarray | None = None
        self.gray_frame: np.ndarray | None = None

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
        self.frame = self.gray_frame = None
        # TODO(shared logger): record camera_started with source_id and camera_id.

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
            self.stop_camera()
            logger.info("source_exhausted")
            raise
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
        )
        self._frame_index += 1
        # TODO(shared logger): record observation.to_dict(), including missing input.
        # Preserve its fixture timestamp; do not replace it with the logging time.
        return observation

    def stop_camera(self) -> None:
        """Stop receiving input and clear cached images; safe to call repeatedly."""
        self._source = None
        self.frame = self.gray_frame = None
        # TODO(shared logger): record camera_stopped.
