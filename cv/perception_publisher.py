"""Turn measured CV observations into the shared detection + track contract."""

from dataclasses import replace
import math
from time import perf_counter_ns
from uuid import uuid4

import numpy as np

from .computer_vision import CVObservation
from .shared_perception import CameraToENU, PerceptionUpdate, SharedDetectionResult, valid_ns, world_pose
from .tracking import MarkerTracker
from .universal_logging import log_entry


class PerceptionPublisher:
    """Explicit scheduling boundary: update(observation or None, now_ns).

    One instance per camera/source stream. None advances track lifecycle without
    inventing an image. reset() starts a new session after a source/clock restart.
    Exceptions from publication/logging propagate; accepted updates aren't rolled
    back, so callers must not retry the same image as a new measurement.
    """

    def __init__(self, *, session_id=None, confirmation_hits=2, max_gap_ns=500_000_000,
                 publisher=None, log_sink=log_entry, state="IDLE"):
        self._base_session = uuid4().hex if session_id is None else session_id
        if not isinstance(self._base_session, str) or not self._base_session:
            raise ValueError("session_id must be a nonempty string")
        self._generation = 0
        self._confirmation_hits, self._max_gap_ns = confirmation_hits, max_gap_ns
        self._publisher, self._log_sink = publisher, log_sink
        self.set_log_context(state=state)
        self._initialize()
        self._log_sink({"State": self._state, "Details": {
            "module": "CV", "event": "tracking_started", "session_id": self.session_id,
            "confirmation_hits": confirmation_hits, "max_gap_ns": max_gap_ns,
            "estimator": "last_measurement", "association": "camera_local_aruco_identity"}})

    def _initialize(self):
        self.session_id = f"{self._base_session}/{self._generation}"
        self._tracker = MarkerTracker(self.session_id, confirmation_hits=self._confirmation_hits,
                                      max_gap_ns=self._max_gap_ns)
        self._last = None
        self._now = None
        self._origin = None

    def set_log_context(self, *, state):
        if not isinstance(state, str) or not state:
            raise ValueError("state must be a nonempty mission-state string")
        self._state = state

    def reset(self):
        old_session = self.session_id
        self._generation += 1
        self._initialize()
        self._log_sink({"State": self._state, "Details": {
            "module": "CV", "event": "tracking_reset", "previous_session_id": old_session,
            "session_id": self.session_id}})

    def _validate(self, observation, now_ns, transform):
        valid_ns(now_ns)
        if self._now is not None and now_ns < self._now:
            raise ValueError("Update time moved backwards; reset the stream explicitly")
        if observation is None:
            if self._last is None:
                raise ValueError("A first sample is required before heartbeat updates")
            if transform is not None:
                raise ValueError("Heartbeat updates cannot re-transform old measurements")
            return
        if not isinstance(observation, CVObservation) or observation.processing_status not in ("ok", "error", "unavailable"):
            raise ValueError("Expected a generic perception CVObservation, not a bird fixture")
        valid_ns(observation.timestamp_ns)
        valid_ns(observation.frame_index)
        valid_ns(observation.simulation_timestamp_ns)
        if observation.simulation_timestamp_ns > now_ns:
            raise ValueError("Capture time cannot be later than update time")
        if not all(isinstance(v, str) and v for v in (observation.source_id, observation.camera_id, observation.clock)):
            raise ValueError("Sample requires source, camera, and clock identity")
        if type(observation.camera_ok) is not bool or type(observation.synthetic) is not bool:
            raise ValueError("Camera and synthetic flags must be booleans")
        if observation.processing_status == "ok":
            if not observation.camera_ok or observation.detections is None or observation.reason is not None:
                raise ValueError("Successful processing requires an image and detection collection")
        elif observation.detections is not None or not observation.reason:
            raise ValueError("Failed processing requires a reason and no detection collection")
        if self._last is not None:
            old = self._last
            if (observation.source_id, observation.camera_id, observation.clock, observation.synthetic) != (
                    old.source_id, old.camera_id, old.clock, old.synthetic):
                raise ValueError("Source identity changed; reset the stream explicitly")
            if (observation.frame_index <= old.frame_index or observation.timestamp_ns <= old.timestamp_ns
                    or observation.simulation_timestamp_ns <= old.simulation_timestamp_ns):
                raise ValueError("Repeated/out-of-order frame; pass None for heartbeats or reset for a restart")
        for detection in observation.detections or ():
            if not all(isinstance(v, str) and v for v in (detection.object_id, detection.type)):
                raise ValueError("Detection requires object identity and type")
            corners = np.asarray(detection.corners_px)
            if corners.shape != (4, 2) or not np.isfinite(corners).all():
                raise ValueError("Expected four finite image corners")
            score = detection.confidence
            if score is not None and (type(score) not in (int, float) or not math.isfinite(score) or not 0 <= score <= 1):
                raise ValueError("Confidence must be null or finite in [0,1]")
            if type(detection.pose_valid) is not bool:
                raise ValueError("pose_valid must be a boolean")
        if transform is not None:
            if not isinstance(transform, CameraToENU):
                raise ValueError("Expected CameraToENU")
            if self._origin is not None and transform.origin_id != self._origin:
                raise ValueError("ENU origin changed; reset the stream explicitly")

    def update(self, observation=None, *, now_ns, transform=None):
        start = perf_counter_ns()
        self._validate(observation, now_ns, transform)
        received = observation is not None
        sample = observation if received else self._last
        status = sample.processing_status if received else "no_new_frame"
        reason = sample.reason if received else "no_new_frame"
        detections = []
        if received:
            for detection in sample.detections or ():
                detections.append(SharedDetectionResult(
                    timestamp_ns=sample.simulation_timestamp_ns, source_timestamp_ns=sample.timestamp_ns,
                    source_clock=sample.clock, source_id=sample.source_id, source_camera=sample.camera_id,
                    object_id=detection.object_id, object_type=detection.type, confidence=detection.confidence,
                    confidence_method=detection.confidence_method,
                    corners_px=tuple(tuple(float(v) for v in point) for point in detection.corners_px),
                    pose=world_pose(detection, sample, transform)))
        transform_ns = perf_counter_ns() - start
        tracking_start = perf_counter_ns()
        detections, tracks = self._tracker.update(detections, now_ns, sample_received=received,
                                                  missing_reason=reason or "not_observed")
        timings = (("adapter", transform_ns), ("tracking", perf_counter_ns() - tracking_start))
        update = PerceptionUpdate(
            session_id=self.session_id, source_id=sample.source_id, source_camera=sample.camera_id,
            update_timestamp_ns=now_ns, capture_timestamp_ns=sample.simulation_timestamp_ns,
            source_timestamp_ns=sample.timestamp_ns, source_clock=sample.clock,
            frame_index=sample.frame_index, synthetic=sample.synthetic, sample_received=received,
            frame_valid=sample.camera_ok if received else None,
            valid=sample.valid if received else False, reason=reason, processing_status=status,
            detections=detections, tracks=tracks, timings_ns=timings)
        self._now = now_ns
        if received:
            self._last = observation
            if any(d.pose.pose_valid for d in detections):
                self._origin = transform.origin_id
        publish_start = perf_counter_ns()
        try:
            if self._publisher is not None:
                self._publisher(update)
        except Exception as exc:
            self._log_sink({"State": self._state, "Details": {
                "module": "CV", "event": "perception_update_publication_error",
                "error_type": type(exc).__name__, **update.to_dict()}})
            raise
        update = replace(update, timings_ns=timings + (("publication", perf_counter_ns()-publish_start),))
        self._log_sink({"State": self._state, "Details": {
            "module": "CV", "event": "perception_update", **update.to_dict()}})
        return update
