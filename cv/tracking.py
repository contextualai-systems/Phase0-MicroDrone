"""Camera/session-local association. Lifecycle is separate from state estimation."""

from collections import Counter
from dataclasses import dataclass, replace

from .shared_perception import TrackState


class LastMeasurementEstimator:
    """Initial estimator seam: observe updates state, advance only retains it.

    A later estimator can maintain velocity/covariance and emit predicted states
    through these same TrackState fields. This implementation never predicts.
    """

    def observe(self, track, detection):
        return replace(track, pose=detection.pose, estimate_kind="measured",
                       estimate_timestamp_ns=detection.timestamp_ns if detection.pose.pose_valid else None,
                       confidence=detection.confidence, velocity_m_s=None, position_covariance_m2=None)

    def advance(self, track, now_ns):
        # now_ns is deliberately NOT substituted for the old estimate timestamp.
        return replace(track, estimate_kind="retained")


@dataclass
class _Entry:
    state: TrackState
    confirmed: bool = False
    streak: int = 1


class MarkerTracker:
    """Owned by one PerceptionPublisher; timestamps validated at that boundary."""

    def __init__(self, session_id, *, confirmation_hits=2, max_gap_ns=500_000_000):
        if type(confirmation_hits) is not int or confirmation_hits < 1:
            raise ValueError("confirmation_hits must be a positive integer")
        if type(max_gap_ns) is not int or max_gap_ns <= 0:
            raise ValueError("max_gap_ns must be a positive integer")
        self.session_id = session_id
        self.confirmation_hits, self.max_gap_ns = confirmation_hits, max_gap_ns
        self._entries = {}
        self._counter = 0
        self._estimator = LastMeasurementEstimator()

    def update(self, detections, now_ns, *, sample_received, missing_reason):
        expired = []
        for key, entry in list(self._entries.items()):
            if now_ns - entry.state.last_observed_timestamp_ns >= self.max_gap_ns:
                expired.append(replace(self._estimator.advance(entry.state, now_ns),
                                       status="expired", valid=False, reason="track_expired"))
                del self._entries[key]
        counts = Counter((d.source_camera, d.object_id) for d in detections)
        seen, associated = set(), []
        for detection in detections:
            key = detection.source_camera, detection.object_id
            reason = None
            if detection.object_type != "aruco_marker":
                reason = "unsupported_tracking_type"
            elif counts[key] > 1:
                reason = "ambiguous_object_id"
            elif now_ns - detection.timestamp_ns >= self.max_gap_ns:
                reason = "stale_observation"
            if reason:
                associated.append(replace(detection, association_reason=reason))
                continue
            seen.add(key)
            entry = self._entries.get(key)
            if entry is None:
                self._counter += 1
                state = TrackState(f"{self.session_id}:{self._counter}", detection.object_id,
                                   detection.object_type, detection.source_id, detection.source_camera,
                                   "tentative", "measured", detection.timestamp_ns, None, detection.pose)
                entry = _Entry(state)
                self._entries[key] = entry
            else:
                entry.streak += 1
                entry.state = replace(entry.state, hits=entry.state.hits + 1)
            entry.confirmed = entry.confirmed or entry.streak >= self.confirmation_hits
            entry.state = self._estimator.observe(entry.state, detection)
            entry.state = replace(entry.state, last_observed_timestamp_ns=detection.timestamp_ns,
                                  status="confirmed" if entry.confirmed else "tentative",
                                  valid=True, reason=None)
            associated.append(replace(detection, tracking_id=entry.state.tracking_id))
        for key, entry in self._entries.items():
            if key in seen:
                continue
            # A heartbeat without a new frame doesn't break consecutive-sample
            # confirmation. A genuinely empty, missing, or ambiguous sample does.
            if sample_received:
                entry.streak = 0
            reason = "ambiguous_object_id" if counts[key] > 1 else missing_reason
            entry.state = replace(self._estimator.advance(entry.state, now_ns),
                                  status="temporarily_lost" if sample_received else entry.state.status,
                                  reason=reason)
        return tuple(associated), tuple(e.state for e in self._entries.values()) + tuple(expired)
