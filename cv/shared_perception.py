"""Versioned coordinator values. No mission enums, commands, or clock ownership."""

from dataclasses import asdict, dataclass
import math

import cv2
import numpy as np


def valid_ns(value):
    if type(value) is not int or value < 0:
        raise ValueError("Expected nonnegative integer simulation nanoseconds")


def quaternion_matrix(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1., atol=1e-6):
        raise ValueError("Expected finite unit quaternion xyzw")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(matrix):
    rvec, _ = cv2.Rodrigues(matrix)
    angle = float(np.linalg.norm(rvec))
    xyz = np.zeros(3) if angle < 1e-12 else rvec.reshape(3) * (math.sin(angle/2)/angle)
    return tuple(float(v) for v in xyz) + (math.cos(angle/2),)


@dataclass(frozen=True)
class CameraToENU:
    """p_enu = R * p_camera + translation, evaluated at capture time.

    The caller supplies a calibrated camera-to-world transform (including vehicle
    attitude where necessary), never just the latest vehicle position.
    """
    source_camera: str
    timestamp_ns: int
    origin_id: str
    translation_m: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    transform_id: str

    def __post_init__(self):
        valid_ns(self.timestamp_ns)
        if not all(isinstance(v, str) and v for v in (self.source_camera, self.origin_id, self.transform_id)):
            raise ValueError("Transform requires camera, origin, and transform identity")
        p = np.asarray(self.translation_m, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("Expected finite translation in ENU metres")
        quaternion_matrix(self.orientation)
        object.__setattr__(self, "translation_m", tuple(float(v) for v in p))
        object.__setattr__(self, "orientation", tuple(float(v) for v in self.orientation))


@dataclass(frozen=True)
class WorldPose:
    position_x: float | None = None
    position_y: float | None = None
    position_z: float | None = None
    orientation: tuple[float, float, float, float] | None = None
    yaw_deg: float | None = None
    pose_valid: bool = False
    pose_reason: str | None = "transform_unavailable"
    frame: str = "local_enu"
    origin_id: str | None = None
    transform_id: str | None = None

    def to_dict(self):
        result = asdict(self)
        if self.orientation is not None:
            result["orientation"] = dict(zip(("x", "y", "z", "w"), self.orientation))
        return result


def world_pose(detection, observation, transform):
    if not detection.pose_valid:
        return WorldPose(pose_reason=detection.pose_reason or "measurement_pose_unavailable")
    if transform is None:
        return WorldPose()
    if transform.source_camera != observation.camera_id:
        return WorldPose(pose_reason="transform_camera_mismatch")
    if transform.timestamp_ns != observation.simulation_timestamp_ns:
        return WorldPose(pose_reason="transform_time_mismatch")
    try:
        position = np.asarray([detection.position_x, detection.position_y, detection.position_z], dtype=float)
        rotation = quaternion_matrix(detection.orientation)
        if not np.isfinite(position).all():
            raise ValueError("Nonfinite pose")
        camera_rotation = quaternion_matrix(transform.orientation)
        position = camera_rotation @ position + np.asarray(transform.translation_m)
        rotation = camera_rotation @ rotation
        if not np.isfinite(position).all():
            raise ValueError("Transformed position overflow")
        q = matrix_quaternion(rotation)
    except (ValueError, TypeError, cv2.error):
        return WorldPose(pose_reason="invalid_measurement_pose")
    # Heading is the object's +X axis projected into ENU XY. Vertical +X has no yaw.
    yaw = None if np.hypot(rotation[0, 0], rotation[1, 0]) < 1e-8 else (
        math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])) % 360)
    return WorldPose(*(float(v) for v in position), q, yaw, True, None,
                     origin_id=transform.origin_id, transform_id=transform.transform_id)


@dataclass(frozen=True)
class SharedDetectionResult:
    timestamp_ns: int
    source_timestamp_ns: int
    source_clock: str
    source_id: str
    source_camera: str
    object_id: str
    object_type: str
    confidence: float | None
    confidence_method: str | None
    corners_px: tuple
    pose: WorldPose
    tracking_id: str | None = None
    association_reason: str | None = None
    valid: bool = True
    reason: str | None = None
    clock: str = "simulation"

    def to_dict(self):
        result = asdict(self)
        result.pop("pose")
        result.update(self.pose.to_dict())
        return result


@dataclass(frozen=True)
class TrackState:
    tracking_id: str
    object_id: str
    object_type: str
    source_id: str
    source_camera: str
    status: str
    estimate_kind: str
    last_observed_timestamp_ns: int
    estimate_timestamp_ns: int | None
    pose: WorldPose
    confidence: float | None = None
    hits: int = 1
    valid: bool = True
    reason: str | None = None
    velocity_m_s: tuple[float, float, float] | None = None
    position_covariance_m2: tuple | None = None

    def to_dict(self):
        result = asdict(self)
        result.pop("pose")
        result.update(self.pose.to_dict())
        return result


@dataclass(frozen=True)
class PerceptionUpdate:
    session_id: str
    source_id: str
    source_camera: str
    update_timestamp_ns: int
    capture_timestamp_ns: int
    source_timestamp_ns: int
    source_clock: str
    frame_index: int
    synthetic: bool
    sample_received: bool
    frame_valid: bool | None
    valid: bool
    reason: str | None
    processing_status: str
    detections: tuple[SharedDetectionResult, ...]
    tracks: tuple[TrackState, ...]
    timings_ns: tuple[tuple[str, int], ...] = ()
    schema_name: str = "perception_update"
    schema_version: int = 1
    clock: str = "simulation"

    def to_dict(self):
        result = asdict(self)
        result["detections"] = [d.to_dict() for d in self.detections]
        result["tracks"] = [t.to_dict() for t in self.tracks]
        result["timings_ns"] = dict(self.timings_ns)
        return result
