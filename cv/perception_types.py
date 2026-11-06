"""CV-only values; no mission states, actions, or evaluation truth."""

from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DetectionResult:
    object_id: str
    type: str
    corners_px: tuple[tuple[float, float], ...]
    confidence: float | None = None
    confidence_method: str | None = None
    position_x: float | None = None
    position_y: float | None = None
    position_z: float | None = None
    orientation: tuple[float, float, float, float] | None = None
    pose_valid: bool = False
    pose_reason: str | None = "calibration_unavailable"
    reprojection_error_px: float | None = None
    tracking_id: str | None = None

    def to_dict(self) -> dict:
        result = asdict(self)
        if self.orientation is not None:
            result["orientation"] = dict(zip(("x", "y", "z", "w"), self.orientation))
        return result


@dataclass(frozen=True)
class CameraCalibration:
    calibration_id: str
    width: int
    height: int
    matrix: tuple[tuple[float, float, float], ...]
    distortion: tuple[float, ...] = (0., 0., 0., 0., 0.)

    def __post_init__(self):
        matrix = np.asarray(self.matrix, dtype=float)
        distortion = np.asarray(self.distortion, dtype=float)
        if not self.calibration_id or any(type(v) is not int or v <= 0 for v in (self.width, self.height)):
            raise ValueError("Calibration requires an ID and positive integer dimensions")
        if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1])
                or matrix[0, 1] != 0 or matrix[1, 0] != 0):
            raise ValueError("Expected finite pinhole intrinsics with positive focal lengths")
        if distortion.shape not in ((4,), (5,), (8,), (12,), (14,)) or not np.isfinite(distortion).all():
            raise ValueError("Invalid OpenCV distortion coefficients")
        object.__setattr__(self, "matrix", tuple(tuple(float(v) for v in row) for row in matrix))
        object.__setattr__(self, "distortion", tuple(float(v) for v in distortion))


@dataclass(frozen=True)
class FrameSample:
    image: np.ndarray | None
    timestamp_ns: int
    frame_index: int
    camera_id: str
    source_id: str
    clock: str = "fixture"
    synthetic: bool = True
    reason: str | None = None
    calibration_id: str | None = None

    def __post_init__(self):
        if any(type(v) is not int or v < 0 for v in (self.timestamp_ns, self.frame_index)):
            raise ValueError("Sample time and index must be nonnegative integers")
        if not all(isinstance(v, str) and v for v in (self.camera_id, self.source_id, self.clock)):
            raise ValueError("Sample identity and clock must be nonempty strings")
        if self.image is None:
            if not self.reason:
                raise ValueError("Missing images require a reason")
        elif (self.reason is not None or self.image.dtype != np.uint8
              or self.image.ndim != 3 or self.image.shape[2] != 3
              or min(self.image.shape[:2]) <= 0):
            raise ValueError("Available images must be nonempty uint8 BGR with no failure reason")


def marker_corners(side_m: float) -> np.ndarray:
    if isinstance(side_m, bool) or not math.isfinite(side_m) or side_m <= 0:
        raise ValueError("Marker side must be positive finite metres")
    h = side_m / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
