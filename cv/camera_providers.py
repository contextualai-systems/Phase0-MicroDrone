"""Pull-based image sources. The coordinator owns scheduling and clock mapping."""

from collections.abc import Iterable
import math
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from .perception_types import CameraCalibration, FrameSample, marker_corners


class CameraProvider(Protocol):
    def read(self) -> FrameSample: ...
    def close(self) -> None: ...


class ImageSequenceProvider:
    """Finite synthetic pixels; None entries represent explicitly missing frames."""

    def __init__(self, images: Iterable[np.ndarray | None], *, fps: float = 10,
                 camera_id: str = "front", source_id: str = "synthetic_images",
                 calibration_id: str | None = None):
        if not math.isfinite(fps) or not 0 < fps <= 1_000_000_000:
            raise ValueError("FPS must be positive and allow distinct nanosecond timestamps")
        self._images = iter(images)
        self._index = 0
        self._closed = False
        self.fps, self.camera_id, self.source_id = fps, camera_id, source_id
        self.calibration_id = calibration_id

    def read(self) -> FrameSample:
        if self._closed:
            raise StopIteration
        try:
            image = next(self._images)
        except StopIteration:
            self.close()
            raise
        sample = FrameSample(
            None if image is None else image.copy(), round(self._index * 1e9 / self.fps),
            self._index, self.camera_id, self.source_id,
            reason="missing_frame" if image is None else None,
            calibration_id=self.calibration_id,
        )
        self._index += 1
        return sample

    def close(self) -> None:
        self._closed = True


class RecordedVideoProvider:
    """Constant-rate file playback using an explicit frame-index/FPS clock.

    Unknown frame count is rejected: a failed decode must not silently mean EOF.
    Variable-rate recordings need a timestamp-aware provider in a later phase.
    """

    def __init__(self, path: str | Path, *, camera_id: str = "front",
                 calibration_id: str | None = None):
        self._capture = cv2.VideoCapture(str(path))
        self._index = 0
        self._closed = False
        self.camera_id, self.source_id = camera_id, str(Path(path).resolve())
        self.calibration_id = calibration_id
        fps = self._capture.get(cv2.CAP_PROP_FPS)
        count = self._capture.get(cv2.CAP_PROP_FRAME_COUNT)
        if (not self._capture.isOpened() or not math.isfinite(fps) or not 0 < fps <= 1e9
                or not math.isfinite(count) or count < 1):
            self.close()
            raise ValueError("Video requires a readable file, positive FPS, and known frame count")
        self.fps, self.frame_count = fps, int(count)

    def read(self) -> FrameSample:
        if self._closed or self._index >= self.frame_count:
            self.close()
            raise StopIteration
        try:
            ok, image = self._capture.read()
        except cv2.error as exc:
            self.close()
            raise OSError("Video decode failed before declared EOF") from exc
        if not ok:
            self.close()
            raise OSError("Video decode failed before declared EOF")
        sample = FrameSample(image, round(self._index * 1e9 / self.fps), self._index,
                             self.camera_id, self.source_id, clock="media_fps",
                             synthetic=False, calibration_id=self.calibration_id)
        self._index += 1
        return sample

    def close(self) -> None:
        self._capture.release()
        self._closed = True


def render_marker(calibration: CameraCalibration, *, marker_id: int = 17,
                  side_m: float = .2, rvec=(np.pi, 0., 0.), tvec=(0., 0., 1.),
                  dictionary_id: int = cv2.aruco.DICT_4X4_50) -> np.ndarray:
    """Clean calibrated scene; truth stays with the caller, never in FrameSample.

    Initial renderer supports zero distortion only. Pose estimation also accepts
    nonzero distortion for recorded images.
    """
    if any(calibration.distortion):
        raise ValueError("Clean renderer requires zero distortion")
    rvec, tvec = np.asarray(rvec, dtype=float).reshape(3), np.asarray(tvec, dtype=float).reshape(3)
    if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        raise ValueError("Pose must be finite")
    corners = marker_corners(side_m)
    rotation, _ = cv2.Rodrigues(rvec)
    if np.any((corners @ rotation.T + tvec)[:, 2] <= 0):
        raise ValueError("Marker must be in front of the camera")
    projected, _ = cv2.projectPoints(corners, rvec, tvec, np.array(calibration.matrix),
                                    np.array(calibration.distortion))
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 256)
    transform = cv2.getPerspectiveTransform(
        np.float32([[0, 0], [255, 0], [255, 255], [0, 255]]),
        projected.reshape(4, 2).astype(np.float32))
    gray = cv2.warpPerspective(marker, transform, (calibration.width, calibration.height),
                               flags=cv2.INTER_NEAREST, borderValue=255)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
