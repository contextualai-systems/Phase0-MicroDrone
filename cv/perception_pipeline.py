"""Pixel-based ArUco sensing. No mission transitions, motion, or world poses."""

from dataclasses import replace
import math
from time import perf_counter_ns
from typing import Callable

import cv2
import numpy as np

from .camera_providers import CameraProvider
from .computer_vision import CVObservation
from .perception_types import CameraCalibration, DetectionResult, marker_corners
from .universal_logging import log_entry


class PerceptionPipeline:
    """One pull processes one sample; scheduling remains with the coordinator.

    `state` is opaque logging context, normally State.value. This module neither
    defines nor chooses mission states. The caller owns explicit clock mapping.
    """

    def __init__(self, provider: CameraProvider, *, calibration: CameraCalibration | None = None,
                 marker_side_m: float | None = None,
                 dictionary_id: int = cv2.aruco.DICT_4X4_50,
                 max_reprojection_error_px: float = 3.,
                 timestamp_mapper: Callable[[int], int] | None = None,
                 publisher: Callable[[CVObservation], object] | None = None,
                 log_sink: Callable[[dict], object] = log_entry, state: str = "IDLE"):
        self.provider, self.calibration = provider, calibration
        self._object_points = None if marker_side_m is None else marker_corners(marker_side_m)
        if not math.isfinite(max_reprojection_error_px) or max_reprojection_error_px <= 0:
            raise ValueError("Reprojection limit must be positive and finite")
        self.max_reprojection_error_px = max_reprojection_error_px
        self.dictionary_id = dictionary_id
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary_id), parameters)
        self._mapper, self._publisher, self._log_sink = timestamp_mapper, publisher, log_sink
        self._closed = False
        self.set_log_context(state=state)

    def set_log_context(self, *, state: str) -> None:
        if not isinstance(state, str) or not state:
            raise ValueError("state must be a nonempty mission-state string")
        self._state = state

    def _pose(self, result: DetectionResult, shape, calibration_id) -> DetectionResult:
        calibration = self.calibration
        if calibration is None:
            return result
        if calibration_id != calibration.calibration_id:
            return replace(result, pose_reason="calibration_mismatch")
        if shape[:2] != (calibration.height, calibration.width):
            return replace(result, pose_reason="calibration_resolution_mismatch")
        if self._object_points is None:
            return replace(result, pose_reason="marker_size_unavailable")
        image_points = np.array(result.corners_px, dtype=np.float64)
        matrix, distortion = np.array(calibration.matrix), np.array(calibration.distortion)
        try:
            ok, rvec, tvec = cv2.solvePnP(self._object_points, image_points, matrix, distortion,
                                         flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                return replace(result, pose_reason="pose_solver_failed")
            rotation, _ = cv2.Rodrigues(rvec)
            if np.any((self._object_points @ rotation.T + tvec.reshape(3))[:, 2] <= 0):
                return replace(result, pose_reason="pose_behind_camera")
            projected, _ = cv2.projectPoints(self._object_points, rvec, tvec, matrix, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points) ** 2, axis=1))))
            if not math.isfinite(error) or error > self.max_reprojection_error_px:
                return replace(result, pose_reason="reprojection_error")
            angle = float(np.linalg.norm(rvec))
            xyz = np.zeros(3) if angle < 1e-12 else rvec.reshape(3) / angle * math.sin(angle / 2)
            quaternion = tuple(float(v) for v in xyz) + (math.cos(angle / 2),)
            x, y, z = (float(v) for v in tvec.reshape(3))
            return replace(result, position_x=x, position_y=y, position_z=z,
                           orientation=quaternion, pose_valid=True, pose_reason=None,
                           reprojection_error_px=error)
        except cv2.error:
            return replace(result, pose_reason="pose_solver_failed")

    def read(self) -> CVObservation:
        if self._closed:
            raise StopIteration
        started = perf_counter_ns()
        try:
            sample = self.provider.read()
        except StopIteration:
            self.close()
            raise
        except Exception as exc:
            self._log_sink({"State": self._state, "Details": {
                "module": "CV", "event": "acquisition_error", "error_type": type(exc).__name__}})
            self.close()
            raise
        timings = {"acquisition": perf_counter_ns() - started}
        mapped = None if self._mapper is None else self._mapper(sample.timestamp_ns)
        if self._mapper is not None and (type(mapped) is not int or mapped < 0):
            raise ValueError("timestamp_mapper must return nonnegative integer nanoseconds")
        results = None
        status = "unavailable" if sample.image is None else "ok"
        reason = sample.reason
        timings.update(preprocessing=0, detection=0, pose_estimation=0)
        if sample.image is not None:
            stage = "preprocessing"
            stage_started = perf_counter_ns()
            try:
                gray = cv2.cvtColor(sample.image, cv2.COLOR_BGR2GRAY)
                timings[stage] = perf_counter_ns() - stage_started
                stage, stage_started = "detection", perf_counter_ns()
                corners, ids, _ = self.detector.detectMarkers(gray)
                timings[stage] = perf_counter_ns() - stage_started
                stage, stage_started = "pose_estimation", perf_counter_ns()
                results = []
                if ids is not None:
                    for marker_id, points in zip(ids.reshape(-1), corners):
                        result = DetectionResult(
                            object_id=f"aruco:{self.dictionary_id}:{int(marker_id)}",
                            type="aruco_marker",
                            corners_px=tuple(tuple(float(v) for v in p) for p in points.reshape(4, 2)),
                        )
                        results.append(self._pose(result, sample.image.shape, sample.calibration_id))
                results = tuple(results)
                timings[stage] = perf_counter_ns() - stage_started
            except cv2.error:
                timings[stage] = perf_counter_ns() - stage_started
                status, reason, results = "error", f"{stage}_failed", None
        observation = CVObservation(
            detected=None, bird_type=None, distance_m=None,
            camera_ok=sample.image is not None, timestamp_ns=sample.timestamp_ns,
            frame_index=sample.frame_index, camera_id=sample.camera_id, source_id=sample.source_id,
            reason=reason, clock=sample.clock, synthetic=sample.synthetic,
            simulation_timestamp_ns=mapped, detections=results, processing_status=status,
            calibration_id=sample.calibration_id, timings_ns=tuple(timings.items()),
        )
        # Callback sees processing timings; its own duration is only known after
        # return and is included in the returned observation and frame log.
        publish_started = perf_counter_ns()
        try:
            if self._publisher is not None:
                self._publisher(observation)
        except Exception as exc:
            self._log_sink({"State": self._state, "Details": {
                "module": "CV", "event": "publication_error", "error_type": type(exc).__name__,
                **observation.to_dict()}})
            raise
        timings["publication"] = perf_counter_ns() - publish_started if self._publisher else 0
        timings["total_processing"] = perf_counter_ns() - started
        observation = replace(observation, timings_ns=tuple(timings.items()))
        self._log_sink({"State": self._state, "Details": {
            "module": "CV", "event": "perception_frame", **observation.to_dict()}})
        return observation

    def close(self) -> None:
        if not self._closed:
            self.provider.close()
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
