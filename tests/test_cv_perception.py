"""Pixel/pose and compatibility tests, independent of mission-loop scheduling."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from cv.camera_providers import ImageSequenceProvider, RecordedVideoProvider, render_marker
from cv.perception_pipeline import PerceptionPipeline
from cv.perception_types import CameraCalibration, FrameSample
from state_machine.state_machine import State


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.calibration = CameraCalibration("test_camera", 640, 480,
                                             ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)))
        self.records = []

    def pipeline(self, images, **kwargs):
        provider = ImageSequenceProvider(images, calibration_id=self.calibration.calibration_id)
        kwargs.setdefault("log_sink", self.records.append)
        pipeline = PerceptionPipeline(provider, **kwargs)
        self.addCleanup(pipeline.close)
        return pipeline

    def test_pixels_identity_empty_missing_eof_and_legacy_guard(self):
        marker = render_marker(self.calibration)
        blank = np.full_like(marker, 255)
        pipeline = self.pipeline([marker, blank, None], timestamp_mapper=lambda ns: ns + 123)
        first, empty, missing = [pipeline.read() for _ in range(3)]
        self.assertEqual(first.detections[0].object_id, "aruco:0:17")
        self.assertTrue(first.valid)
        self.assertEqual(first.status, "detected")
        self.assertIsNone(first.to_dict()["bird"])
        self.assertIsNone(first.detections[0].confidence)
        self.assertFalse(first.detections[0].pose_valid)
        self.assertEqual(empty.detections, ())
        self.assertTrue(empty.valid)
        self.assertEqual(empty.status, "absent")
        self.assertIsNone(missing.detections)
        self.assertFalse(missing.valid)
        self.assertEqual(missing.reason, "missing_frame")
        self.assertEqual(empty.timestamp_ns, 100_000_000)
        self.assertEqual(empty.simulation_timestamp_ns, 100_000_123)
        with self.assertRaises(ValueError):
            first.to_bird_snapshot()
        for _ in range(2):
            with self.assertRaises(StopIteration):
                pipeline.read()
        self.assertEqual(len(self.records), 3)
        json.dumps([r for r in self.records], allow_nan=False)

    def test_calibrated_pose_translation_and_rotation(self):
        for rvec, tvec in [((np.pi, 0., 0.), (0., 0., 1.)),
                           ((2.9, .2, .1), (.08, -.04, 1.2))]:
            with self.subTest(rvec=rvec):
                image = render_marker(self.calibration, rvec=rvec, tvec=tvec)
                result = self.pipeline([image], calibration=self.calibration, marker_side_m=.2).read().detections[0]
                self.assertTrue(result.pose_valid, result.pose_reason)
                np.testing.assert_allclose([result.position_x, result.position_y, result.position_z], tvec, atol=.025)
                angle = np.linalg.norm(rvec)
                truth_q = np.r_[np.array(rvec) / angle * np.sin(angle / 2), np.cos(angle / 2)]
                error = 2 * np.arccos(np.clip(abs(np.dot(truth_q, result.orientation)), 0, 1))
                self.assertLess(np.degrees(error), 5.)
                self.assertLess(result.reprojection_error_px, 1.)

    def test_pose_unavailable_does_not_erase_detection(self):
        image = render_marker(self.calibration)
        wrong_size = CameraCalibration("test_camera", 320, 240, self.calibration.matrix)
        wrong_id = CameraCalibration("other_camera", 640, 480, self.calibration.matrix)
        for calibration, side, reason in [(wrong_size, .2, "calibration_resolution_mismatch"),
                                           (wrong_id, .2, "calibration_mismatch"),
                                           (self.calibration, None, "marker_size_unavailable")]:
            result = self.pipeline([image], calibration=calibration, marker_side_m=side).read()
            self.assertTrue(result.valid)
            self.assertEqual(result.detections[0].pose_reason, reason)
            self.assertIsNone(result.detections[0].position_z)

    def test_multiple_markers_and_source_metadata_do_not_supply_detections(self):
        left = render_marker(self.calibration, marker_id=3, tvec=(-.25, 0, 1.))
        right = render_marker(self.calibration, marker_id=17, tvec=(.25, 0, 1.))
        image = np.minimum(left, right)
        outputs = []
        for source_id in ("claims_empty", "claims_marker_49"):
            with PerceptionPipeline(ImageSequenceProvider([image], source_id=source_id),
                                    log_sink=self.records.append) as pipeline:
                outputs.append(pipeline.read().detections)
        self.assertEqual(outputs[0], outputs[1])
        self.assertEqual({d.object_id for d in outputs[0]}, {"aruco:0:3", "aruco:0:17"})

    def test_video_decode_failure_is_not_eof(self):
        capture = Mock()
        capture.isOpened.return_value = True
        capture.get.side_effect = [10., 2.]
        capture.read.return_value = (False, None)
        with patch("cv.camera_providers.cv2.VideoCapture", return_value=capture):
            provider = RecordedVideoProvider("test.avi")
            with self.assertRaises(OSError):
                provider.read()
        capture.release.assert_called_once()

    def test_all_mission_names_are_passive_logging_context(self):
        image = render_marker(self.calibration)
        published = []
        pipeline = self.pipeline([image] * len(State), publisher=published.append)
        for state in State:
            pipeline.set_log_context(state=state.value)
            result = pipeline.read()
            self.assertEqual(self.records[-1]["State"], state.value)
            self.assertEqual(result.detections[0].object_id, "aruco:0:17")
            timings = dict(result.timings_ns)
            self.assertTrue(all(value >= 0 for value in timings.values()))
            self.assertGreaterEqual(timings["total_processing"], sum(v for k, v in timings.items() if k != "total_processing"))
        self.assertEqual(len(published), len(State))

    def test_detection_error_is_not_valid_absence(self):
        pipeline = self.pipeline([render_marker(self.calibration)])
        pipeline.detector = Mock()
        pipeline.detector.detectMarkers.side_effect = cv2.error("test failure")
        result = pipeline.read()
        self.assertTrue(result.camera_ok)
        self.assertFalse(result.valid)
        self.assertEqual(result.processing_status, "error")
        self.assertEqual(result.reason, "detection_failed")
        self.assertIsNone(result.detections)

    def test_video_pixels_timestamps_and_eof(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "markers.avi"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 10., (640, 480))
            self.assertTrue(writer.isOpened(), "Test requires OpenCV MJPG encoder")
            try:
                writer.write(render_marker(self.calibration, marker_id=23))
                writer.write(np.full((480, 640, 3), 255, np.uint8))
            finally:
                writer.release()
            provider = RecordedVideoProvider(path, calibration_id="test_camera")
            with PerceptionPipeline(provider, log_sink=self.records.append) as pipeline:
                detected, empty = pipeline.read(), pipeline.read()
                self.assertEqual(detected.detections[0].object_id, "aruco:0:23")
                self.assertFalse(detected.synthetic)
                self.assertEqual(empty.timestamp_ns, 100_000_000)
                self.assertEqual(empty.clock, "media_fps")
                self.assertEqual(empty.detections, ())
                with self.assertRaises(StopIteration):
                    pipeline.read()
            self.assertFalse(provider._capture.isOpened())

    def test_acquisition_failure_is_logged_and_closes_source(self):
        provider = Mock()
        provider.read.side_effect = OSError("corrupt recording")
        pipeline = PerceptionPipeline(provider, log_sink=self.records.append)
        with self.assertRaises(OSError):
            pipeline.read()
        provider.close.assert_called_once()
        self.assertEqual(self.records[-1]["Details"]["event"], "acquisition_error")

    def test_contract_validation_and_explicit_close(self):
        with self.assertRaises(ValueError):
            FrameSample(None, 0, 0, "front", "test")
        with self.assertRaises(ValueError):
            ImageSequenceProvider([], fps=0)
        with self.assertRaises(ValueError):
            CameraCalibration("bad", 640, 480, ((float('nan'), 0, 0), (0, 1, 0), (0, 0, 1)))
        provider = ImageSequenceProvider([render_marker(self.calibration)])
        provider.close()
        provider.close()
        with self.assertRaises(StopIteration):
            provider.read()


if __name__ == "__main__":
    unittest.main()
