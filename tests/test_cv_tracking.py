"""Shared publication, temporal association, geometry, and failure semantics."""

from dataclasses import replace
import json
import unittest

import numpy as np

from cv.camera_providers import ImageSequenceProvider, render_marker
from cv.computer_vision import CVObservation
from cv.perception_pipeline import PerceptionPipeline
from cv.perception_publisher import PerceptionPublisher
from cv.perception_types import CameraCalibration, DetectionResult
from cv.shared_perception import CameraToENU
from state_machine.state_machine import State

MS = 1_000_000


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.records = []
        self.output = PerceptionPublisher(session_id="test", log_sink=self.records.append)
        self.marker = DetectionResult("aruco:0:17", "aruco_marker",
            ((0., 0.), (10., 0.), (10., 10.), (0., 10.)), position_x=1., position_y=2., position_z=3.,
            orientation=(0., 0., 0., 1.), pose_valid=True, pose_reason=None)

    def observation(self, index, detections=None, *, missing=False, error=False):
        return CVObservation(None, None, None, not missing, index*100*MS, index, "front", "fixture",
            "missing_frame" if missing else "detection_failed" if error else None,
            simulation_timestamp_ns=index*100*MS,
            detections=None if missing or error else (self.marker,) if detections is None else detections,
            processing_status="unavailable" if missing else "error" if error else "ok")

    def transform(self, index, **kwargs):
        return CameraToENU("front", index*100*MS, "world", (10., 20., 30.), (0., 0., 0., 1.), "extrinsics-v1", **kwargs)

    def publish(self, index, **kwargs):
        return self.output.update(self.observation(index, **kwargs), now_ns=index*100*MS, transform=self.transform(index))

    def test_confirmation_retention_reacquisition_and_expiry(self):
        first, second = self.publish(0), self.publish(1)
        identity = first.tracks[0].tracking_id
        self.assertEqual(first.tracks[0].status, "tentative")
        self.assertEqual(second.tracks[0].status, "confirmed")
        self.assertEqual(second.detections[0].tracking_id, identity)
        empty = self.publish(2, detections=())
        self.assertEqual(empty.detections, ())
        self.assertEqual(empty.tracks[0].status, "temporarily_lost")
        self.assertEqual(empty.tracks[0].estimate_kind, "retained")
        self.assertEqual(empty.tracks[0].estimate_timestamp_ns, 100*MS)
        self.assertEqual(empty.tracks[0].last_observed_timestamp_ns, 100*MS)
        returned = self.publish(3)
        self.assertEqual(returned.tracks[0].tracking_id, identity)
        self.assertEqual(returned.tracks[0].status, "confirmed")
        expired = self.output.update(now_ns=800*MS)
        self.assertFalse(expired.tracks[0].valid)
        self.assertEqual(expired.tracks[0].status, "expired")
        self.assertEqual(self.output.update(now_ns=801*MS).tracks, ())
        self.assertNotEqual(self.publish(9).tracks[0].tracking_id, identity)

    def test_heartbeat_does_not_refresh_measurement_or_confirm_track(self):
        self.publish(0)
        update = self.output.update(now_ns=50*MS)
        self.assertFalse(update.sample_received)
        self.assertIsNone(update.frame_valid)
        self.assertFalse(update.valid)
        self.assertEqual(update.capture_timestamp_ns, 0)
        self.assertEqual(update.tracks[0].hits, 1)
        self.assertEqual(update.tracks[0].status, "tentative")
        self.assertEqual(update.tracks[0].estimate_timestamp_ns, 0)
        self.assertEqual(self.publish(1).tracks[0].status, "confirmed")

    def test_missing_and_processing_failure_preserve_tracks_not_detections(self):
        self.publish(0)
        missing = self.publish(1, missing=True)
        failed = self.publish(2, error=True)
        self.assertFalse(missing.frame_valid)
        self.assertTrue(failed.frame_valid)
        for result in (missing, failed):
            self.assertFalse(result.valid)
            self.assertEqual(result.detections, ())
            self.assertEqual(result.tracks[0].last_observed_timestamp_ns, 0)
            self.assertEqual(result.tracks[0].estimate_kind, "retained")
        self.assertEqual(self.publish(3).tracks[0].status, "tentative")
        self.assertEqual(self.publish(4).tracks[0].status, "confirmed")

    def test_duplicate_identity_is_not_silently_associated(self):
        first = self.publish(0)
        result = self.publish(1, detections=(self.marker, self.marker))
        self.assertTrue(all(d.tracking_id is None and d.association_reason == "ambiguous_object_id" for d in result.detections))
        self.assertEqual(len(result.tracks), 1)
        self.assertEqual(result.tracks[0].tracking_id, first.tracks[0].tracking_id)
        self.assertEqual(result.tracks[0].last_observed_timestamp_ns, 0)

    def test_multiple_marker_identities_have_distinct_tracks(self):
        second = replace(self.marker, object_id="aruco:0:18")
        result = self.publish(0, detections=(self.marker, second))
        ids = {d.object_id: d.tracking_id for d in result.detections}
        self.assertEqual(len(set(ids.values())), 2)
        result = self.publish(1, detections=(second, self.marker))
        self.assertEqual({d.object_id: d.tracking_id for d in result.detections}, ids)

    def test_transform_translation_rotation_and_yaw(self):
        q = (0., 0., float(np.sqrt(.5)), float(np.sqrt(.5)))
        transform = replace(self.transform(0), orientation=q)
        result = self.output.update(self.observation(0), now_ns=0, transform=transform)
        payload = result.to_dict()
        d = payload["detections"][0]
        np.testing.assert_allclose([d["position_x"], d["position_y"], d["position_z"]], [8, 21, 33])
        self.assertAlmostEqual(d["yaw_deg"], 90.)
        self.assertEqual(d["frame"], "local_enu")
        self.assertEqual(d["origin_id"], "world")
        self.assertEqual(d["object_type"], "aruco_marker")
        self.assertEqual(d["source_camera"], "front")
        self.assertEqual(set(d["orientation"]), {"x", "y", "z", "w"})
        self.assertIsNone(payload["tracks"][0]["velocity_m_s"])
        self.assertIsNone(payload["tracks"][0]["position_covariance_m2"])
        json.dumps(payload, allow_nan=False)

    def test_missing_wrong_time_or_wrong_camera_transform_keeps_detection(self):
        for transform, reason in [(None, "transform_unavailable"),
            (replace(self.transform(0), timestamp_ns=1), "transform_time_mismatch"),
            (replace(self.transform(0), source_camera="down"), "transform_camera_mismatch")]:
            output = PerceptionPublisher(log_sink=lambda _: None)
            result = output.update(self.observation(0), now_ns=0, transform=transform)
            self.assertTrue(result.valid)
            self.assertTrue(result.detections[0].valid)
            self.assertFalse(result.detections[0].pose.pose_valid)
            self.assertEqual(result.detections[0].pose.pose_reason, reason)
            self.assertIsNone(result.detections[0].pose.position_z)
            self.assertIsNone(result.tracks[0].estimate_timestamp_ns)

    def test_invalid_pose_clears_geometry_without_losing_identity(self):
        first = self.publish(0)
        result = self.publish(1, detections=(replace(self.marker, pose_valid=False, pose_reason="calibration_unavailable"),))
        self.assertEqual(result.tracks[0].tracking_id, first.tracks[0].tracking_id)
        self.assertEqual(result.tracks[0].last_observed_timestamp_ns, 100*MS)
        self.assertIsNone(result.tracks[0].estimate_timestamp_ns)
        self.assertFalse(result.tracks[0].pose.pose_valid)

    def test_validation_rejects_replay_future_and_clock_changes_without_mutation(self):
        first = self.publish(0)
        cases = [self.observation(0), replace(self.observation(1), simulation_timestamp_ns=999*MS),
                 replace(self.observation(1), simulation_timestamp_ns=None),
                 replace(self.observation(1), source_id="other"),
                 replace(self.observation(1), timestamp_ns=0)]
        for sample in cases:
            with self.assertRaises(ValueError):
                self.output.update(sample, now_ns=100*MS)
        good = self.publish(1)
        self.assertEqual(good.tracks[0].hits, 2)
        self.assertEqual(first.tracks[0].tracking_id, good.tracks[0].tracking_id)
        with self.assertRaises(ValueError):
            self.output.update(now_ns=0)

    def test_stale_observation_does_not_create_fresh_track(self):
        result = self.output.update(self.observation(0), now_ns=500*MS)
        self.assertEqual(result.detections[0].timestamp_ns, 0)
        self.assertEqual(result.detections[0].association_reason, "stale_observation")
        self.assertEqual(result.tracks, ())

    def test_offset_mapping_keeps_source_and_capture_time_distinct(self):
        sample = replace(self.observation(0), simulation_timestamp_ns=1_000*MS)
        transform = replace(self.transform(0), timestamp_ns=1_000*MS)
        update = self.output.update(sample, now_ns=1_100*MS, transform=transform)
        self.assertEqual(update.capture_timestamp_ns, 1_000*MS)
        self.assertEqual(update.update_timestamp_ns, 1_100*MS)
        self.assertEqual(update.source_timestamp_ns, 0)
        self.assertEqual(update.detections[0].timestamp_ns, 1_000*MS)
        self.assertEqual(update.tracks[0].last_observed_timestamp_ns, 1_000*MS)

    def test_nonmarker_detections_are_published_without_false_association(self):
        result = self.publish(0, detections=(replace(self.marker, type="person"),))
        self.assertEqual(result.tracks, ())
        self.assertEqual(result.detections[0].association_reason, "unsupported_tracking_type")

    def test_publication_failure_does_not_permit_replaying_measurement(self):
        def fail(update):
            raise OSError("consumer unavailable")
        output = PerceptionPublisher(publisher=fail, log_sink=self.records.append)
        with self.assertRaises(OSError):
            output.update(self.observation(0), now_ns=0)
        self.assertEqual(self.records[-1]["Details"]["event"], "perception_update_publication_error")
        with self.assertRaises(ValueError):
            output.update(self.observation(0), now_ns=0)

    def test_explicit_reset_changes_ids_and_permits_clock_restart(self):
        first = self.publish(0)
        with self.assertRaises(ValueError):
            self.output.update(self.observation(1), now_ns=100*MS,
                               transform=replace(self.transform(1), origin_id="other"))
        self.output.reset()
        next_update = self.publish(0)
        self.assertNotEqual(first.session_id, next_update.session_id)
        self.assertNotEqual(first.tracks[0].tracking_id, next_update.tracks[0].tracking_id)
        self.assertEqual(self.records[-2]["Details"]["event"], "tracking_reset")

    def test_state_names_are_logging_context_and_callback_receives_update(self):
        published = []
        output = PerceptionPublisher(publisher=published.append, log_sink=self.records.append)
        for i, state in enumerate(State):
            output.set_log_context(state=state.value)
            output.update(self.observation(i), now_ns=i*100*MS)
            self.assertEqual(self.records[-1]["State"], state.value)
        self.assertEqual(len(published), len(State))
        payload = published[0].to_dict()
        payload["tracks"][0]["object_id"] = "changed"
        self.assertEqual(published[0].tracks[0].object_id, self.marker.object_id)

    def test_pipeline_to_shared_update_with_actual_pixels(self):
        calibration = CameraCalibration("test", 640, 480, ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)))
        image = render_marker(calibration)
        provider = ImageSequenceProvider([image, image, None], calibration_id="test")
        with PerceptionPipeline(provider, calibration=calibration, marker_side_m=.2,
                                timestamp_mapper=lambda ns: ns, log_sink=lambda _: None) as pipeline:
            updates = [self.output.update(pipeline.read(), now_ns=i*100*MS,
                                          transform=self.transform(i)) for i in range(3)]
        self.assertEqual(updates[0].detections[0].object_id, "aruco:0:17")
        self.assertEqual(updates[1].tracks[0].status, "confirmed")
        self.assertEqual(updates[2].tracks[0].estimate_kind, "retained")
        self.assertEqual(updates[2].detections, ())

    def test_transform_validation_and_vertical_axis_heading(self):
        with self.assertRaises(ValueError):
            replace(self.transform(0), orientation=(0., 0., 0., 0.))
        with self.assertRaises(ValueError):
            replace(self.transform(0), translation_m=(float("nan"), 0., 0.))
        transform = replace(self.transform(0), orientation=(0., float(np.sqrt(.5)), 0., float(np.sqrt(.5))))
        result = self.output.update(self.observation(0), now_ns=0, transform=transform)
        self.assertTrue(result.detections[0].pose.pose_valid)
        self.assertIsNone(result.detections[0].pose.yaw_deg)


if __name__ == "__main__":
    unittest.main()
