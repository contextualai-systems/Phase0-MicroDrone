"""Sequence truth, intentionally broken identity traces, and pixel-path replay."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from cv.shared_perception import PerceptionUpdate, SharedDetectionResult, TrackState, WorldPose
from cv.tracking_benchmark import run
from cv.tracking_metrics import evaluate_sequence
from cv.tracking_sequences import make_sequence, SCENARIOS


class TrackingBenchmarkTests(unittest.TestCase):
    def truth(self, index, *, observable=True, duplicate=False):
        obj = {"entity_id": "A", "object_id": "aruco:0:17", "observable": observable,
               "corners_px": [[0, 0], [10, 0], [10, 10], [0, 10]]}
        objects = [obj]
        if duplicate:
            objects.append({**obj, "entity_id": "C", "corners_px": [[20, 0], [30, 0], [30, 10], [20, 10]]})
        return {"frame_index": index, "timestamp_ns": index*100_000_000, "objects": objects}

    def detection(self, index, identity="track1", *, offset=0, reason=None):
        return SharedDetectionResult(index*100_000_000, index*100_000_000, "fixture", "test", "front",
            "aruco:0:17", "aruco_marker", None, None,
            ((offset, 0), (offset+10, 0), (offset+10, 10), (offset, 10)), WorldPose(), identity, reason)

    def update(self, index, detections=(), tracks=()):
        return PerceptionUpdate("session", "test", "front", index*100_000_000,
                                index*100_000_000, index*100_000_000, "fixture", index, True,
                                True, True, True, None, "ok", tuple(detections), tuple(tracks))

    def test_seeded_sequences_include_all_required_events(self):
        for name in SCENARIOS:
            a, truth_a = make_sequence(name, 5)
            b, truth_b = make_sequence(name, 5)
            self.assertEqual(truth_a, truth_b)
            self.assertEqual(len(a), 36)
            self.assertTrue(all((x is None and y is None) or np.array_equal(x, y) for x, y in zip(a, b)))
        images, _ = make_sequence("missing_frames")
        self.assertEqual(sum(image is None for image in images), 3)
        _, truth = make_sequence("duplicate_id")
        self.assertEqual(len(truth[10]["objects"]), 3)

    def test_unexpected_switch_and_missed_detection_reduce_continuity(self):
        frames = [self.truth(i) for i in range(3)]
        updates = [self.update(0, [self.detection(0)]), self.update(1, [self.detection(1, "wrong")]), self.update(2)]
        report, _ = evaluate_sequence(frames, updates)
        self.assertEqual(report["unexpected_id_changes"], 1)
        self.assertEqual(report["identity_continuity"]["opportunities"], 2)
        self.assertEqual(report["identity_continuity"]["successes"], 0)
        self.assertEqual(report["fn"], 1)
        self.assertEqual(report["confirmation"]["unconfirmed"], 1)

    def test_rotating_tilted_markers_keep_identity_through_pixel_pipeline(self):
        _, truth = make_sequence("rotation_tilt")
        corners = np.asarray(truth[0]["objects"][0]["corners_px"])
        # Perspective tilt produces unequal opposite edge lengths.
        self.assertGreater(abs(np.linalg.norm(corners[2]-corners[1]) -
                               np.linalg.norm(corners[3]-corners[0])), 1.)
        with tempfile.TemporaryDirectory() as temp:
            report = run(Path(temp)/"rotation", scenarios=("rotation_tilt",))
        metrics = report["sequences"]["rotation_tilt"]
        self.assertEqual((metrics["tp"], metrics["fp"], metrics["fn"]), (72, 0, 0))
        self.assertEqual(metrics["identity_continuity"]["rate"], 1.)
        self.assertEqual(metrics["unexpected_id_changes"], 0)

    def test_short_gap_and_long_gap_have_different_id_expectations(self):
        frames = [self.truth(0), self.truth(3), self.truth(9)]
        updates = [self.update(0, [self.detection(0)]), self.update(3, [self.detection(3)]),
                   self.update(9, [self.detection(9, "track2")])]
        report, _ = evaluate_sequence(frames, updates)
        self.assertEqual(report["short_gap_reacquisition"]["rate"], 1.)
        self.assertEqual(report["new_id_after_expiry"]["rate"], 1.)
        updates[-1] = self.update(9, [self.detection(9)])
        self.assertEqual(evaluate_sequence(frames, updates)[0]["new_id_after_expiry"]["rate"], 0.)

    def test_ambiguity_requires_detection_and_explicit_refusal(self):
        truth = [self.truth(0, duplicate=True)]
        updates = [self.update(0, [self.detection(0, None, reason="ambiguous_object_id"),
                                   self.detection(0, None, offset=20, reason="ambiguous_object_id")])]
        report, _ = evaluate_sequence(truth, updates)
        self.assertEqual(report["ambiguity_refusal"]["rate"], 1.)
        self.assertEqual(report["tp"], 2)
        report, _ = evaluate_sequence(truth, [self.update(0)])
        self.assertEqual(report["ambiguity_refusal"]["rate"], 0.)
        self.assertEqual(report["fn"], 2)

    def test_occluded_objects_are_not_detection_opportunities(self):
        report, _ = evaluate_sequence([self.truth(0, observable=False)], [self.update(0)])
        self.assertEqual(report["fn"], 0)
        self.assertIsNone(report["detection_rate"])

    def test_expiry_and_retention_timestamp_audit_catches_bad_outputs(self):
        track = TrackState("track1", "aruco:0:17", "aruco_marker", "test", "front", "confirmed",
                           "measured", 0, 0, WorldPose())
        updates = [self.update(0, tracks=[track]),
                   self.update(1, tracks=[replace(track, estimate_kind="retained", estimate_timestamp_ns=100_000_000)]),
                   self.update(5, tracks=[replace(track, estimate_kind="retained")])]
        report, _ = evaluate_sequence([self.truth(i, observable=False) for i in (0, 1, 5)], updates)
        self.assertGreater(report["emitted_lifecycle_audit"]["retention_timestamp_or_pose_violations"], 0)
        self.assertEqual(report["emitted_lifecycle_audit"]["expiry_violations"], 1)

    def test_real_pipeline_replay_and_artifacts(self):
        with tempfile.TemporaryDirectory() as temp:
            first, second = Path(temp)/"first", Path(temp)/"second"
            a = run(first, scenarios=("short_occlusion", "duplicate_id"))
            b = run(second, dataset=first)
            for name in a["sequences"]:
                metrics_a, metrics_b = a["sequences"][name].copy(), b["sequences"][name].copy()
                metrics_a.pop("latency")
                metrics_b.pop("latency")
                self.assertEqual(metrics_a, metrics_b)
                self.assertEqual(metrics_a["unexpected_id_changes"], 0)
            self.assertEqual(a["sequences"]["short_occlusion"]["short_gap_reacquisition"]["rate"], 1.)
            self.assertEqual(a["sequences"]["duplicate_id"]["ambiguity_refusal"]["rate"], 1.)
            for name in ("manifest.json", "report.md", "report.json", "events.jsonl", "updates.jsonl", "evaluations.jsonl", "playback.html"):
                self.assertTrue((first/name).is_file())
            with self.assertRaises(FileExistsError):
                run(first)
            image = next((first/"short_occlusion").glob("input_*.png"))
            image.write_bytes(b"corrupted")
            with self.assertRaises(ValueError):
                run(Path(temp)/"corrupted", dataset=first)


if __name__ == "__main__":
    unittest.main()
