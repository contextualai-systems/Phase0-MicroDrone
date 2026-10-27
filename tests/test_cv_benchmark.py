"""Ground-truth accounting, reproducibility, and full saved-image replay."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from cv.benchmark import SavedImageProvider, run_benchmark
from cv.benchmark_dataset import generate_dataset, render_case
from cv.benchmark_metrics import evaluate_frame, reliable_range, summarize, wilson
from cv.perception_types import DetectionResult


class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.truth = {"object_id": "aruco:0:17", "corners_px": [[0, 0], [10, 0], [10, 10], [0, 10]],
                      "position_m": [0, 0, 1], "rotation_matrix": np.eye(3).tolist()}
        self.detection = DetectionResult("aruco:0:17", "aruco_marker", tuple(map(tuple, self.truth["corners_px"])),
                                         position_x=0., position_y=0., position_z=1.,
                                         orientation=(0., 0., 0., 1.), pose_valid=True, pose_reason=None)

    def test_matching_duplicate_wrong_id_and_corner_gate(self):
        wrong = replace(self.detection, object_id="aruco:0:4")
        row = evaluate_frame([self.truth], [self.detection, self.detection, wrong])
        self.assertEqual((row["tp"], row["fp"], row["fn"]), (1, 2, 0))
        far = replace(self.detection, corners_px=((50, 50), (60, 50), (60, 60), (50, 60)))
        for detection in (wrong, far):
            row = evaluate_frame([self.truth], [detection])
            self.assertEqual((row["tp"], row["fp"], row["fn"]), (0, 1, 1))

    def test_empty_missing_and_pose_coverage(self):
        rows = [evaluate_frame([], []), evaluate_frame([], [self.detection]),
                evaluate_frame([self.truth], None),
                evaluate_frame([self.truth], [replace(self.detection, pose_valid=False)])]
        for row in rows:
            row["processing_status"] = "ok"
        rows[2]["processing_status"] = "error"
        result = summarize(rows)
        self.assertEqual((result["tp"], result["fp"], result["fn"]), (1, 1, 1))
        self.assertEqual(result["fp_per_negative_frame"], .5)
        self.assertEqual(result["pose_coverage_all_truth"], 0)
        self.assertEqual(result["processing_failures"], 1)
        self.assertIsNone(summarize([])["detection_rate"])
        self.assertIsNone(summarize([])["xyz_error_m"]["z"]["rmse"])
        json.dumps(result, allow_nan=False)

    def test_pose_error_and_quaternion_sign(self):
        row = evaluate_frame([self.truth], [replace(self.detection, position_x=.1,
                                                   orientation=(0., 0., 0., -1.))])
        result = summarize([row])
        self.assertAlmostEqual(result["xyz_error_m"]["x"]["rmse"], .1)
        self.assertAlmostEqual(result["rotation_error_deg"]["mae"], 0)
        malformed = evaluate_frame([self.truth], [replace(self.detection, position_z=float("nan"))])
        self.assertEqual(malformed["malformed_valid_poses"], 1)
        self.assertEqual(malformed["poses"], [])

    def test_latency_percentiles_and_wilson(self):
        rows = []
        for value in (1, 2, 3):
            row = evaluate_frame([], [])
            row.update(processing_status="ok", timings_ns={"detection": value * 1_000_000})
            rows.append(row)
        self.assertEqual(summarize(rows)["latency"]["detection"]["p50_ms"], 2)
        self.assertIsNone(wilson(0, 0))
        lo, hi = wilson(100, 100)
        self.assertLess(lo, 1)
        self.assertAlmostEqual(hi, 1)

    def test_euler_wrap_and_singularity_do_not_inflate_rotation_error(self):
        angle = np.deg2rad(179.)
        truth = {**self.truth, "rotation_matrix": [[np.cos(angle), -np.sin(angle), 0],
                                                     [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]}
        measured = replace(self.detection, orientation=(0., 0., float(np.sin(-angle/2)), float(np.cos(-angle/2))))
        pose = evaluate_frame([truth], [measured])["poses"][0]
        self.assertAlmostEqual(pose["yaw_pitch_roll_error_deg"][0], 2.)
        self.assertAlmostEqual(pose["angular_error_deg"], 2.)
        singular = {**self.truth, "rotation_matrix": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]}
        measured = replace(self.detection, orientation=(0., float(np.sqrt(.5)), 0., float(np.sqrt(.5))))
        result = summarize([evaluate_frame([singular], [measured])])
        self.assertEqual(result["euler_excluded_poses"], 1)
        self.assertEqual(result["rotation_error_deg"]["count"], 1)

    def test_multiple_truth_objects_have_independent_matches(self):
        second = {**self.truth, "object_id": "aruco:0:4"}
        row = evaluate_frame([self.truth, second], [self.detection])
        self.assertEqual((row["tp"], row["fp"], row["fn"]), (1, 0, 1))
        row = evaluate_frame([self.truth, second], [self.detection, replace(self.detection, object_id="aruco:0:4")])
        self.assertEqual((row["tp"], row["fp"], row["fn"]), (2, 0, 0))

    def test_range_requires_samples_and_contiguous_success(self):
        specs = {str(d): {"factor": "distance", "distance_m": d} for d in (1, 2, 3)}
        groups = {name: {"eligible_objects": 10, "detection_rate": 1.} for name in specs}
        self.assertIsNone(reliable_range(groups, specs)["maximum_contiguous_tested_distance_m"])
        for group in groups.values():
            group["eligible_objects"] = 100
        groups["2"]["detection_rate"] = .5
        self.assertEqual(reliable_range(groups, specs)["maximum_contiguous_tested_distance_m"], 1)

    def test_seeded_generation_and_degradation_metadata(self):
        a = generate_dataset(self.root / "a", seed=8, samples=1)
        b = generate_dataset(self.root / "b", seed=8, samples=1)
        self.assertEqual(a, b)
        factors = {f["config"]["factor"] for f in a["frames"]}
        self.assertTrue({"distance", "rotation", "tilt", "lighting", "blur", "occlusion", "scale", "clutter", "negative", "combined"} <= factors)
        occluded = next(f for f in a["frames"] if f["condition"] == "occlusion_0.25")
        self.assertAlmostEqual(occluded["truth"][0]["occluded_fraction"], .25, delta=.03)
        image_a, _, _ = render_case({"factor": "clutter", "clutter_count": 20}, 1)
        image_b, _, _ = render_case({"factor": "clutter", "clutter_count": 20}, 2)
        self.assertFalse(np.array_equal(image_a, image_b))
        with self.assertRaises(FileExistsError):
            generate_dataset(self.root / "a", samples=1)

    def test_saved_image_hash_and_path_validation(self):
        manifest = generate_dataset(self.root / "data", samples=1)
        frame = manifest["frames"][0]
        for edited in ({**frame, "sha256": "bad"}, {**frame, "image": "../escape.png"}):
            provider = SavedImageProvider(self.root / "data", [edited], "cal", "source", 10)
            with self.assertRaises(ValueError):
                provider.read()

    def test_end_to_end_replay_preserves_accuracy_and_artifacts(self):
        first = run_benchmark(self.root / "first", seed=4, samples=1, warmup=0)
        replay = run_benchmark(self.root / "replay", dataset=self.root / "first" / "dataset", warmup=0)
        for name in first["conditions"]:
            a, b = first["conditions"][name].copy(), replay["conditions"][name].copy()
            a.pop("latency")
            b.pop("latency")
            self.assertEqual(a, b, name)
        self.assertEqual(first["conditions"]["clean"]["tp"], 1)
        self.assertEqual(first["conditions"]["negative_clean"]["fp"], 0)
        for file in ("report.md", "report.json", "run.json", "observations.jsonl", "evaluation.jsonl", "opencv_build.txt"):
            self.assertTrue((self.root / "first" / file).is_file())
        records = [json.loads(line) for line in (self.root / "first" / "observations.jsonl").read_text().splitlines()]
        self.assertEqual(len(records), first["overall"]["frames"])
        self.assertNotIn("truth", records[0]["Details"])


if __name__ == "__main__":
    unittest.main()
