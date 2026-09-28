"""Integration tests for the CV interface using tiny synthetic scenarios."""

import json
from pathlib import Path
import tempfile
import unittest

from cv.computer_vision import ComputerVision


class ComputerVisionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "scenario.json"
        self.scenario = {
            "scenario_id": "test", "fps": 10, "loop": True, "appearance": "bird",
            "segments": [
                {"frames": 1, "camera_ok": True,
                 "bird": {"detected": True, "type": "fixture", "distance_m": 12}},
                {"frames": 1, "camera_ok": True,
                 "bird": {"detected": False, "type": None, "distance_m": None}},
                {"frames": 1, "camera_ok": False, "bird": None},
            ],
        }

    def start(self):
        self.path.write_text(json.dumps(self.scenario), encoding="utf-8")
        cv = ComputerVision()
        cv.start_camera(self.path, "downward")
        return cv

    def test_present_empty_missing_and_wrap(self):
        cv = self.start()
        present = cv.read()
        self.assertTrue(present.detected)
        self.assertEqual(cv.gray_frame.shape, (480, 640))
        empty = cv.read()
        self.assertFalse(empty.detected)
        self.assertTrue(empty.camera_ok)
        missing = cv.read()
        self.assertIsNone(missing.detected)
        self.assertIsNone(missing.to_dict()["bird"])
        self.assertFalse(missing.camera_ok)
        self.assertIsNone(cv.frame)
        self.assertIsNone(cv.gray_frame)
        repeated = cv.read()
        self.assertEqual(repeated.timestamp_ns, 300_000_000)
        self.assertEqual(repeated.camera_id, "downward")
        self.assertEqual(present.to_dict()["bird"], repeated.to_dict()["bird"])

    def test_lifecycle(self):
        cv = ComputerVision()
        with self.assertRaises(RuntimeError):
            cv.read()
        cv = self.start()
        with self.assertRaises(RuntimeError):
            cv.start_camera(self.path)
        cv.stop_camera()
        cv.stop_camera()
        with self.assertRaises(RuntimeError):
            cv.read()
        cv.start_camera(self.path)
        self.assertEqual(cv.read().timestamp_ns, 0)

    def test_finite_source_ends_and_clears_frames(self):
        self.scenario["loop"] = False
        self.scenario["segments"] = self.scenario["segments"][:1]
        cv = self.start()
        cv.read()
        with self.assertRaises(StopIteration):
            cv.read()
        self.assertIsNone(cv.frame)
        with self.assertRaises(RuntimeError):
            cv.read()


if __name__ == "__main__":
    unittest.main()
