"""Integration tests for the CV interface using tiny synthetic scenarios."""

import json
import logging
from pathlib import Path
import tempfile
import unittest

from cv.computer_vision import ComputerVision
from cv.universal_logging import log_entry


class ComputerVisionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "scenario.json"
        self.records = []
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

    def start(self, **kwargs):
        self.path.write_text(json.dumps(self.scenario), encoding="utf-8")
        kwargs.setdefault("log_sink", self.records.append)
        cv = ComputerVision(**kwargs)
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

    def test_in_memory_fixture_matches_file_and_is_isolated(self):
        file_cv = self.start(timestamp_mapper=lambda ns: ns + 100)
        memory_cv = ComputerVision(timestamp_mapper=lambda ns: ns + 100,
                                   log_sink=lambda record: None)
        memory_cv.start_camera(scenario=self.scenario, camera_id="downward")
        self.scenario["segments"][0]["bird"]["distance_m"] = 999
        self.scenario["fps"] = 1
        for now in (100, 100_000_100, 200_000_100, 300_000_100):
            self.assertEqual(file_cv.read_due(now).to_dict(), memory_cv.read_due(now).to_dict())
        file_cv.stop_camera()
        memory_cv.stop_camera()

    def test_in_memory_fixture_validation_does_not_start_camera(self):
        cv = ComputerVision(log_sink=lambda record: None)
        for invalid in ({}, [], "{}"):
            with self.assertRaises(ValueError):
                cv.start_camera(scenario=invalid)
        with self.assertRaises(ValueError):
            cv.start_camera(self.path, scenario=self.scenario)
        cv.start_camera(scenario=self.scenario)
        with self.assertRaises(RuntimeError):
            cv.start_camera(scenario=self.scenario)
        self.assertTrue(cv.read().detected)
        cv.stop_camera()

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

    def test_observation_logs_distinguish_all_three_outcomes(self):
        cv = self.start(state="TRACKING")
        observations = [cv.read() for _ in range(3)]
        self.assertEqual([o.status for o in observations],
                         ["detected", "absent", "unavailable"])
        self.assertEqual([o.valid for o in observations], [True, True, False])
        records = [r for r in self.records if r["Details"]["event"] == "observation"]
        self.assertEqual(len(records), 3)
        for record, observation in zip(records, observations):
            self.assertEqual(record["State"], "TRACKING")
            self.assertEqual(record["Details"], {
                "module": "CV", "event": "observation", **observation.to_dict(),
            })
            self.assertEqual(observation.camera_id, "downward")
            self.assertEqual(observation.source_id, "test")
            self.assertNotIn("mapped_time", observation.to_dict())
        self.assertEqual(records[1]["Details"]["bird"], {
            "detected": False, "type": None, "distance_m": None,
        })
        self.assertIsNone(records[2]["Details"]["bird"])
        self.assertEqual(records[2]["Details"]["reason"], "missing_frame")

    def test_mapping_preserves_fixture_time_and_identity_across_wrap(self):
        calls = []

        def mapper(timestamp):
            calls.append(timestamp)
            return timestamp + 5_000_000_000

        cv = self.start(timestamp_mapper=mapper)
        for index in range(4):
            observation = cv.read()
            expected = index * 100_000_000
            self.assertEqual(observation.timestamp_ns, expected)
            self.assertEqual(observation.clock, "fixture")
            self.assertEqual(observation.frame_index, index)
            self.assertEqual(observation.camera_id, "downward")
            self.assertEqual(observation.source_id, "test")
            self.assertEqual(observation.to_dict()["mapped_time"], {
                "clock": "simulation", "timestamp_ns": expected + 5_000_000_000,
            })
            self.assertEqual(self.records[-1]["Details"]["mapped_time"],
                             observation.to_dict()["mapped_time"])
        self.assertEqual(calls, [0, 100_000_000, 200_000_000, 300_000_000])

    def test_invalid_clock_mapping_does_not_consume_a_sample(self):
        for invalid in (None, True, -1, 0.5, "0"):
            with self.subTest(invalid=invalid):
                mapped = [invalid, 0]
                cv = self.start(timestamp_mapper=lambda timestamp: mapped.pop(0))
                with self.assertRaises(ValueError):
                    cv.read()
                self.assertEqual(cv.read().frame_index, 0)
                cv.stop_camera()

    def test_lifecycle_logs_keep_source_identity_and_updated_state(self):
        self.scenario["loop"] = False
        self.scenario["segments"] = self.scenario["segments"][:1]
        cv = self.start()
        cv.set_log_context(state="HOVERING")
        observation = cv.read()
        with self.assertRaises(StopIteration):
            cv.read()
        cv.stop_camera()  # Must not emit a duplicate stop.
        self.assertEqual([r["Details"]["event"] for r in self.records], [
            "camera_started", "observation", "source_exhausted", "camera_stopped",
        ])
        self.assertEqual(self.records[0]["State"], "IDLE")
        for record in self.records[1:]:
            self.assertEqual(record["State"], "HOVERING")
        for record in self.records:
            self.assertEqual(record["Details"]["source_id"], "test")
            self.assertEqual(record["Details"]["camera_id"], "downward")
        self.assertEqual(self.records[-1]["Details"]["last_observation"],
                         observation.to_dict())

    def test_real_universal_log_writes_observations(self):
        # Use the real shared class with its normal FileHandler output, directed
        # to a temporary file so the test does not leave logs in the repository.
        logger = logging.getLogger("universal_log")
        original_handlers = logger.handlers[:]
        original_level, original_propagate = logger.level, logger.propagate
        original_console = getattr(logger, "_universal_console_handler", None)
        logger.handlers = []
        logger._universal_console_handler = None
        path = Path(self.temp.name) / "cv.log"
        handler = logging.FileHandler(path, encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(handler)
        try:
            cv = self.start(log_sink=log_entry, state="TRACKING",
                            timestamp_mapper=lambda timestamp: timestamp + 1_000)
            observations = [cv.read() for _ in range(3)]
            cv.stop_camera()
            handler.flush()
            lines = path.read_text(encoding="utf-8").splitlines()
        finally:
            handler.close()
            logger.handlers = original_handlers
            logger.setLevel(original_level)
            logger.propagate = original_propagate
            logger._universal_console_handler = original_console
        self.assertEqual(len(lines), 5)
        details = [json.loads(line.split("TRACKING: ", 1)[1]) for line in lines]
        for record, observation in zip(details[1:4], observations):
            self.assertEqual(record, {
                "module": "CV", "event": "observation", **observation.to_dict(),
            })
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

    def test_scheduled_15hz_samples_in_10hz_loop(self):
        self.scenario["fps"] = 15
        cv = self.start(timestamp_mapper=lambda t: t)
        observations = [cv.read_due(t) for t in (0, 100_000_000, 200_000_000)]
        self.assertEqual([o.frame_index for o in observations], [0, 1, 3])
        self.assertEqual(observations[1].timestamp_ns, 66_666_667)
        records = [r for r in self.records if r["Details"]["event"] == "observation"]
        self.assertEqual([r["Details"]["frame_index"] for r in records], [0, 1, 2, 3])

    def test_cached_sample_can_be_stale_without_refreshing_timestamp(self):
        self.scenario["fps"] = 1
        cv = self.start(timestamp_mapper=lambda t: t)
        first = cv.read_due(0)
        count = len(self.records)
        self.assertIs(cv.read_due(600_000_000), first)
        self.assertEqual(first.to_bird_snapshot()["timestamp_ns"], 0)
        self.assertEqual(len(self.records), count)

    def test_offset_wait_and_snapshot_distinguish_absence_from_missing(self):
        cv = self.start(timestamp_mapper=lambda t: t + 1_000_000_000)
        self.assertIsNone(cv.read_due(0))
        present = cv.read_due(1_000_000_000).to_bird_snapshot()
        self.assertEqual(present["timestamp_ns"], 1_000_000_000)
        self.assertEqual(present["source_timestamp_ns"], 0)
        self.assertEqual(present["source_id"], "test")
        absent = cv.read_due(1_100_000_000).to_bird_snapshot()
        missing = cv.read_due(1_200_000_000).to_bird_snapshot()
        self.assertIs(absent["detected"], False)
        self.assertIs(absent["valid"], True)
        self.assertIsNone(missing["detected"])
        self.assertIs(missing["valid"], False)
        self.assertEqual(missing["reason"], "missing_frame")
        present["detected"] = False
        self.assertIsNone(cv.read_due(1_200_000_000).detected)

    def test_scheduled_exhaustion_is_stable_and_restart_is_explicit(self):
        self.scenario["loop"] = False
        cv = self.start(timestamp_mapper=lambda t: t)
        self.assertEqual(cv.read_due(200_000_000).frame_index, 2)
        for now in (300_000_000, 400_000_000):
            with self.assertRaises(StopIteration):
                cv.read_due(now)
        self.assertTrue(cv.exhausted)
        self.assertIsNone(cv.frame)
        events = [r["Details"]["event"] for r in self.records]
        self.assertEqual(events.count("source_exhausted"), 1)
        cv.start_camera(self.path)
        self.assertFalse(cv.exhausted)
        self.assertEqual(cv.read_due(0).frame_index, 0)

    def test_scheduling_rejects_bad_clock_and_mixed_read_modes(self):
        cv = self.start(timestamp_mapper=lambda t: t)
        for invalid in (True, -1, 0.5, "0"):
            with self.assertRaises(ValueError):
                cv.read_due(invalid)
        cv.read_due(100_000_000)
        with self.assertRaises(ValueError):
            cv.read_due(0)
        with self.assertRaises(RuntimeError):
            cv.read()
        cv.stop_camera()
        cv.start_camera(self.path)
        cv.read()
        with self.assertRaises(RuntimeError):
            cv.read_due(0)

    def test_mapping_required_for_snapshot_and_scheduling(self):
        cv = self.start()
        with self.assertRaises(ValueError):
            cv.read_due(0)
        with self.assertRaises(ValueError):
            cv.read().to_bird_snapshot()

    def test_non_increasing_mapping_is_rejected(self):
        cv = self.start(timestamp_mapper=lambda t: 0)
        with self.assertRaises(ValueError):
            cv.read_due(0)

    def test_fixture_trace_is_reproducible_and_covers_absence_and_failure(self):
        path = Path(__file__).resolve().parents[1] / "cv/scenarios/integration_cycle.json"
        def replay():
            records = []
            cv = ComputerVision(timestamp_mapper=lambda t: t, log_sink=records.append)
            cv.start_camera(path)
            trace = [cv.read_due(t * 100_000_000).to_bird_snapshot() for t in range(130)]
            with self.assertRaises(StopIteration):
                cv.read_due(13_000_000_000)
            return trace, records
        first = replay()
        self.assertEqual(first, replay())
        trace = first[0]
        self.assertIs(trace[10]["detected"], True)
        self.assertIs(trace[50]["detected"], False)
        self.assertIs(trace[60]["detected"], True)
        self.assertTrue(all(row["detected"] is False for row in trace[80:120]))
        self.assertIsNone(trace[120]["detected"])

    def test_malformed_boolean_fixture_is_rejected_at_start(self):
        self.scenario["segments"][0]["camera_ok"] = "true"
        with self.assertRaises(ValueError):
            self.start()


if __name__ == "__main__":
    unittest.main()
