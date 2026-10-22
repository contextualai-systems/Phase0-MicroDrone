import unittest

from docking.event_logger import DockingEventLogger, DOCKING_EVENTS


class TestDockingEventLogger(unittest.TestCase):
    def test_logs_expected_docking_events(self):
        logger = DockingEventLogger()

        for event_name in [
            "alignment_start",
            "alignment_complete",
            "descent_start",
            "landing",
        ]:
            logger.record(event_name)

        self.assertEqual(
            [event.name for event in logger.events],
            [
                "alignment_start",
                "alignment_complete",
                "descent_start",
                "landing",
            ],
        )

    def test_rejects_unknown_docking_event(self):
        logger = DockingEventLogger()

        with self.assertRaises(ValueError):
            logger.record("takeoff")

    def test_known_event_names_are_exposed(self):
        self.assertEqual(
            DOCKING_EVENTS,
            (
                "alignment_start",
                "alignment_complete",
                "descent_start",
                "landing",
            ),
        )


if __name__ == "__main__":
    unittest.main()
