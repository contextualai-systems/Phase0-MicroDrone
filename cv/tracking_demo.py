"""Real pixel detections -> shared updates, including loss, expiry, reacquisition.

python -m cv.tracking_demo --output cv/synthetic_output/tracking_demo
The camera-to-ENU transform is an explicit synthetic fixture, not drone calibration.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from .camera_providers import ImageSequenceProvider, render_marker
from .perception_pipeline import PerceptionPipeline
from .perception_publisher import PerceptionPublisher
from .perception_types import CameraCalibration
from .shared_perception import CameraToENU


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("cv/synthetic_output/tracking_demo"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = CameraCalibration("demo", 640, 480, ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)))
    image = render_marker(calibration)
    empty = np.full_like(image, 255)
    images = [image, image, empty, None, image, empty, None, empty, empty, empty, image]
    provider = ImageSequenceProvider(images, calibration_id="demo", source_id="tracking_demo")
    updates = []
    with (args.output / "events.jsonl").open("w", encoding="utf-8") as events:
        def log(record):
            events.write(json.dumps(record, allow_nan=False) + "\n")
        publisher = PerceptionPublisher(session_id="tracking_demo", publisher=updates.append, log_sink=log)
        with PerceptionPipeline(provider, calibration=calibration, marker_side_m=.2,
                                timestamp_mapper=lambda ns: ns, log_sink=log) as pipeline:
            for index in range(len(images)):
                now = index * 100_000_000
                # Fixture: camera 2m above ENU origin, looking down, optical +X east.
                transform = CameraToENU("front", now, "demo_world", (0., 0., 2.),
                                        (1., 0., 0., 0.), "synthetic_downward_camera")
                publisher.update(pipeline.read(), now_ns=now, transform=transform)
                if index == 1:
                    publisher.update(now_ns=150_000_000)  # no new camera frame
    with (args.output / "updates.jsonl").open("w", encoding="utf-8") as out:
        for update in updates:
            out.write(json.dumps(update.to_dict(), allow_nan=False) + "\n")
            tracks = ", ".join(f"{t.tracking_id} {t.status}/{t.estimate_kind} last={t.last_observed_timestamp_ns}"
                               for t in update.tracks)
            print(f"{update.update_timestamp_ns:10d} ns | {update.processing_status:12s} | "
                  f"detections={len(update.detections)} | {tracks}")
    for name, index in (("measured", 1), ("heartbeat", 2), ("empty", 3), ("missing", 4), ("expired", 10)):
        (args.output / f"{name}.json").write_text(json.dumps(updates[index].to_dict(), indent=2, allow_nan=False), encoding="utf-8")
    print(f"Coordinator payloads: {args.output / 'updates.jsonl'}")


if __name__ == "__main__":
    main()
