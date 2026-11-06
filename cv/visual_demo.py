"""Run real pixel detection and save a visual comparison with scene truth.

Run from the repository root: python -m cv.visual_demo
No GUI/display server required; outputs go to the ignored synthetic_output folder.
"""

import json
from pathlib import Path

import cv2
import numpy as np

from .camera_providers import ImageSequenceProvider, render_marker
from .perception_pipeline import PerceptionPipeline
from .perception_types import CameraCalibration


def main():
    output = Path(__file__).parent / "synthetic_output" / "visual_demo"
    output.mkdir(parents=True, exist_ok=True)
    calibration = CameraCalibration("demo_camera", 640, 480,
                                     ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)))
    scenes = [
        ("Centered marker", (0., 0., 1.), (np.pi, 0., 0.)),
        ("Tilted and offset marker", (.08, -.04, 1.2), (2.9, .2, .1)),
        ("Empty camera frame", None, None),
    ]
    images = [render_marker(calibration, tvec=t, rvec=r) if t is not None
              else np.full((480, 640, 3), 255, np.uint8) for _, t, r in scenes]
    provider = ImageSequenceProvider(images, calibration_id=calibration.calibration_id)
    records, panels, results = [], [], []
    with PerceptionPipeline(provider, calibration=calibration, marker_side_m=.2,
                            timestamp_mapper=lambda ns: ns, log_sink=records.append) as pipeline:
        for (title, truth, _), original in zip(scenes, images):
            observation = pipeline.read()
            annotated = original.copy()
            for detection in observation.detections:
                corners = np.array(detection.corners_px, dtype=np.int32)
                cv2.polylines(annotated, [corners], True, (0, 150, 0), 2)
                for index, corner in enumerate(corners):
                    cv2.circle(annotated, tuple(corner), 4, (0, 0, 220), -1)
                    cv2.putText(annotated, str(index), tuple(corner + [5, -7]),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 180), 1)
            panel = np.full((640, 640, 3), 245, np.uint8)
            panel[45:525] = annotated
            lines = [title, f"Result: {observation.status} | {len(observation.detections)} marker(s)"]
            if truth is not None:
                detection = observation.detections[0] if observation.detections else None
                lines.append("True XYZ (m):     " + ", ".join(f"{v:.4f}" for v in truth))
                if detection and detection.pose_valid:
                    estimate = (detection.position_x, detection.position_y, detection.position_z)
                    lines.append("Measured XYZ (m): " + ", ".join(f"{v:.4f}" for v in estimate))
                    lines.append(f"ID: {detection.object_id} | reprojection: {detection.reprojection_error_px:.3f} px")
                else:
                    lines.append("Pose unavailable")
            else:
                lines.append("Valid image with no detection; not a camera failure")
            for index, line in enumerate(lines):
                y = 29 if index == 0 else 540 + (index - 1) * 25
                cv2.putText(panel, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                            .62 if index == 0 else .51, (30, 30, 30), 1, cv2.LINE_AA)
            panels.append(panel)
            results.append({"scene": title, "truth_position_m": truth,
                            "observation": observation.to_dict()})
            print("\n" + "\n".join(lines))
    image_path = output / "comparison.png"
    if not cv2.imwrite(str(image_path), np.hstack(panels)):
        raise OSError(f"Failed to write {image_path}")
    (output / "results.json").write_text(json.dumps(results, indent=2, allow_nan=False), encoding="utf-8")
    (output / "observations.jsonl").write_text(
        "".join(json.dumps(record, allow_nan=False) + "\n" for record in records), encoding="utf-8")
    print(f"\nVisual comparison: {image_path}\nRaw results: {output / 'results.json'}")


if __name__ == "__main__":
    main()
