"""Deterministic temporal scenes. Truth is evaluator-only, not provider input."""

import cv2
import numpy as np

from .camera_providers import render_marker
from .perception_types import CameraCalibration, marker_corners

CALIBRATION = CameraCalibration("sequence_camera", 640, 480,
                                ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)))
SIDE_M = .12
FPS = 10
SCENARIOS = ("motion", "short_occlusion", "long_occlusion", "missing_frames", "duplicate_id", "entry_exit",
             "rotation_tilt")


def make_sequence(name, seed=2026):
    if name not in SCENARIOS:
        raise ValueError(f"Unknown sequence: {name}")
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    rng = np.random.default_rng(seed)
    offsets = rng.uniform(-.015, .015, (3, 2))
    images, frames = [], []
    for index in range(36):
        image = np.full((480, 640, 3), 255, np.uint8)
        missing = name == "missing_frames" and 10 <= index <= 12
        objects = []
        for entity, marker_id, x, y, slot in (
            ("A", 17, -.22 + .04*np.sin(index/5), -.12, 0),
            ("B", 23, .2, .12 + .04*np.cos(index/6), 1),
            ("C", 17, .24, -.17, 2),
        ):
            if entity == "C" and not (name == "duplicate_id" and 10 <= index <= 12):
                continue
            if entity == "A" and name == "entry_exit" and (index < 5 or 16 <= index < 25):
                continue
            occluded = entity == "A" and (
                (name == "short_occlusion" and 10 <= index <= 11)
                or (name == "long_occlusion" and 10 <= index <= 17))
            position = np.array([x + offsets[slot, 0], y + offsets[slot, 1], 1.])
            rvec = np.array([np.pi, 0., 0.])
            if name == "rotation_tilt":
                # Independent in-plane rotation and out-of-plane tilt; marker
                # geometry and rasterization must use the same physical pose.
                phase = index / 35 * 2 * np.pi + slot * np.pi / 2
                rz, _ = cv2.Rodrigues(np.array([0., 0., .6*np.sin(phase)]))
                ry, _ = cv2.Rodrigues(np.array([0., .6*np.cos(phase), 0.]))
                rvec, _ = cv2.Rodrigues(rz @ ry @ np.diag([1., -1., -1.]))
            corners, _ = cv2.projectPoints(marker_corners(SIDE_M), rvec, position,
                                          np.array(CALIBRATION.matrix), np.zeros(5))
            corners = corners.reshape(4, 2)
            marker = render_marker(CALIBRATION, marker_id=marker_id, side_m=SIDE_M,
                                   rvec=rvec, tvec=position)
            mask = np.zeros((480, 640), np.uint8)
            cv2.fillConvexPoly(mask, np.rint(corners).astype(np.int32), 255)
            image[mask > 0] = marker[mask > 0]
            if occluded:
                # Fully opaque cover; present physically but not observable.
                image[mask > 0] = 190
            objects.append({"entity_id": entity, "object_id": f"aruco:0:{marker_id}",
                            "observable": not (occluded or missing), "occluded": occluded,
                            "corners_px": corners.tolist(), "position_camera_m": position.tolist(),
                            "rotation_camera_matrix": cv2.Rodrigues(rvec)[0].tolist()})
        images.append(None if missing else image)
        frames.append({"frame_index": index, "timestamp_ns": index*100_000_000,
                       "missing_frame": missing, "objects": objects})
    return images, frames
