"""Seeded calibrated images and separate evaluator-only ground truth."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from .camera_providers import render_marker
from .perception_types import CameraCalibration, marker_corners

GENERATOR_VERSION = 1
CALIBRATION = CameraCalibration("benchmark_640x480", 640, 480,
                                ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)))


def conditions() -> list[dict]:
    """Single-factor sweeps plus negatives and one combined stress condition."""
    items = [{"name": "clean", "factor": "baseline"}]
    for factor, values, key in (
        ("distance", (.5, 1., 2., 3., 4., 6.), "distance_m"),
        ("rotation", (0., 30., 60., 120.), "rotation_deg"),
        ("tilt", (20., 40., 60.), "tilt_deg"),
        ("lighting", (.25, .5, 1.), "brightness"),
        ("blur", (3, 9, 17), "blur_px"),
        ("occlusion", (.1, .25, .5), "occlusion_fraction"),
        ("scale", (.08, .12, .3), "marker_side_m"),
        ("clutter", (20, 80), "clutter_count"),
    ):
        items.extend({"name": f"{factor}_{value:g}", "factor": factor, key: value} for value in values)
    items += [{"name": "negative_clean", "factor": "negative", "present": False},
              {"name": "negative_clutter", "factor": "negative", "present": False, "clutter_count": 80},
              {"name": "combined", "factor": "combined", "brightness": .5,
               "blur_px": 9, "occlusion_fraction": .1, "clutter_count": 40, "tilt_deg": 30.}]
    return items


def render_case(config: dict, seed: int) -> tuple[np.ndarray, list[dict], dict]:
    """Truth never enters the provider or detector. All present objects are eligible.

    Scale varies physical side length at fixed distance. Occlusion covers a strip
    from the marker's left edge in marker coordinates; actual raster area covered
    is recorded. Clutter is drawn behind the marker, including its white cells.
    """
    rng = np.random.default_rng(seed)
    c = {"present": True, "distance_m": 1., "marker_side_m": .2,
         "rotation_deg": 0., "tilt_deg": 0., "brightness": 1.,
         "blur_px": 1, "occlusion_fraction": 0., "clutter_count": 0, **config}
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    for _ in range(c["clutter_count"]):
        x, y = int(rng.integers(0, 640)), int(rng.integers(0, 480))
        w, h = int(rng.integers(5, 80)), int(rng.integers(5, 80))
        color = tuple(int(v) for v in rng.integers(0, 220, 3))
        cv2.rectangle(image, (x, y), (x + w, y + h), color, -1)
    truth = []
    actual_occlusion = 0.
    if c["present"]:
        marker_id = int(rng.integers(0, 50))
        z = c["distance_m"]
        tvec = np.array([rng.uniform(-.025, .025) * z, rng.uniform(-.025, .025) * z, z])
        rz, _ = cv2.Rodrigues(np.array([0., 0., np.deg2rad(c["rotation_deg"])]))
        ry, _ = cv2.Rodrigues(np.array([0., np.deg2rad(c["tilt_deg"]), 0.]))
        rotation = rz @ ry @ np.diag([1., -1., -1.])
        rvec, _ = cv2.Rodrigues(rotation)
        marker = render_marker(CALIBRATION, marker_id=marker_id, side_m=c["marker_side_m"],
                               rvec=rvec, tvec=tvec)
        points, _ = cv2.projectPoints(marker_corners(c["marker_side_m"]), rvec, tvec,
                                     np.array(CALIBRATION.matrix), np.zeros(5))
        points = points.reshape(4, 2)
        if np.any(points < [2, 2]) or np.any(points > [637, 477]):
            raise ValueError("Benchmark marker must remain fully inside image")
        mask = np.zeros(image.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, np.rint(points).astype(np.int32), 255)
        image[mask > 0] = marker[mask > 0]
        fraction = c["occlusion_fraction"]
        if fraction:
            covered = np.zeros_like(mask)
            strip = [points[0], points[0] + fraction * (points[1] - points[0]),
                     points[3] + fraction * (points[2] - points[3]), points[3]]
            cv2.fillConvexPoly(covered, np.rint(strip).astype(np.int32), 255)
            actual_occlusion = float(np.count_nonzero((covered > 0) & (mask > 0)) / np.count_nonzero(mask))
            image[covered > 0] = 180
        truth = [{"object_id": f"aruco:0:{marker_id}", "corners_px": points.tolist(),
                  "position_m": tvec.tolist(), "rotation_matrix": rotation.tolist(),
                  "projected_width_px": float(np.linalg.norm(points[1] - points[0])),
                  "occluded_fraction": actual_occlusion}]
    image = np.clip(image.astype(float) * c["brightness"], 0, 255).astype(np.uint8)
    blur = c["blur_px"]
    if blur > 1:
        # Horizontal uniform motion PSF, in image pixels.
        image = cv2.filter2D(image, -1, np.ones((1, blur), dtype=float) / blur)
    return image, truth, c


def generate_dataset(directory: Path, *, seed: int = 2026, samples: int = 10) -> dict:
    if type(seed) is not int or seed < 0 or type(samples) is not int or samples < 1:
        raise ValueError("Seed must be nonnegative; samples must be a positive integer")
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "images").mkdir()
    manifest = {"schema_version": 1, "generator_version": GENERATOR_VERSION,
                "seed": seed, "samples_per_condition": samples, "calibration": asdict(CALIBRATION),
                "dictionary_id": 0, "fps": 10, "frames": [],
                "eligibility": "All present markers, including occluded ones; no exclusions",
                "versions": {"opencv": cv2.__version__, "numpy": np.__version__}}
    rng = np.random.default_rng(seed)
    for condition in conditions():
        for _ in range(samples):
            frame_seed = int(rng.integers(0, 2**32))
            image, truth, config = render_case(condition, frame_seed)
            index = len(manifest["frames"])
            relative = f"images/{index:06d}.png"
            path = directory / relative
            if not cv2.imwrite(str(path), image):
                raise OSError(f"Cannot write {path}")
            manifest["frames"].append({"frame_index": index, "seed": frame_seed,
                "image": relative, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "condition": condition["name"], "config": config, "truth": truth})
    (directory / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8")
    return manifest
