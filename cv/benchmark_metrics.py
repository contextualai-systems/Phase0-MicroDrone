"""Offline evaluator. No truth or evaluation policy enters the CV pipeline."""

import math

import numpy as np


def ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def wilson(successes: int, count: int) -> list[float] | None:
    """95% Wilson interval for the sampled binomial detection proportion."""
    if not count:
        return None
    z = 1.959963984540054
    p = successes / count
    center = (p + z*z / (2*count)) / (1 + z*z / count)
    half = z * math.sqrt(p*(1-p)/count + z*z/(4*count*count)) / (1 + z*z/count)
    return [max(0., center-half), min(1., center+half)]


def error_summary(values) -> dict:
    a = np.array(values, dtype=float)
    if not len(a):
        return {"count": 0, "bias": None, "mae": None, "rmse": None, "p95_absolute": None}
    return {"count": len(a), "bias": float(a.mean()), "mae": float(np.abs(a).mean()),
            "rmse": float(np.sqrt(np.mean(a*a))), "p95_absolute": float(np.percentile(np.abs(a), 95))}


def quaternion_matrix(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1., atol=1e-5):
        raise ValueError("Pose orientation must be a finite unit quaternion xyzw")
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def euler_degrees(rotation):
    """ZYX yaw/pitch/roll; omit all components within 0.1 deg of gimbal lock."""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    if abs(math.cos(pitch)) < math.sin(math.radians(.1)):
        return None
    return np.degrees([math.atan2(rotation[1, 0], rotation[0, 0]), pitch,
                       math.atan2(rotation[2, 1], rotation[2, 2])])


def evaluate_frame(truth: list[dict], detections, *, tolerance_px: float = 8.) -> dict:
    """Same-ID, ordered-corner RMS gate. One-to-one, closest pair first.

    Generator uses unique IDs per frame. Duplicate or wrong-ID predictions are
    FP; unmatched truth is FN. Detector failure is passed as no predictions.
    """
    if not math.isfinite(tolerance_px) or tolerance_px <= 0:
        raise ValueError("Matching tolerance must be positive and finite")
    detections = tuple(detections or ())
    candidates = []
    for i, target in enumerate(truth):
        for j, detection in enumerate(detections):
            if detection.object_id != target["object_id"]:
                continue
            corners = np.asarray(detection.corners_px)
            if corners.shape != (4, 2) or not np.isfinite(corners).all():
                continue
            error = float(np.sqrt(np.mean(np.sum((corners - target["corners_px"])**2, axis=1))))
            if error <= tolerance_px:
                candidates.append((error, i, j))
    used_truth, used_detection, pairs = set(), set(), []
    for error, i, j in sorted(candidates):
        if i not in used_truth and j not in used_detection:
            used_truth.add(i)
            used_detection.add(j)
            pairs.append((i, j))
    poses = []
    invalid_poses = 0
    for i, j in pairs:
        detection, target = detections[j], truth[i]
        if not detection.pose_valid:
            continue
        try:
            position = np.array([detection.position_x, detection.position_y, detection.position_z], dtype=float)
            rotation = quaternion_matrix(detection.orientation)
            if not np.isfinite(position).all():
                raise ValueError("Nonfinite pose")
        except (ValueError, TypeError):
            invalid_poses += 1
            continue
        expected = np.array(target["rotation_matrix"])
        angular = math.degrees(math.acos(float(np.clip((np.trace(rotation @ expected.T)-1)/2, -1, 1))))
        measured_euler, truth_euler = euler_degrees(rotation), euler_degrees(expected)
        euler = None if measured_euler is None or truth_euler is None else (
            (measured_euler - truth_euler + 180) % 360 - 180).tolist()
        poses.append({"xyz_error_m": (position - target["position_m"]).tolist(),
                      "angular_error_deg": angular, "yaw_pitch_roll_error_deg": euler})
    return {"tp": len(pairs), "fp": len(detections)-len(pairs), "fn": len(truth)-len(pairs),
            "truth_count": len(truth), "negative_frame": not truth,
            "poses": poses, "malformed_valid_poses": invalid_poses}


def summarize(rows: list[dict]) -> dict:
    tp, fp, fn = (sum(row[key] for row in rows) for key in ("tp", "fp", "fn"))
    poses = [pose for row in rows for pose in row["poses"]]
    negative_frames = sum(row["negative_frame"] for row in rows)
    negative_fp = sum(row["fp"] for row in rows if row["negative_frame"])
    eulers = [pose["yaw_pitch_roll_error_deg"] for pose in poses if pose["yaw_pitch_roll_error_deg"] is not None]
    timings = {}
    for stage in sorted({key for row in rows for key in row.get("timings_ns", {})}):
        values = [row["timings_ns"][stage] / 1e6 for row in rows if stage in row.get("timings_ns", {})]
        timings[stage] = {"count": len(values), **{
            f"p{percentile}_ms": float(np.percentile(values, percentile)) for percentile in (50, 95, 99)}}
    return {"frames": len(rows), "eligible_objects": tp+fn, "tp": tp, "fp": fp, "fn": fn,
            "detection_rate": ratio(tp, tp+fn), "detection_rate_ci95": wilson(tp, tp+fn),
            "precision": ratio(tp, tp+fp), "negative_frames": negative_frames,
            "negative_false_positives": negative_fp, "fp_per_negative_frame": ratio(negative_fp, negative_frames),
            "processing_failures": sum(row.get("processing_status") != "ok" for row in rows),
            "pose_count": len(poses), "pose_coverage_all_truth": ratio(len(poses), tp+fn),
            "pose_coverage_matched": ratio(len(poses), tp), "matched_without_usable_pose": tp-len(poses),
            "malformed_valid_poses": sum(row["malformed_valid_poses"] for row in rows),
            "xyz_error_m": {axis: error_summary([p["xyz_error_m"][i] for p in poses]) for i, axis in enumerate("xyz")},
            "rotation_error_deg": error_summary([p["angular_error_deg"] for p in poses]),
            "yaw_pitch_roll_error_deg": {axis: error_summary([p[i] for p in eulers])
                                          for i, axis in enumerate(("yaw", "pitch", "roll"))},
            "euler_excluded_poses": len(poses)-len(eulers), "latency": timings}


def reliable_range(groups: dict, specs: dict, *, min_samples=100, threshold=.95) -> dict:
    """Point-estimate threshold, contiguous from nearest tested distance; not hardware range."""
    distances = sorted((spec["distance_m"], name) for name, spec in specs.items() if spec["factor"] == "distance")
    farthest = None
    for distance, name in distances:
        metrics = groups[name]
        if metrics["eligible_objects"] < min_samples or (metrics["detection_rate"] or 0) < threshold:
            break
        farthest = distance
    return {"maximum_contiguous_tested_distance_m": farthest, "minimum_objects_per_distance": min_samples,
            "required_detection_rate": threshold, "criterion": "point estimate; see per-condition Wilson intervals",
            "scope": "clean synthetic distance sweep only; not docking-qualified or physical-camera range"}
