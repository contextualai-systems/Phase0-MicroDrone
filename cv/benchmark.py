"""Generate or replay a synthetic CV benchmark: python -m cv.benchmark --help."""

import argparse
from collections import defaultdict
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from time import perf_counter_ns

import cv2
import numpy as np

from .benchmark_dataset import generate_dataset
from .benchmark_metrics import evaluate_frame, reliable_range, summarize
from .camera_providers import ImageSequenceProvider
from .perception_pipeline import PerceptionPipeline
from .perception_types import CameraCalibration, FrameSample


class SavedImageProvider:
    """Only image paths and acquisition metadata cross the perception boundary."""

    def __init__(self, root, frames, calibration_id, source_id, fps):
        self.root = Path(root).resolve()
        self.frames = iter((f["frame_index"], f["image"], f["sha256"]) for f in frames)
        self.calibration_id, self.source_id, self.fps = calibration_id, source_id, fps
        self.closed = False

    def read(self):
        if self.closed:
            raise StopIteration
        index, relative, expected_hash = next(self.frames)
        path = (self.root / relative).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("Dataset image path escapes dataset directory")
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != expected_hash:
            raise ValueError(f"Dataset image hash mismatch: {relative}")
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise OSError(f"Unreadable dataset image: {relative}")
        return FrameSample(image, round(index * 1e9 / self.fps), index, "benchmark_camera",
                           self.source_id, calibration_id=self.calibration_id)

    def close(self):
        self.closed = True


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")


def report_markdown(report):
    lines = ["# Synthetic CV benchmark", "",
             "Camera-relative pose; fully synthetic images. No tracking or ENU claims.", "",
             "| Condition | Frames | TP | FP | FN | Detection rate | Pose coverage | Z RMSE (m) | Total p95 (ms) |",
             "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    def fmt(value):
        return "n/a" if value is None else f"{value:.4f}"
    for name, result in report["conditions"].items():
        lines.append(f"| {name} | {result['frames']} | {result['tp']} | {result['fp']} | {result['fn']} | "
                     f"{fmt(result['detection_rate'])} | {fmt(result['pose_coverage_all_truth'])} | "
                     f"{fmt(result['xyz_error_m']['z']['rmse'])} | "
                     f"{fmt(result['latency']['total_processing']['p95_ms'])} |")
    lines += ["", f"Range assessment: `{json.dumps(report['range'])}`", "",
              "Full XYZ, Euler/angular errors, counts, Wilson intervals, and stage p50/p95/p99 are in report.json.",
              "Latency includes disk read, SHA-256 verification, and PNG decoding under acquisition.",
              "Publication measures a local no-op callback; it does not measure ROS/network transport.",
              "Logging measures JSON serialization and buffered file write, not fsync/durable storage.",
              "Timing varies by machine and load. Accuracy replay requires the same software/configuration.",
              "All present markers remain eligible, including heavily occluded ones.",
              "Euler errors use ZYX; poses within 0.1 degree of pitch singularity are excluded only from Euler metrics.",
              "Range uses a point-estimate detection threshold, not an uncertainty-bound or pose-accuracy guarantee."]
    return "\n".join(lines) + "\n"


def run_benchmark(output, *, dataset=None, seed=2026, samples=10, warmup=3, tolerance_px=8.):
    if type(warmup) is not int or warmup < 0 or not np.isfinite(tolerance_px) or tolerance_px <= 0:
        raise ValueError("Warmup must be nonnegative and matching tolerance positive finite")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    if dataset is None:
        dataset = output / "dataset"
        generate_dataset(dataset, seed=seed, samples=samples)
    dataset = Path(dataset).resolve()
    manifest_bytes = (dataset / "manifest.json").read_bytes()
    manifest = json.loads(manifest_bytes)
    if manifest["schema_version"] != 1 or not manifest["frames"]:
        raise ValueError("Expected nonempty dataset manifest schema 1")
    calibration = CameraCalibration(**manifest["calibration"])
    manifest_hash = hashlib.sha256(manifest_bytes).hexdigest()
    try:
        revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=Path(__file__).parent,
                                           text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    source_files = ["benchmark.py", "benchmark_dataset.py", "benchmark_metrics.py", "camera_providers.py",
                    "perception_pipeline.py", "perception_types.py", "computer_vision.py"]
    metadata = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                "dataset": str(dataset), "manifest_sha256": manifest_hash, "seed": manifest["seed"],
                "python": sys.version, "opencv": cv2.__version__, "numpy": np.__version__,
                "platform": platform.platform(), "machine": platform.machine(), "processor": platform.processor(),
                "opencv_threads": cv2.getNumThreads(), "git_revision": revision,
                "source_sha256": {name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
                                  for name in source_files},
                "calibration": asdict(calibration), "dictionary_id": manifest["dictionary_id"],
                "corner_refinement": "CORNER_REFINE_SUBPIX", "pose_solver": "SOLVEPNP_ITERATIVE",
                "max_reprojection_error_px": 3., "matching_corner_rms_px": tolerance_px,
                "warmup_frames_per_condition": warmup, "fps": manifest["fps"],
                "eligibility": manifest["eligibility"], "publication": "local no-op callback"}
    write_json(output / "run.json", metadata)
    (output / "opencv_build.txt").write_text(cv2.getBuildInformation(), encoding="utf-8")
    grouped = defaultdict(list)
    for frame in manifest["frames"]:
        grouped[frame["condition"]].append(frame)
    rows, summaries, specs = [], {}, {}
    with (output / "observations.jsonl").open("w", encoding="utf-8") as log, \
            (output / "evaluation.jsonl").open("w", encoding="utf-8") as evaluation:
        logging_ns = 0
        def sink(record):
            nonlocal logging_ns
            start = perf_counter_ns()
            log.write(json.dumps(record, allow_nan=False) + "\n")
            logging_ns = perf_counter_ns() - start
        for name, frames in grouped.items():
            config = frames[0]["config"]
            specs[name] = config
            kwargs = dict(calibration=calibration, marker_side_m=config["marker_side_m"],
                          dictionary_id=manifest["dictionary_id"], timestamp_mapper=lambda ns: ns,
                          publisher=lambda observation: None)
            # Discard warm-up records and timings; evaluation frames are never discarded.
            first = SavedImageProvider(dataset, frames[:1], calibration.calibration_id, manifest_hash, manifest["fps"]).read()
            with PerceptionPipeline(ImageSequenceProvider([first.image] * warmup,
                                    calibration_id=calibration.calibration_id), log_sink=lambda _: None, **kwargs) as pipeline:
                for _ in range(warmup):
                    pipeline.read()
            provider = SavedImageProvider(dataset, frames, calibration.calibration_id, manifest_hash, manifest["fps"])
            condition_rows = []
            with PerceptionPipeline(provider, log_sink=sink, **kwargs) as pipeline:
                for frame in frames:
                    start = perf_counter_ns()
                    observation = pipeline.read()
                    elapsed = perf_counter_ns() - start
                    result = evaluate_frame(frame["truth"], observation.detections, tolerance_px=tolerance_px)
                    result.update(frame_index=frame["frame_index"], condition=name,
                                  processing_status=observation.processing_status,
                                  timings_ns={**dict(observation.timings_ns), "logging": logging_ns,
                                              "read_including_logging": elapsed})
                    evaluation.write(json.dumps(result, allow_nan=False) + "\n")
                    condition_rows.append(result)
            rows.extend(condition_rows)
            summaries[name] = summarize(condition_rows)
    report = {"schema_version": 1, "run": metadata, "overall": summarize(rows),
              "conditions": summaries, "range": reliable_range(summaries, specs)}
    write_json(output / "report.json", report)
    (output / "report.md").write_text(report_markdown(report), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing directories are refused")
    parser.add_argument("--dataset", type=Path, help="Replay existing dataset/manifest.json and saved PNGs")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--samples", type=int, default=10, help="Frames per condition; at least 100 for range assessment")
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--tolerance-px", type=float, default=8.)
    args = parser.parse_args()
    try:
        report = run_benchmark(args.output, dataset=args.dataset, seed=args.seed, samples=args.samples,
                               warmup=args.warmup, tolerance_px=args.tolerance_px)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f"Benchmark failed: {exc}\n")
    print(report_markdown(report))
    print(f"Saved report and replay metadata to {args.output}")


if __name__ == "__main__":
    main()
