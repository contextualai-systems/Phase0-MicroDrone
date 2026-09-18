"""Repeatable Phase 0 image demo; requires Python, NumPy, and OpenCV.

Run: python synthetic_frames.py
Without a display: python synthetic_frames.py --headless
Press q or Escape in the preview window to stop.

Each 120-frame cycle contains 90 target frames, 20 empty frames, and
10 missing frames. Pixels depend only on the frame index, not wall time.
Missing input is represented by None, never by a black camera image.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


WIDTH, HEIGHT = 640, 480
CYCLE_LENGTH = 120


def make_frame(frame_index: int) -> tuple[np.ndarray | None, str]:
    """Return a uint8 BGR frame (480, 640, 3), or None for missing input.

    Pixel coordinates originate at the top left: x rightward, y downward.
    """
    if frame_index < 0:
        raise ValueError("frame_index must be nonnegative")

    phase = frame_index % CYCLE_LENGTH
    if phase >= 110:
        return None, "missing"

    frame = np.full((HEIGHT, WIDTH, 3), 30, dtype=np.uint8)
    if phase >= 90:
        return frame, "empty"

    # Slice endpoints are exclusive, giving an exact 80 x 60 pixel target.
    x = 20 + phase * 5
    y = 80 + phase * 3
    frame[y : y + 60, x : x + 80] = (0, 200, 255)
    return frame, "target"


def save_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise OSError(f"Could not save image: {path}")


def make_observation(index: int, fps: float, camera_id: str) -> dict:
    """Describe fixture ground truth, not a detection inferred from pixels.

    Time starts at zero on the fixture clock. Missing samples retain their
    scheduled timestamp. Consumers must use this same clock for freshness.
    """
    phase = index % CYCLE_LENGTH
    missing = phase >= 110
    return {
        "schema_version": 1,
        "timestamp_ns": round(index * 1_000_000_000 / fps),
        "clock": "fixture",
        "source_id": "moving_rectangle_v1",
        "frame_index": index,
        "camera_id": camera_id,
        "image_width": None if missing else WIDTH,
        "image_height": None if missing else HEIGHT,
        "synthetic": True,
        "observation_method": "fixture_stub",
        "valid": not missing,
        "reason": "missing_frame" if missing else None,
        "detections": ([{
            "label": "synthetic_target",
            "bbox_xywh": [20 + phase * 5, 80 + phase * 3, 80, 60],
        }] if phase < 90 else []),
        "saved_images": {},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Disable preview windows")
    parser.add_argument("--frames", type=int, default=120, help="Frame count (default: 120)")
    parser.add_argument("--fps", type=float, default=15, help="Playback rate (default: 15)")
    parser.add_argument("--camera-id", choices=("front", "downward"), default="front")
    parser.add_argument("--json-stdout", action="store_true", help="Stream JSON Lines to stdout")
    parser.add_argument("--save-all", action="store_true", help="Save every available frame pair")
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path(__file__).resolve().parent / "synthetic_output",
        help="Directory for PNGs and observations.jsonl (existing outputs are overwritten)",
    )
    args = parser.parse_args()
    if args.frames <= 0 or not np.isfinite(args.fps) or args.fps <= 0:
        parser.error("--frames and --fps must be positive, finite values")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Images and observations: {args.output_dir.resolve()}", file=sys.stderr)
    log = (args.output_dir / "observations.jsonl").open("w", encoding="utf-8")
    previous_state = None
    try:
        for index in range(args.frames):
            started = time.monotonic()
            frame, state = make_frame(index)
            observation = make_observation(index, args.fps, args.camera_id)
            if state != previous_state:
                print(f"Frame {index:04d}: {state}", file=sys.stderr)
                previous_state = state

            if frame is None:
                # This placeholder is preview UI only, not a sensor frame.
                preview = np.zeros((HEIGHT, WIDTH * 2, 3), dtype=np.uint8)
                cv2.putText(preview, "MISSING INPUT - no image to process", (30, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # Preprocessing preserves pixel coordinates and dimensions.
                # Output: uint8 grayscale, shape (480, 640).
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if args.save_all or index in (0, 45, 89, 90):
                    stem = f"{args.camera_id}_frame_{index:04d}_{observation['timestamp_ns']}ns_{state}"
                    save_image(args.output_dir / f"{stem}_bgr.png", frame)
                    save_image(args.output_dir / f"{stem}_gray.png", gray)
                    observation["saved_images"] = {
                        "bgr": f"{stem}_bgr.png", "gray": f"{stem}_gray.png",
                    }

                preview = np.hstack((frame, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)))
                # Annotate only the preview; saved fixtures stay unmodified.
                cv2.putText(preview, f"{index:04d} {state} | BGR left, grayscale right",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            line = json.dumps(observation, separators=(",", ":"), allow_nan=False)
            log.write(line + "\n")
            log.flush()
            if args.json_stdout:
                print(line, flush=True)

            if not args.headless:
                cv2.imshow("Synthetic camera demo", preview)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                remaining = 1 / args.fps - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        log.close()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
