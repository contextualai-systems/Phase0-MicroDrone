"""Repeatable synthetic camera stream, not image-based detection. See --help."""

import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import uuid

import cv2
import numpy as np

WIDTH, HEIGHT = 640, 480
DEFAULT_SCENARIO = Path(__file__).parent / "scenarios" / "bird_demo.json"


def validate_scenario(data: dict) -> dict:
    """Reject ambiguous inputs before emitting frames or observations."""
    if not isinstance(data, dict):
        raise ValueError("Scenario must be an object")
    if not isinstance(data.get("scenario_id"), str) or not data["scenario_id"]:
        raise ValueError("scenario_id must be a nonempty string")
    fps = data.get("fps")
    if type(fps) not in (int, float) or not math.isfinite(fps) or not 0 < fps <= 1000:
        raise ValueError("fps must be finite and between 0 and 1000")
    if type(data.get("loop")) is not bool:
        raise ValueError("loop must be boolean")
    if data.get("appearance") not in ("bird", "rectangle"):
        raise ValueError("appearance must be bird or rectangle")
    segments = data.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("segments must be a nonempty list")
    for segment in segments:
        if not isinstance(segment, dict):
            raise ValueError("Each segment must be an object")
        if type(segment.get("frames")) is not int or segment["frames"] <= 0:
            raise ValueError("Segment frames must be a positive integer")
        if type(segment.get("camera_ok")) is not bool:
            raise ValueError("camera_ok must be boolean")
        if "bird" not in segment:
            raise ValueError("Each segment requires bird (null for missing input)")
        bird = segment["bird"]
        if not segment["camera_ok"]:
            if bird is not None:
                raise ValueError("Missing camera input requires bird: null")
            continue
        if not isinstance(bird, dict) or type(bird.get("detected")) is not bool:
            raise ValueError("Available frames require a bird observation")
        if "type" not in bird or "distance_m" not in bird:
            raise ValueError("bird requires type and distance_m")
        if bird["detected"]:
            distance = bird["distance_m"]
            if not isinstance(bird["type"], str) or not bird["type"]:
                raise ValueError("Detected bird requires a fixture type")
            if type(distance) not in (int, float) or not math.isfinite(distance) or distance < 0:
                raise ValueError("distance_m must be finite and nonnegative")
        elif bird["type"] is not None or bird["distance_m"] is not None:
            raise ValueError("No detection requires null type and distance")
    return data


class SyntheticSource:
    """Random-access source: the caller owns scheduling and module integration."""

    def __init__(self, scenario: dict, camera_id: str = "front", image_path: Path | None = None):
        self.scenario = copy.deepcopy(validate_scenario(scenario))
        if camera_id not in ("front", "downward"):
            raise ValueError("camera_id must be front or downward")
        self.camera_id = camera_id
        self.cycle_length = sum(s["frames"] for s in self.scenario["segments"])
        self.sprite = None
        if image_path is not None:
            sprite = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
            if sprite is None:
                raise ValueError(f"Cannot read bird image: {image_path}")
            if sprite.ndim == 2:
                sprite = cv2.cvtColor(sprite, cv2.COLOR_GRAY2BGR)
            if sprite.dtype != np.uint8 or sprite.shape[2] not in (3, 4):
                raise ValueError("Bird image must be 8-bit grayscale, BGR or BGRA")
            scale = min(100 / sprite.shape[1], 70 / sprite.shape[0])
            self.sprite = cv2.resize(sprite, (max(1, round(sprite.shape[1] * scale)),
                                              max(1, round(sprite.shape[0] * scale))))

    def sample(self, index: int) -> tuple[np.ndarray | None, dict]:
        """Return frame and schema-v2 observation; timestamps never wrap."""
        if type(index) is not int or index < 0:
            raise ValueError("index must be a nonnegative integer")
        if not self.scenario["loop"] and index >= self.cycle_length:
            raise StopIteration
        phase = index % self.cycle_length
        local = phase
        for segment_id, segment in enumerate(self.scenario["segments"]):
            if local < segment["frames"]:
                break
            local -= segment["frames"]
        bird = copy.deepcopy(segment["bird"])
        valid = segment["camera_ok"]
        frame = np.full((HEIGHT, WIDTH, 3), (220, 210, 190), np.uint8) if valid else None
        detections = []
        if valid and bird["detected"]:
            w, h = (100, 70) if self.sprite is None else (self.sprite.shape[1], self.sprite.shape[0])
            x, y = 20 + (local * 5) % (WIDTH - w - 40), 160
            if self.sprite is not None:
                roi = frame[y:y+h, x:x+w]
                if self.sprite.shape[2] == 4:
                    alpha = self.sprite[:, :, 3:4].astype(float) / 255
                    roi[:] = np.rint(self.sprite[:, :, :3] * alpha + roi * (1-alpha)).astype(np.uint8)
                else:
                    roi[:] = self.sprite
            elif self.scenario["appearance"] == "rectangle":
                frame[y:y+h, x:x+w] = (0, 200, 255)
            else:
                # Original code-drawn bird silhouette; no external image dependency.
                points = np.array([(0, 3), (38, 22), (48, 38), (62, 27), (88, 6),
                                   (74, 40), (88, 42), (99, 48), (83, 51),
                                   (68, 62), (43, 58), (13, 69), (27, 48)], np.int32)
                cv2.fillPoly(frame, [points + (x, y)], (45, 45, 45))
            detections = [{"label": bird["type"], "bbox_xywh": [x, y, w, h], "confidence": None}]
        observation = {
            "schema_version": 2, "timestamp_ns": round(index * 1_000_000_000 / self.scenario["fps"]),
            "clock": "fixture", "source_id": self.scenario["scenario_id"], "frame_index": index,
            "cycle_index": index // self.cycle_length, "scenario_frame": phase, "segment_index": segment_id,
            "camera_id": self.camera_id, "image_width": WIDTH if valid else None,
            "image_height": HEIGHT if valid else None, "synthetic": True,
            "observation_method": "fixture_stub", "valid": valid,
            "reason": None if valid else "missing_frame", "camera_ok": valid,
            "bird": bird, "detections": detections, "saved_images": {},
        }
        return frame, observation


def make_frame(frame_index: int) -> tuple[np.ndarray | None, str]:
    """Convenience API using the default scenario; prefer a reused SyntheticSource."""
    source = SyntheticSource(json.loads(DEFAULT_SCENARIO.read_text(encoding="utf-8")))
    frame, obs = source.sample(frame_index)
    return frame, "missing" if not obs["valid"] else "target" if obs["detections"] else "empty"


def make_observation(index: int, fps: float, camera_id: str) -> dict:
    """Convenience API for callers of the previous script; output is now v2."""
    scenario = json.loads(DEFAULT_SCENARIO.read_text(encoding="utf-8"))
    scenario["fps"] = fps
    return SyntheticSource(scenario, camera_id).sample(index)[1]


def save_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise OSError(f"Could not save image: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument("--bird-image", type=Path, help="Optional local PNG/JPEG; PNG alpha is supported")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fast", action="store_true", help="Unpaced generation; requires --frames or --once")
    parser.add_argument("--once", action="store_true", help="Stop after one scenario cycle")
    parser.add_argument("--frames", type=int, help="Maximum frames; otherwise follow scenario loop setting")
    parser.add_argument("--fps", type=float, help="Override scenario frame rate")
    parser.add_argument("--camera-id", choices=("front", "downward"), default="front")
    parser.add_argument("--json-stdout", action="store_true")
    parser.add_argument("--save-all", action="store_true", help="Save every available frame; uses substantial disk space")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "synthetic_output")
    args = parser.parse_args()
    try:
        scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
        if args.fps is not None:
            if not isinstance(scenario, dict):
                raise ValueError("Scenario must be an object")
            scenario["fps"] = args.fps
        source = SyntheticSource(scenario, args.camera_id, args.bird_image)
        if args.frames is not None and args.frames <= 0:
            raise ValueError("--frames must be positive")
        if args.fast and args.frames is None and not args.once:
            raise ValueError("--fast requires --frames or --once")
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    limit = args.frames
    if args.once or not scenario["loop"]:
        limit = min(limit, source.cycle_length) if limit is not None else source.cycle_length
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_dir = args.output_dir / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8])
    run_dir.mkdir()
    (run_dir / "scenario.json").write_text(json.dumps(source.scenario, indent=2) + "\n", encoding="utf-8")
    if source.sprite is not None:
        save_image(run_dir / "bird_fixture.png", source.sprite)
    print(f"Run output: {run_dir.resolve()} (Ctrl+C, q or Escape to stop)", file=sys.stderr)
    index, previous_segment = 0, None
    try:
        with (run_dir / "observations.jsonl").open("w", encoding="utf-8") as log:
            while limit is None or index < limit:
                started = time.monotonic()
                frame, obs = source.sample(index)
                changed = obs["segment_index"] != previous_segment
                if changed:
                    print(f"Frame {index}: segment {obs['segment_index']}, camera_ok={obs['camera_ok']}", file=sys.stderr)
                    previous_segment = obs["segment_index"]
                gray = None if frame is None else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if frame is not None and (args.save_all or (obs["cycle_index"] == 0 and changed)):
                    stem = f"{args.camera_id}_{index:06d}_{obs['timestamp_ns']}ns"
                    for name, pixels in (("bgr", frame), ("gray", gray)):
                        filename = f"{stem}_{name}.png"
                        save_image(run_dir / filename, pixels)
                        obs["saved_images"][name] = filename
                line = json.dumps(obs, separators=(",", ":"), allow_nan=False)
                log.write(line + "\n")
                log.flush()
                if args.json_stdout:
                    print(line, flush=True)
                if not args.headless:
                    preview = np.zeros((HEIGHT, WIDTH*2, 3), np.uint8) if frame is None else np.hstack((frame, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)))
                    caption = f"{index} {args.camera_id}: " + ("MISSING INPUT" if frame is None else f"bird={obs['bird']['detected']} | BGR / gray")
                    cv2.putText(preview, caption, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
                    cv2.imshow("Synthetic camera", preview)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                index += 1
                if not args.fast:
                    time.sleep(max(0, 1/source.scenario["fps"] - (time.monotonic()-started)))
    except (KeyboardInterrupt, BrokenPipeError):
        print("Stopped synthetic camera stream.", file=sys.stderr)
    finally:
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
