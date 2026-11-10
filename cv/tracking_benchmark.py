"""Multi-object sequence benchmark and browser playback, without GUI dependencies."""

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import platform
from time import perf_counter_ns

import cv2
import numpy as np

from .camera_providers import ImageSequenceProvider
from .perception_pipeline import PerceptionPipeline
from .perception_publisher import PerceptionPublisher
from .shared_perception import CameraToENU
from .tracking_metrics import evaluate_sequence
from .tracking_sequences import CALIBRATION, FPS, SCENARIOS, SIDE_M, make_sequence


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(f"Cannot write {path}")


def annotate(image, update, scenario):
    canvas = np.full((720, 640, 3), 245, np.uint8)
    canvas[:480] = image if image is not None else np.full((480, 640, 3), 70, np.uint8)
    if image is None:
        cv2.putText(canvas, "MISSING CAMERA FRAME", (100, 240), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    for detection in update.detections:
        points = np.rint(detection.corners_px).astype(np.int32)
        cv2.polylines(canvas, [points], True, (0, 160, 0), 2)
        label = f"{detection.object_id} / {detection.tracking_id.rsplit(':', 1)[-1] if detection.tracking_id else 'UNASSOCIATED'}"
        cv2.putText(canvas, label, tuple(points[0] + [0, -8]), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 100, 0), 1)
    lines = [f"{scenario} | frame {update.frame_index} | {update.update_timestamp_ns/1e9:.1f}s",
             f"{update.processing_status} | {len(update.detections)} detections | {len(update.tracks)} track records"]
    for track in update.tracks:
        age = (update.update_timestamp_ns-track.last_observed_timestamp_ns)/1e6
        lines.append(f"#{track.tracking_id.rsplit(':', 1)[-1]} {track.object_id} {track.status}/{track.estimate_kind} age={age:.0f}ms")
    lines.append("Green = fresh detection. Retained tracks are listed, not drawn as fresh boxes.")
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (8, 505+i*27), cv2.FONT_HERSHEY_SIMPLEX, .43, (30, 30, 30), 1, cv2.LINE_AA)
    return canvas


def playback_html(index):
    return '''<!doctype html><meta charset="utf-8"><title>CV tracking replay</title>
<style>body{font:16px system-ui;margin:24px;background:#eee}img{max-width:100%;display:block}
button,select,input{margin:8px}pre{white-space:pre-wrap}</style>
<h1>CV tracking replay</h1><p>Actual pixel detections and track states. This is synthetic evaluation, not flight footage.</p>
<select id="scenario"></select><button id="play">Play / pause</button>
<input id="frame" type="range" min="0" value="0"><span id="label"></span>
<img id="view" alt="Annotated camera frame"><pre id="details"></pre>
<script>
const data = ''' + json.dumps(index).replace("<", "\\u003c") + ''';
const select=document.getElementById('scenario'), slider=document.getElementById('frame');
for(const name of Object.keys(data)){const o=document.createElement('option');o.value=name;o.textContent=name;select.append(o);}
function show(){const rows=data[select.value];slider.max=rows.length-1;const r=rows[+slider.value];
document.getElementById('view').src=r.image;document.getElementById('label').textContent=`frame ${slider.value} / ${rows.length-1}`;
document.getElementById('details').textContent=JSON.stringify(r.update,null,2);}
select.onchange=()=>{slider.value=0;show();};slider.oninput=show;
let playing=false;document.getElementById('play').onclick=()=>playing=!playing;
setInterval(()=>{if(playing){slider.value=(+slider.value+1)%data[select.value].length;show();}},100);show();
</script>'''


def markdown(report):
    lines = ["# Multi-object tracking benchmark", "",
             "| Sequence | TP / FP / FN | ID changes | Short-gap recovery | New ID after expiry | Ambiguity refusal | Confirmation p95 ms |",
             "| --- | --- | ---: | --- | --- | --- | ---: |"]
    def count(metric):
        return f"{metric['successes']}/{metric['opportunities']}" if metric['opportunities'] else "n/a"
    for name, r in report["sequences"].items():
        lines.append(f"| {name} | {r['tp']} / {r['fp']} / {r['fn']} | {r['unexpected_id_changes']} | "
                     f"{count(r['short_gap_reacquisition'])} | {count(r['new_id_after_expiry'])} | "
                     f"{count(r['ambiguity_refusal'])} | {r['confirmation']['p95_delay_ms']} |")
    lines += ["", "Open playback.html in a browser for frame-by-frame annotated playback and exact payloads.",
              "Full metrics and stage p50/p95/p99 timings are in report.json; evaluations.jsonl explains identity checks.",
              "Fully covered markers and missing frames are not observable detection opportunities. Duplicate-ID markers",
              "remain detection opportunities, but require refusal of association; they are excluded from identity continuity.",
              "A missed duplicate detection counts as an ambiguity-test failure, not successful refusal.",
              "Continuity/reacquisition opportunities follow scene visibility, not the detector's success.",
              "Confirmation delay is measured from the first observable frame of each truth-defined track episode;",
              "unconfirmed episodes are reported separately, not treated as zero delay.",
              "Expiry and retention audits are separately checked against emitted last-observed times; they do not measure",
              "ground-truth localization accuracy. Expected expiry is at age >=500ms; no prediction is implemented.",
              "No warmup frames are discarded: cold-start latency is included. Publication is a local append callback.",
              "Images are preloaded; timing excludes generation, PNG save/load, rendering, evaluation, and playback.",
              "End-to-end timing includes both pipeline and shared-publisher JSONL logging (buffered writes, no fsync).",
              "These are scripted baseline sequences, not real-camera or crossing-object/general tracking benchmarks."]
    return "\n".join(lines)+"\n"


def run(output, *, seed=2026, dataset=None, scenarios=SCENARIOS):
    if type(seed) is not int or seed < 0:
        raise ValueError("seed must be nonnegative integer")
    if not scenarios or any(name not in SCENARIOS for name in scenarios):
        raise ValueError("Choose supported nonempty scenarios")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    metadata = {"schema_version": 1, "seed": seed, "fps": FPS, "marker_side_m": SIDE_M,
                "calibration": asdict(CALIBRATION), "opencv": cv2.__version__, "numpy": np.__version__,
                "python": platform.python_version(), "platform": platform.platform(),
                "confirmation_hits": 2, "max_gap_ns": 500_000_000, "corner_gate_px": 8.,
                "warmup_frames": 0, "publication": "local append callback", "scenarios": {}}
    replay = None
    if dataset is not None:
        dataset = Path(dataset).resolve()
        raw = (dataset / "manifest.json").read_bytes()
        replay = json.loads(raw)
        if (replay["schema_version"] != 1 or replay["fps"] != FPS or replay["marker_side_m"] != SIDE_M
                or replay["calibration"] != json.loads(json.dumps(asdict(CALIBRATION)))):
            raise ValueError("Replay configuration does not match sequence schema/calibration")
        metadata["seed"] = replay["seed"]
        metadata["replayed_manifest_sha256"] = hashlib.sha256(raw).hexdigest()
        scenarios = tuple(replay["scenarios"])
    metadata["source_sha256"] = {name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
        for name in ("tracking_sequences.py", "tracking_metrics.py", "tracking_benchmark.py", "tracking.py",
                     "perception_pipeline.py", "perception_publisher.py", "shared_perception.py", "camera_providers.py",
                     "perception_types.py", "computer_vision.py")}
    reports, playback = {}, {}
    with (output / "events.jsonl").open("w", encoding="utf-8") as logs, \
         (output / "updates.jsonl").open("w", encoding="utf-8") as update_log, \
         (output / "evaluations.jsonl").open("w", encoding="utf-8") as eval_log:
        for name in scenarios:
            if name not in SCENARIOS:
                raise ValueError("Unsupported replay scenario")
            directory = output / name
            directory.mkdir()
            if replay is None:
                images, truth = make_sequence(name, seed)
            else:
                truth = replay["scenarios"][name]
                images = []
                for frame in truth:
                    if frame["missing_frame"]:
                        images.append(None)
                        continue
                    path = (dataset / frame["image"]).resolve()
                    if not path.is_relative_to(dataset):
                        raise ValueError("Image path escapes dataset")
                    raw = path.read_bytes()
                    if hashlib.sha256(raw).hexdigest() != frame["sha256"]:
                        raise ValueError("Image hash mismatch")
                    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        raise ValueError("Unreadable image")
                    images.append(image)
            for frame, image in zip(truth, images):
                relative = f"{name}/input_{frame['frame_index']:03d}.png"
                frame["image"], frame["sha256"] = None, None
                if image is not None:
                    save_image(output / relative, image)
                    frame["image"] = relative
                    frame["sha256"] = hashlib.sha256((output / relative).read_bytes()).hexdigest()
            metadata["scenarios"][name] = truth
            def sink(record):
                logs.write(json.dumps({"scenario": name, **record}, allow_nan=False)+"\n")
            published = []
            publisher = PerceptionPublisher(session_id=f"sequence-{name}", publisher=published.append, log_sink=sink)
            provider = ImageSequenceProvider(images, fps=FPS, source_id=name, calibration_id=CALIBRATION.calibration_id)
            updates, views = [], []
            with PerceptionPipeline(provider, calibration=CALIBRATION, marker_side_m=SIDE_M,
                                    timestamp_mapper=lambda ns: ns, log_sink=sink) as pipeline:
                for index, image in enumerate(images):
                    now = index*100_000_000
                    transform = CameraToENU("front", now, "sequence_world", (0., 0., 2.),
                                            (1., 0., 0., 0.), "synthetic_downward")
                    start = perf_counter_ns()
                    observation = pipeline.read()
                    update = publisher.update(observation, now_ns=now, transform=transform)
                    elapsed = perf_counter_ns()-start
                    update = replace(update, timings_ns=update.timings_ns + tuple(
                        (f"pixel_{k}", v) for k, v in observation.timings_ns) + (("end_to_end", elapsed),))
                    updates.append(update)
                    payload = update.to_dict()
                    update_log.write(json.dumps({"scenario": name, "update": payload}, allow_nan=False)+"\n")
                    relative = f"{name}/annotated_{index:03d}.png"
                    save_image(output / relative, annotate(image, update, name))
                    views.append({"image": relative, "update": payload})
            reports[name], evaluations = evaluate_sequence(truth, updates)
            for row in evaluations:
                eval_log.write(json.dumps({"scenario": name, **row}, allow_nan=False)+"\n")
            playback[name] = views
    save_json(output / "manifest.json", metadata)
    report = {"schema_version": 1, "sequences": reports}
    save_json(output / "report.json", report)
    (output / "report.md").write_text(markdown(report), encoding="utf-8")
    (output / "playback.html").write_text(playback_html(playback), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--dataset", type=Path, help="Replay an earlier run's saved images and manifest")
    args = parser.parse_args()
    try:
        report = run(args.output, seed=args.seed, dataset=args.dataset)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(2, f"Tracking benchmark failed: {exc}\n")
    print(markdown(report))
    print(f"Playback: {args.output / 'playback.html'}")


if __name__ == "__main__":
    main()
