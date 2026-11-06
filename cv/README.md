# Computer vision

[Shared integration contract](../INTEGRATION_README.md)

CV answers **what the camera sees**. It implements image ingestion, ArUco
detection, camera-relative pose estimation, camera-local tracking, and shared
observation publication. It does not select mission states, issue motion commands,
or grant tracking/docking permissions.

**Current status:** the standalone CV pipeline, shared publisher, synthetic
benchmarks, and diagnostic demos are implemented. The state-machine run loop
still uses scripted bird/alignment inputs; it does not yet consume
`PerceptionUpdate`. Hardware capture and ROS2 transport are not implemented.

| Component | Implemented behavior | Boundary / limitation |
| --- | --- | --- |
| `computer_vision.py` | Scheduled, prescribed bird fixtures and schema-v1 bird snapshots | Compatibility path; no pixel-based bird recognition |
| `camera_providers.py` | Common `read() -> FrameSample` / `close()` interface, synthetic image sequences, recorded video, clean marker renderer | No live-camera provider; video assumes constant FPS and known frame count |
| `perception_pipeline.py`, `perception_types.py` | Pixel-based ArUco detection, calibrated pose, internal `DetectionResult` in `CVObservation` | Camera-relative geometry; confidence is null; no tracking in the raw pipeline |
| `shared_perception.py`, `perception_publisher.py` | Shared `PerceptionUpdate`, detection provenance, ENU conversion, local callback publication | Requires explicit simulation mapping and capture-time transform; no ROS2 messages/transport |
| `tracking.py` | Persistent ArUco IDs, confirmation, retention, expiry, reset | One camera/source per publisher; no prediction or cross-camera association |
| `benchmark_dataset.py`, `benchmark_metrics.py`, `benchmark.py` | Seeded degradation sweeps, saved-image replay, detection/pose metrics and latency reports | Single-marker scenes; benchmarks raw perception, not full tracking performance |
| `tracking_sequences.py`, `tracking_metrics.py`, `tracking_benchmark.py` | Multi-marker temporal sequences, truth-based identity checks, lifecycle audits, timing reports, annotated browser playback | Scripted synthetic baseline; no Kalman prediction or general-object/cross-camera tracking |

```mermaid
flowchart LR
    subgraph CV[CV subsystem]
        A[Camera Provider] --> B[Preprocessing]
        B --> C[ArUco Detection]
        C --> D[Camera-relative Pose]
        D --> E[Capture-time ENU Adapter]
        E --> F[Tracking and Retention]
        F --> G[PerceptionUpdate Publisher]
    end
    INPUT[Synthetic Images or Recorded Video] --> A
    CONTEXT[Caller: Clock Mapping and Capture-time Transform] --> E
    G --> OUTPUT[Coordinator and Consumer Adapters]
```

The prescribed bird fixture path is separate from this pixel pipeline. Both use
injected logging sinks. Synthetic evaluation truth stays outside the detector
and tracker. Identity tracking can continue without ENU pose; unavailable world
geometry is represented explicitly as null.

## Coordinator interface

This section describes the existing bird fixture adapter, available for coordinator
integration. The pixel-based shared contract is documented under
[Coordinator-facing PerceptionUpdate](#coordinator-facing-perceptionupdate).

```python
from cv.computer_vision import ComputerVision

records = []
cv = ComputerVision(
    timestamp_mapper=lambda source_ns: source_ns,  # Explicit shared origin.
    log_sink=records.append,  # Coordinator-owned callable receiving a dict.
)
cv.start_camera("cv/scenarios/integration_cycle.json", camera_id="front")
try:
    for tick in range(200):
        cv.set_log_context(state="IDLE")  # Use the actual mission State.value.
        observation = cv.read_due(tick * 100_000_000)
        if observation is not None:
            bird = observation.to_bird_snapshot()
            # Coordinator puts bird in snapshot['bird']; Safety checks age.
except StopIteration:
    pass  # Controlled scenario completion, not camera failure.
finally:
    cv.stop_camera()
```

`read_due(now_ns)` consumes all samples due at the supplied simulation time and
returns the latest one. A 15 Hz fixture in a 10 Hz loop can consume more than one
sample per tick. Every consumed sample is logged. Before the first sample is
due it returns `None`; the coordinator must treat that as unavailable input.
Between samples it returns the cached immutable observation without changing
its timestamp or logging it again. Safety owns freshness decisions (the shared
default maximum age is 0.5 seconds).

The mapper must be pure and strictly increasing; use a fixed simulation-origin
offset, never the current tick time. `now_ns` must be a nonnegative integer and
must not move backwards. A finite source ends at the scheduled time of the
sample immediately after its last frame. `StopIteration` remains stable on
subsequent scheduled reads, and `exhausted` is true. Stop/start explicitly
restarts the fixture and scheduling clock. A jump beyond the end consumes/logs
remaining due samples and raises `StopIteration` in that call.

`read()` remains available for sequential frame export. Do not mix it with
`read_due()` in the same camera session. Sequential reads retain their existing
source-clock output and optional mapped time.

## Shared bird snapshot

`observation.to_bird_snapshot()` returns a fresh schema-version-1 dictionary:

- `detected`: true for a fixture bird, false for valid absence, null for missing input.
- `valid`, `camera_ok`, `reason`: missing frames are invalid with `missing_frame`.
- `type`, `distance_m`: prescribed fixture values, not measured geometry.
- `timestamp_ns`, `clock`: mapped sample time and `simulation`.
- `source_timestamp_ns`, `source_clock`: unchanged fixture time and `fixture`.
- `source_id`, `camera_id`, `frame_index`, `synthetic`: sample identity/provenance.

The adapter requires an explicit clock mapper. Valid absence and missing input
remain distinct. The coordinator may copy camera status into its environment
section, preserving the sample's validity/time; CV never overwrites unrelated
battery, environmental, or docking fields. Association with any separately
supplied synthetic target can use source ID, camera ID and frame index.

## Fixtures and logging

- `scenarios/bird_demo.json`: original looping image demo.
- `scenarios/integration_cycle.json`: finite 15 Hz appearance, brief absence,
  reappearance, prolonged absence, and missing frames.
- `scenarios/stale_samples.json`: 1 Hz stream; at a 10 Hz loop, cached samples
  exceed the 0.5-second freshness limit before the next sample arrives.

These fixtures prescribe observations only. They do not assert return, Safety
permissions, alignment, or touchdown. Existing scenario validation rejects
malformed booleans, missing observation fields, and invalid fixture distances.

Inject `log_sink` to route CV events through the coordinator's shared logger.
Records contain `State` and `Details` (module, event, and source/sample metadata).
The coordinator owns the full JSONL tick stream. The default compatibility sink
uses `universal_log/Universal_log.py`; its text logs are not that tick stream.
`to_dict()` keeps the legacy CV observation shape; `to_bird_snapshot()` is the
shared-loop adapter. The standalone image exporter uses its own schema v2.

## Setup and checks

Generated exports in `cv/synthetic_output/` are ignored by Git. Commit scenario
definitions and generators; keep any deliberately curated regression images in
a separate fixtures directory.

In WSL, use the existing CV environment and run from the repository root:

```bash
source ~/venvs/contextualai-cv/bin/activate
cd /mnt/c/Users/capta/Documents/RCOS/ContextualAI/Phase0-MicroDrone
# Install dependencies if this environment has not been set up:
python -m pip install -r cv/requirements.txt
# Full CV suite:
python -m unittest discover -s tests -p 'test_computer_vision.py' -v
python -m unittest discover -s tests -p 'test_cv*.py' -v
```

Current coverage is 62 tests: 17 fixture/interface, 10 perception, 10 benchmark,
17 tracking/shared-publication, and 8 temporal-benchmark tests. Tests run
headlessly, including temporary-video
playback. `.github/workflows/cv-ci.yml` defines the same suites on Python 3.12
and 3.14; configured CI is not itself evidence that a remote run passed.

For inspectable outputs:

```bash
python -m cv.visual_demo
code cv/synthetic_output/visual_demo/comparison.png
python -m cv.synthetic_frames --scenario cv/scenarios/integration_cycle.json --headless --fast --once
```

The visual demo uses real pixel detection on centered, tilted, and empty scenes.
It writes annotated images, `results.json`, and `observations.jsonl` into
`synthetic_output/visual_demo/` (re-running replaces those demo outputs). The
`synthetic_frames` command exports prescribed fixtures, not measured detections.
Benchmark and tracking-demo commands appear in their respective sections below;
they require new output directories.

## Pixel perception interface

```python
from cv.camera_providers import ImageSequenceProvider, render_marker
from cv.perception_pipeline import PerceptionPipeline
from cv.perception_types import CameraCalibration

calibration = CameraCalibration(
    "front_640", 640, 480,
    ((600., 0., 320.), (0., 600., 240.), (0., 0., 1.)),
)
provider = ImageSequenceProvider(
    [render_marker(calibration, marker_id=17)],
    camera_id="front", calibration_id=calibration.calibration_id,
)
records = []
with PerceptionPipeline(
    provider, calibration=calibration, marker_side_m=0.2,
    timestamp_mapper=lambda ns: ns, log_sink=records.append,
) as pipeline:
    pipeline.set_log_context(state="IDLE")  # Coordinator supplies State.value.
    observation = pipeline.read()
    print(observation.to_dict())
```

Replace the provider with `RecordedVideoProvider(path, camera_id="front",
calibration_id=calibration.calibration_id)` to use the same pipeline on recorded
pixels. Files must have known frame counts and constant FPS; timestamps use
`frame_index / fps` with `clock="media_fps"`. A decode failure before declared
EOF raises and logs an acquisition error; EOF raises `StopIteration` and closes
the source. `close()` is idempotent. Providers do not perform wall-clock pacing.

`read()` returns the existing `CVObservation` with additive fields: `detections`,
`processing_status`, `calibration_id`, and `timings_ns`. Its serialized perception
extension is named `cv_perception`, version 1. Legacy fixture serialization is
unchanged. Perception returns `bird=null`; `to_bird_snapshot()` rejects perception
results, so markers cannot accidentally become bird detections or launch inputs.
Use `valid`, `status`, and `detections` for generic perception, not the legacy
`detected` bird field. Missing/error results have `detections=null`; successful
empty images have an empty list. Detection without pose remains valid.

The default dictionary is `DICT_4X4_50`; IDs use `aruco:<numeric_dictionary>:<id>`.
All detected markers currently share the configured physical side length.
Confidence and tracking IDs remain null in the raw pipeline output. The optional
shared publisher below supplies track IDs. Pose uses iterative planar PnP with
positive-depth and reprojection checks (default maximum RMS error: 3 pixels).
This is a clean-scene baseline: planar pose ambiguity is not resolved using
temporal information, and a low reprojection error does not guarantee accurate
pose. Calibration ID and resolution must match. Corner order is top-left,
top-right, bottom-right, bottom-left in the decoded marker's orientation.

Raw pose uses camera optical axes x right, y down, z forward. Marker-local axes
are x right, y up, z out of its printed face; orientation is a quaternion rotating
marker-local vectors into the camera frame. Metric pose requires intrinsics,
distortion coefficients, matching image resolution/calibration ID, and physical
marker side length. A detected marker without valid pose retains its identity and
corners. Raw `DetectionResult.type` and envelope `camera_id` become `object_type`
and `source_camera` in the separate shared publication contract; no legacy fields
are renamed. Raw observations and shared updates are not interchangeable schemas.

The clean renderer supports zero distortion; recorded-image pose estimation
accepts OpenCV distortion coefficients. Images and calibration use BGR uint8
pixels at the original resolution. Evaluation truth stays with the scene caller
and never enters the perception sample.

The coordinator still owns scheduling and clock mapping. This new pull API does
not implement `read_due()` and is not wired into `run_loop.py`. Logging state is
opaque context supplied by the caller; CV defines no mission-state enum and
cannot transition states. The existing eight state names remain unchanged.

Each processed sample emits a `State`/`Details` log record through the injected
sink. Durations cover acquisition, preprocessing, detection, pose, publication,
and total processing, excluding final logging overhead. The optional publisher
receives processing timings; its duration is known only afterward and appears in
the returned observation and log. Callback/logging exceptions propagate to the
caller. The offline benchmark below supplies run manifests and aggregate latency
reports. The shared publisher below adds persistent ArUco identity and lifecycle.

## Synthetic benchmark and replay

In the activated WSL `~/venvs/contextualai-cv` environment, from the repository root:

```bash
python -m unittest discover -s tests -p 'test_cv_benchmark.py' -v
# Quick run: 310 frames, ten per condition.
python -m cv.benchmark --output cv/synthetic_output/benchmark_quick --samples 10 --seed 2026
# Range assessment requires at least 100 samples per distance: 3,100 frames total.
python -m cv.benchmark --output cv/synthetic_output/benchmark_full --samples 100 --seed 2026
# Replay the exact saved PNGs; use a NEW output directory each time.
python -m cv.benchmark --dataset cv/synthetic_output/benchmark_full/dataset --output cv/synthetic_output/benchmark_replay
code cv/synthetic_output/benchmark_full/report.md
```

Existing output directories are refused to prevent overwriting evidence. Outputs:

| File | Contents |
| --- | --- |
| `dataset/manifest.json`, `dataset/images/*.png` | Seeds, per-frame settings, calibration, image hashes, and separate ground truth |
| `run.json`, `opencv_build.txt` | Effective detector/evaluator settings, warmup, software/build/machine information, Git revision and source hashes |
| `observations.jsonl` | Actual perception records; no evaluation truth |
| `evaluation.jsonl` | Per-frame TP/FP/FN, pose errors, and timings |
| `report.json`, `report.md` | Machine-readable metrics and a readable per-condition table |

There are 31 conditions: baseline, physical distance (0.5-6 m), in-plane rotation,
out-of-plane tilt, brightness, horizontal motion blur, partial occlusion, physical
marker size (scale), background clutter, two negative scenes, and a combined
stress case. Each present frame currently contains one marker with a seeded ID
and subpixel position variation. Multiple-marker detection has separate tests;
multi-object temporal scenes are covered by the separate tracking benchmark below. Default seed 2026 is the
baseline; use a separately recorded seed for tuning and hold it fixed for comparisons.

Clutter stays behind the marker, including its white cells. Occlusion covers a
marker-coordinate strip and records actual raster area coverage. Every present
marker counts in the denominator, even under heavy occlusion. Brightness is a
pixel-intensity multiplier, not calibrated lux. Scale changes known physical
marker size at fixed distance; distance uses fixed size and calibration. Distortion
is zero; these scenes do not establish physical-camera performance.

Matching requires the same marker ID and ordered-corner RMS error at most 8 px
(`--tolerance-px`). Matching is one-to-one; duplicate/wrong-ID detections count as
FP and unmatched truth counts as FN. Undefined ratios are null. Reports include
detection rate and its 95% Wilson interval, precision, false positives per negative
frame, and pose coverage over both all truth and matched detections. Pose errors
cover X/Y/Z bias, MAE, RMSE, p95 absolute error; wrapped ZYX yaw/pitch/roll errors;
and rotation angular error. Euler metrics exclude near-singular poses and report
their count. Detection/pose failures remain visible rather than becoming zero error.

Range is the farthest contiguous passing tested distance, starting at the nearest,
with at least 100 objects per distance and a detection-rate point estimate >=95%.
Per-condition uncertainty intervals are reported separately. This applies only
to the clean synthetic distance sweep, not to combined degradations or docking
pose tolerances. A ten-sample quick run leaves qualified range null.

Three warmup frames per condition are excluded from metrics (`--warmup` changes
this); all evaluation frames are retained. Stage p50/p95/p99 latencies are in ms.
Acquisition includes file read, hash verification, and PNG decoding. Publication
is a no-op local callback; logging measures JSON serialization/buffered writes,
not durable fsync. `read_including_logging` includes the sink; `total_processing`
does not. Evaluation and image generation are outside pipeline timings. No
tracking-stage timing is claimed. Timing is machine/load-dependent and not
expected to reproduce exactly; accuracy replay is tested with the same software.

This benchmark changes no DetectionResult fields, tracking semantics, mission
states, Motion APIs, or coordinator behavior. It benchmarks raw perception;
tracking has its own temporal benchmark, validation suite, and diagnostic replay below.

## Multi-object tracking benchmark

This runner exercises actual pixels through `PerceptionPipeline` and
`PerceptionPublisher`. It preserves the existing shared message format and uses
an explicitly synthetic, fixed downward-camera-to-ENU transform. Each run has seven
36-frame sequences at 10 Hz (252 frames total): independent marker motion, short
full occlusion, long full occlusion, three missing camera frames, duplicate IDs,
marker entry/exit/re-entry, and simultaneous rotation/tilt. The `rotation_tilt`
scenario varies in-plane rotation and out-of-plane tilt independently for two
markers (up to approximately 34 degrees each). Select it in playback to inspect
tracking continuity as perspective changes; this is not an orientation-error metric.
Markers have distinct evaluator entity IDs even
when their printed IDs are duplicated. The generator's truth never enters the
provider, detector, or tracker.

```bash
# From the repository root, in the activated WSL contextualai-cv environment:
python -m unittest discover -s tests -p 'test_cv_tracking_benchmark.py' -v
python -m cv.tracking_benchmark --output cv/synthetic_output/tracking_run --seed 2026
python -m cv.tracking_benchmark --dataset cv/synthetic_output/tracking_run --output cv/synthetic_output/tracking_replay
code cv/synthetic_output/tracking_run/report.md
```

Use a NEW output directory for each run. Open `playback.html` in a browser to
select a sequence, scrub individual frames, or play at 10 Hz. It works locally
without a web server or GUI support in Python. The JSON beneath each image is
the actual shared update. Green boxes represent current detections; retained
tracks are listed with measurement ages rather than drawn as fresh detections.
The individual `annotated_*.png` images can also be opened directly in VS Code.

| Artifact | Contents |
| --- | --- |
| `manifest.json` | Sequence truth, per-frame image paths/hashes, seed, calibration, software/source hashes, and tracking settings |
| `<sequence>/input_*.png` | Exact available input frames; missing images have null paths in the manifest |
| `<sequence>/annotated_*.png`, `playback.html` | Inspectable detections, IDs, lifecycle state, ages, and complete payloads |
| `events.jsonl` | Pipeline and shared publisher logs, labeled by sequence |
| `updates.jsonl` | Returned shared updates with benchmark timing entries |
| `evaluations.jsonl` | Per-frame detection counts and identity checks against truth |
| `report.json`, `report.md` | Per-sequence metrics and readable summary |

The evaluator matches printed identity and ordered corners within 8 px RMS,
one-to-one, including duplicate-ID detections at distinct image locations. It
reports detection TP/FP/FN, identity continuity, unexpected ID changes, same-ID
reacquisition before expiration, new IDs after long gaps, explicit ambiguity
refusal, and confirmation delay. Metrics include opportunity counts; empty
denominators are null. Missing detections count against applicable identity
checks rather than removing those opportunities. Duplicate detections must be
present AND explicitly refused association to pass the ambiguity check.

Unlike the single-frame degradation benchmark, fully covered markers and missing
frames are intentionally excluded from observable detection opportunities. They
remain in scene truth and drive temporal gap scenarios. Confirmation episodes
start at the first uniquely observable frame and restart after an expiry-length
gap; unconfirmed episodes are counted separately from delay percentiles.
Lifecycle audits separately check emitted expiry timing and that retained states
do not refresh measurement/estimate timestamps or change pose. These audits use
emitted last-observed times and are not a substitute for truth-based identity or
localization metrics.

Reports include adapter, tracking, local publication, raw pixel-stage, and total
end-to-end p50/p95/p99 latency. All frames, including cold starts, are measured.
Images are preloaded; generation, PNG loading/saving, evaluation, and annotation
are excluded. End-to-end timing includes buffered JSONL logging from both CV
layers; publication measures a local append callback, not network delivery.
Replay checks image hashes and configuration. Accuracy should reproduce in the
same software environment; execution timings will vary.

These are baseline scripts with separated marker paths, not crossing-object,
cross-camera, or real-world robustness evidence. No prediction or new motion,
state-machine, or shared-message behavior is introduced.

## Coordinator-facing PerceptionUpdate

**Purpose:** distinguish actual camera measurements from persistent track state.
**Publisher:** `cv.perception_publisher.PerceptionPublisher`, a CV-owned local
adapter. **Subscribers:** the coordinator and, through explicit routing, Docking,
Safety, and future Navigation/autonomy. Motion still receives approved commands.
**Rate:** one `update()` per consumed frame, plus optional coordinator-tick
heartbeats with no image. There is no background loop, ROS dependency, or command
dispatch. Existing `ComputerVision` bird fixtures and raw `PerceptionPipeline`
outputs are unchanged.

```python
from cv.perception_publisher import PerceptionPublisher
from cv.shared_perception import CameraToENU

# Construct once per camera/source session. pipeline is a PerceptionPipeline
# configured with the coordinator's explicit source-to-simulation timestamp_mapper.
shared_cv = PerceptionPublisher(
    publisher=lambda update: coordinator_queue.append(update.to_dict()),
    log_sink=shared_log_sink,
    confirmation_hits=2,
    max_gap_ns=500_000_000,
)

# Called only when the coordinator schedules a new camera sample:
shared_cv.set_log_context(state=current_state.value)
observation = pipeline.read()
update = shared_cv.update(
    observation,
    now_ns=clock.timestamp_ns,
    transform=camera_to_enu_at_capture,  # CameraToENU, or None if unavailable
)

# Between samples, advance lifecycle without replaying the last observation:
update = shared_cv.update(now_ns=clock.timestamp_ns)
```

The snippet defines the integration boundary; `coordinator_queue`, clock,
capture-time transforms, and scheduling are caller-owned and are not yet wired
into `run_loop.py`. Use either the callback or the return value as the consumption
path, not both. The callback gets the update before its own publication duration
is measurable; the returned/logged update additionally includes that duration.

### Envelope fields and their producers

`PerceptionUpdate.to_dict()` returns JSON-compatible data with these exact keys:

| Field | Meaning / producer |
| --- | --- |
| `schema_name`, `schema_version` | CV constants: `perception_update`, `1` |
| `session_id` | CV-generated unique session plus reset generation; a supplied base ID must be unique per run/stream |
| `source_id`, `source_camera` | Provider identity from `CVObservation.source_id` / `camera_id` |
| `clock` | `simulation`; all shared timestamps except explicitly named source time use this clock |
| `update_timestamp_ns` | Coordinator's integer `now_ns`; when this update is evaluated |
| `capture_timestamp_ns` | `CVObservation.simulation_timestamp_ns`, produced by the explicit clock mapper |
| `source_timestamp_ns`, `source_clock` | Original provider time/clock, unchanged |
| `frame_index` | Provider frame number |
| `synthetic` | Provider provenance; recorded-video pixels are not automatically synthetic |
| `sample_received` | True for a new observation; false for a heartbeat |
| `frame_valid` | Image availability from `camera_ok`; null on heartbeats |
| `valid` | New frame successfully processed; false for failure or heartbeat; not a pose/freshness permission |
| `reason` | Source/processing failure reason, `no_new_frame`, or null |
| `processing_status` | `ok`, `unavailable`, `error`, or `no_new_frame` |
| `detections` | Actual new detections only; always a list in the shared format |
| `tracks` | Full current camera/session track snapshot, plus one-update expiration records |
| `timings_ns` | Adapter/validation/transformation, tracking, and local publication durations; raw detector timings remain in the pipeline log |

On heartbeats, capture/source timestamps and frame index identify the last
received sample; they are not refreshed. `sample_received=false` makes this
explicit. A first sample is required before sending heartbeats. Missing frames
can be that first sample. EOF remains the provider's `StopIteration`, not a
fabricated missing image; the coordinator decides whether to stop or age tracks.

### Each `detections[]` entry

Internally this is `SharedDetectionResult`. Its `to_dict()` flattens its `WorldPose`:

| Fields | Meaning / producer |
| --- | --- |
| `timestamp_ns`, `clock` | Mapped image capture time, `simulation`; never publication time |
| `source_timestamp_ns`, `source_clock`, `source_id`, `source_camera` | Copied frame provenance so a detection can be inspected independently |
| `object_id`, `object_type` | Detector identity/type; internal `type` maps to shared `object_type` |
| `confidence`, `confidence_method` | Detector score/method; currently null, not a fabricated probability |
| `corners_px` | Four measured image corners in decoded marker order |
| `tracking_id`, `association_reason` | Tracker association; null ID for ambiguous, stale, or unsupported-type observations, with a reason |
| `valid`, `reason` | Actual image detection is valid; pose validity is independent |
| `position_x`, `position_y`, `position_z` | Marker-center ENU metres, or all null |
| `orientation` | Quaternion object `{x,y,z,w}` rotating marker-local vectors into ENU, or null |
| `yaw_deg` | Heading of marker +X projected onto ENU XY, zero east and CCW positive, in [0,360); null if that axis is vertical or pose unavailable |
| `pose_valid`, `pose_reason` | Whether world pose is available; e.g. `transform_unavailable`, `transform_time_mismatch`, `transform_camera_mismatch`, or source pose failure |
| `frame`, `origin_id`, `transform_id` | `local_enu` and the supplied transform's named origin/identity; identities null when pose unavailable |

`CameraToENU(source_camera, timestamp_ns, origin_id, translation_m, orientation,
transform_id)` must match the camera and exact mapped capture timestamp. It
implements `p_enu = R_camera_to_enu * p_camera + translation_m`. The coordinator
supplies the calibrated camera mounting transform composed with capture-time
vehicle attitude/position, or a known fixed camera-to-world transform. A latest
vehicle pose is not interchangeable with capture-time pose. Quaternion and
translation validation reject malformed/nonfinite input. No transform means
valid detection/identity but null shared pose, never relabeled camera geometry.

### Each `tracks[]` entry

| Fields | Meaning / producer |
| --- | --- |
| `tracking_id` | CV session/generation + monotonic track instance number |
| `object_id`, `object_type`, `source_id`, `source_camera` | Associated object's identity/provenance |
| `status` | `tentative`, `confirmed`, `temporarily_lost`, or `expired` |
| `estimate_kind` | `measured` or `retained`; `predicted` is reserved but never emitted yet |
| `last_observed_timestamp_ns` | Latest unambiguous matching detection's capture time |
| `estimate_timestamp_ns` | Time represented by the stored pose; unchanged during retention, null if pose unavailable |
| `hits` | Total associated, distinct observed samples; heartbeats/duplicates do not increment it |
| `confidence` | Last associated detection's confidence; does not become a track probability |
| `valid`, `reason` | Active identity versus expired identity; `valid=true` does not imply fresh measurement or usable pose |
| `velocity_m_s`, `position_covariance_m2` | Reserved nullable XYZ velocity and 3x3 position covariance; both null in the retention estimator |
| Pose fields | Same flattened ENU fields as above, held at `estimate_timestamp_ns` |

Track timestamps inherit the envelope's simulation clock. On a new associated
detection with unavailable pose, the track retains its identity but clears world
geometry; it does not present an older pose as a new measurement. On no matching
detection it retains the previous pose/time, marked `retained`. Predicted state
and a Kalman filter are not implemented. `LastMeasurementEstimator.observe()` /
`advance()` separate estimation from association/lifecycle for later replacement.

Default confirmation is two consecutive unambiguous observed frames. A genuinely
empty/missing/failed/ambiguous sample breaks the confirmation streak; a heartbeat
does not. Heartbeats retain the lifecycle status until expiry (no scheduled image
is not evidence of a missed detection), while marking the estimate `retained`.
A confirmed track stays confirmed on reacquisition within the gap.
Expiry occurs when `now_ns - last_observed_timestamp_ns >= 500_000_000` by default.
It emits `expired, valid=false` once, then disappears; reacquisition gets a new ID.
Thresholds are configurable CV identity policy, not Safety's freshness policy.

Only ArUco IDs are associated initially; other object types remain untracked.
Duplicate IDs in a view are ambiguous and update no track. Each publisher owns
one camera/source stream; there is no cross-camera fusion. Replayed or decreasing
frame indices, source times, or mapped times are rejected before track mutation.
Future captures and backwards update times are rejected. Call `reset()` after
source/clock restart or ENU-origin changes: it logs a reset, clears tracks, and
changes session generation. Consumers replace their cached track set on a new
session ID. State names are opaque log context only.

### Runnable coordinator payload examples

```bash
python -m unittest discover -s tests -p 'test_cv_tracking.py' -v
python -m cv.tracking_demo --output cv/synthetic_output/tracking_demo
code cv/synthetic_output/tracking_demo/measured.json
code cv/synthetic_output/tracking_demo/heartbeat.json
code cv/synthetic_output/tracking_demo/missing.json
code cv/synthetic_output/tracking_demo/expired.json
```

Use a new output directory on subsequent runs. `updates.jsonl` contains full
coordinator-ready payloads; `events.jsonl` includes raw perception and shared
publication records. The demonstration uses real pixel detections and an
explicitly synthetic downward-camera transform, not real drone extrinsics.

| Case | Envelope | Detection list | Track behavior |
| --- | --- | --- | --- |
| Marker observed | `valid=true`, `frame_valid=true` | Current detections | Measured, tentative/confirmed |
| Empty image | `valid=true`, `frame_valid=true` | Empty | Retained or expired |
| Missing image | `valid=false`, `frame_valid=false` | Empty | Retained or expired |
| Detector failed | `valid=false`, `frame_valid=true` | Empty | Retained or expired |
| Heartbeat | `valid=false`, `frame_valid=null`, `sample_received=false` | Empty | Retained or expired |

Malformed adapter inputs raise `ValueError` without changing tracks; the caller
must handle that failure rather than reuse a previous result as fresh. Callback
and logger errors propagate. Publication failures are logged when possible, but
do not roll back accepted measurements; this local callback is not a reliable
transport or retry queue. ROS2 message/IDL bindings and uXRCE-DDS remain pending.

### Required downstream changes (not implemented here)

- **Coordinator (`state_machine/run_loop.py`):** create one shared publisher per
  camera/source; supply `State.value`, clock mapping, capture-time transforms,
  and logger. Schedule raw `read()` only when a sample is due; call `update(None,
  now_ns=...)` between samples. Remove tick-time restamping for CV-sourced fields
  in the loop that currently assigns `timestamp_ns=now_ns` to snapshot sections.
  Keep snapshot tick time separate from measurement time. Queue and consume one
  update once; preserve source/frame identity and reset cached tracks on session
  change. Proposed placement is `snapshot['perception'] = update.to_dict()`;
  this new snapshot section still needs integration-team adoption.
- **Mission function (`state_machine/state_machine.py`):** preserve all eight
  state identities. Do not treat an ArUco marker as `bird.detected`. Existing
  bird fixture inputs remain separate until an explicit consumer adapter exists.
- **Motion / pose supplier:** provide a capture-time camera-to-ENU transform or
  the pose history needed to form it. Existing yaw-only vehicle pose suffices
  only for declared level-attitude synthetic fixtures; arbitrary camera attitude
  requires full orientation/extrinsics. No change to Motion command/ack APIs is
  required, and Motion should not consume detections directly.
- **Docking:** map the designated pad marker and its frame/origin to alignment
  inputs. Check pose validity and last-observed age; retained identity is not
  descent permission. Missing transform must yield unavailable alignment.
- **Safety:** explicitly assess measurement age, estimate kind, pose validity,
  and track status; do not infer freshness from update time or `track.valid`.
  Any future allowance for predicted geometry requires a separate policy.


## Phase 0 perception roadmap

The implemented components and contracts above are the current source of truth.
Remaining work is split by ownership and does not imply that existing detection,
pose, or tracking interfaces must be rebuilt.

| Remaining Phase 0 work | Owner | Completion evidence |
| --- | --- | --- |
| Schedule CV reads and heartbeats, preserve capture timestamps, handle EOF/failures and resets | Coordinator with CV support | Integrated tests at differing camera/tick rates, including stale cached input |
| Supply capture-time ENU transforms and adapt the designated pad marker | Coordinator / pose supplier / Docking | Explicit frame/origin agreement; missing transforms produce unavailable alignment |
| Consume detections and retained tracks according to freshness/validity policy | Safety / Docking / coordinator | Retained identity never silently becomes a fresh measurement or descent permission |
| Expand temporal evaluation beyond the implemented baseline sequences | CV | Crossing/overlapping paths, broader degradation sweeps, and consumer-agreed robustness limits |
| Agree usable accuracy/latency limits and condition envelope | CV and consumers | Measured acceptance criteria; synthetic range not presented as hardware capability |
| ROS2 message definitions and serialization mapping | CV / integration | Message definitions preserve timestamps, nullability, validity, frame, and measured/retained distinctions; not implemented yet |
| Integrated perception-to-consumer replay | Cross-team | Logs explain observed, absent, missing, stale, retained, expired, and reacquired states without drone hardware |

See [required downstream changes](#required-downstream-changes-not-implemented-here)
for concrete coordinator, Motion/pose-supplier, Docking, and Safety tasks. These
are documented requirements, not changes already made to those modules.

The estimator boundary (`LastMeasurementEstimator.observe()` / `advance()`) and
nullable velocity/covariance fields already provide basic Kalman integration
points. A predictive estimator or Kalman filter is a possible later Phase 0
extension; it is not required before coordinator integration. Prediction would
need its own uncertainty, timestamp, and consumer-policy validation. Current
tracking only measures or retains state.

## Phase 1 boundary

Phase 0 establishes a reproducible perception architecture using synthetic inputs
and recorded video. Phase 1 validates and operates that architecture with live
sensors on the intended moving hardware:

- Live-camera drivers, exposure control, connection recovery, and measured
  intrinsics/distortion/mounting calibration.
- Camera/vehicle time synchronization and capture-time pose on real hardware.
- Deployed ROS2/uXRCE-DDS transport and its latency/failure behavior.
- Onboard throughput, power/thermal constraints, dropped-frame handling, and
  controlled hardware/field validation under real lighting, vibration, and motion.
- Estimator tuning and optional additional detectors/depth sources where required
  by the mission; neither YOLO nor bird recognition is a prerequisite for the
  current Phase 0 handoff.

CV continues to publish observations and estimates in either phase. Mission
planning, threat assessment, deterrence selection, and decisions about following,
docking, recovery, or aborting remain outside its responsibility.
