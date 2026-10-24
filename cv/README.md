# Computer vision

[Shared integration contract](../INTEGRATION_README.md)

CV supports two paths: `ComputerVision` supplies prescribed bird observations
for the existing mission loop; `PerceptionPipeline` detects ArUco markers from
synthetic images or recorded video and optionally estimates camera-relative pose.
Neither path decides mission state or issues motion commands. No hardware,
PX4, ROS, or independent mission loop is required.

## Coordinator interface

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
python -m unittest discover -s tests -p "test_computer_vision.py" -v
python -m unittest discover -s tests -p "test_cv_perception.py" -v
python -m cv.visual_demo
```

These run 17 fixture tests and 10 perception tests respectively. Verbose output
shows each test and its outcome; any failed assertion prints a traceback. Tests
are headless, including the test that writes and reads a temporary video. The
visual demo writes `cv/synthetic_output/visual_demo/comparison.png` with detected
corners and known versus estimated XYZ for centered, tilted, and empty scenes.
Open that image in VS Code. Its `results.json` contains full observations and
`observations.jsonl` contains pipeline logs. These are actual pixel-derived
results; the clean synthetic scene truth is used only for comparison.

From the repository root, in a Python environment:

```bash
python -m pip install -r cv/requirements.txt
python -m unittest discover -s tests -p test_computer_vision.py -v
python -m unittest discover -s tests -p test_cv_perception.py -v
python -m cv.synthetic_frames --scenario cv/scenarios/integration_cycle.json --headless --fast --once
```

The fixture path still produces prescribed booleans. Front and downward camera
IDs are supported. The perception path below produces measured camera-relative
marker pose, not world-frame docking alignment. Tested with Python 3.14,
OpenCV 4.14.0, and NumPy 2.5.4 in the repository's `.venv`.

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
Confidence and tracking IDs remain null. Pose uses iterative planar PnP with
positive-depth and reprojection checks (default maximum RMS error: 3 pixels).
This is a clean-scene baseline: planar pose ambiguity is not resolved using
temporal information, and a low reprojection error does not guarantee accurate
pose. Calibration ID and resolution must match. Corner order is top-left,
top-right, bottom-right, bottom-left in the decoded marker's orientation.

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
reports. Persistent tracking remains future work.

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
multi-object benchmark scenes remain an extension. Default seed 2026 is the
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
states, Motion APIs, or coordinator behavior. Shared message/ENU integration and
retained-versus-predicted tracking semantics remain on hold pending agreement.

### Shared DetectionResult integration status

The current `DetectionResult` class is an internal camera-relative detection,
not yet the finalized shared system message. `CVObservation.to_dict()` supplies
`valid`, `reason`, and `source_id` at frame level. `pose_valid`/`pose_reason` are
separate: a valid image detection does not imply a usable metric pose.

The shared-message adapter remains to be implemented:

- Publish integer simulation `timestamp_ns` using the explicit clock mapping,
  retaining original source time. Current generic output keeps source time at
  top level and optional simulation time under `mapped_time`.
- Transform position and orientation from camera optical coordinates to local
  ENU using calibrated camera extrinsics and capture-time vehicle pose (or a
  known camera-to-world transform). Publish yaw in degrees, zero east and
  counterclockwise positive. Do not relabel the existing camera coordinates.
- Map internal `type` and envelope `camera_id` to shared `object_type` and
  `source_camera` if these exact names are required, preserving the internal API.
- Include timestamp, camera/source identity, and validity/reason in the agreed
  shared envelope or each standalone object message; define missing-transform
  behavior explicitly. Never substitute zero position for unavailable geometry.

Confidence and tracking ID fields exist but remain null. Persistent tracking is
still pending. The shared interface write-up should cover purpose, publisher,
subscribers, fields, units/frames, rate, validation, and example payloads once
the adapter contract is agreed. The local publisher callback is not a uXRCE-DDS
publisher; Phase 1 transport and message/IDL bindings are not implemented here.


## Phase 0 perception roadmap

The first implementation slice above is complete; the remaining work below is
planned. CV answers **what the camera sees**:
acquisition, detection, camera-relative pose, object tracking, and observations.
Action selection remains with Navigation, Docking, Safety, and the coordinator.
Preserve the existing fixture API while developing the image-based pipeline.

```mermaid
flowchart LR
    A[Camera provider] --> B[Preprocessing]
    B --> C[Detection]
    C --> D[Pose estimation]
    D --> E[Tracking]
    E --> F[DetectionResult publisher]
```

### Providers and detection contract

Use a common provider interface (`read() -> FrameSample`, `close()`) for synthetic
images and recorded video, with physical cameras supported later. Samples carry
pixels, frame number, source/camera identity, capture timestamp and clock,
validity/reason, and calibration identity. Keep missing input, processing failure,
valid empty frames, and finite-source EOF distinct. Preserve source timestamps;
the coordinator owns clock mapping and scheduling.

Prefer the existing `CVObservation` interface and serialized field names wherever
they convey the required information. Extend it additively for perception; do not
replace it or introduce aliases solely to match the suggested terminology.

| Suggested information | Current interface | Decision |
| --- | --- | --- |
| Timestamp | `timestamp_ns`, `clock`, optional mapped simulation time | Keep; already explicit about units and clock |
| Source camera | `camera_id` | Keep; no `source_camera` alias needed |
| Confidence | Nullable `confidence` in the observation and `to_dict()` | Keep; currently unset by the fixture reader and omitted from `to_bird_snapshot()` |
| Object type | `bird_type`, serialized as `bird.type`; snapshot uses `type` | Keep for bird observations; use `type` for future generic detection entries |
| Object ID | None; `source_id` identifies the input source | Add per-object identity; do not repurpose `source_id` |
| X/Y/Z position | Only fixture `distance_m` | Add camera-relative position; scalar distance cannot encode 3D location |
| Orientation | None | Add nullable orientation |
| Tracking ID | None; `frame_index` identifies a frame | Add persistent track identity |

The current interface therefore covers part of the suggested contract, not all
of it. Preserve its availability/status, provenance, scheduling, and legacy bird
snapshot semantics. The standalone image exporter already has a `detections`
list (`label`, `bbox_xywh`, `confidence`), but it contains prescribed metadata,
not measured pose or persistent identity, and is not the runtime observation API.

The implemented optional generic `detections` collection sits alongside the existing bird
fields, with processing status and stage timings at frame level. Keep timestamps
and `camera_id` in the shared frame envelope rather than duplicating them per
object. Existing fixture-only callers must continue to work; distinguish an
unpopulated generic result from a successfully processed empty list. Version any
serialized extension explicitly and test legacy adapters. Generic detections do
not populate the legacy bird fields automatically.

Proposed per-object additions reuse existing terminology where applicable:

| Field | Meaning |
| --- | --- |
| `object_id`, `type` | Initially dictionary/marker ID and `aruco_marker`; `type` follows the existing serialized convention |
| `confidence`, `confidence_method` | Nullable score and its meaning; do not invent a probability |
| `position_x`, `position_y`, `position_z` | Marker-center translation in camera-relative metres; null without valid pose |
| `orientation` | Quaternion `{x,y,z,w}` rotating marker-frame vectors into camera-frame vectors |
| `tracking_id` | Camera/session-local track instance, distinct from marker identity |
| `corners_px` | Ordered image corners |
| `pose_valid`, `pose_reason`, `reprojection_error_px` | Pose validity and quality, independent of detection success |

Use camera axes x right, y down, z forward; marker axes x right, y up, z out of
the printed face. Record the solver's corner order. Metric pose requires camera
intrinsics, distortion, resolution, and physical marker size. Resizing/cropping
must preserve calibration consistency. A detection without calibration retains
its corners and identity with null pose. Missing quantities are null, not zero.

Camera coordinates require an explicit calibrated transform before use in the
integration loop's ENU frame. An ArUco detection must never become a bird
observation. Replacing synthetic docking alignment requires a validated adapter.

### Synthetic datasets and evaluation

Generate seeded scenes with calibrated marker geometry and separate ground-truth
manifests. The detector and tracker receive pixels, never truth metadata. Include
clean scenes, marker-free negatives, clutter, partial occlusion, variable lighting,
motion blur, rotation, scaling, and multiple markers. Save calibration, marker
sizes, poses, seeds, and degradation parameters for replay. Keep evaluation seeds
separate from tuning scenes.

Benchmark distance, rotation, lighting, projected scale, and occlusion. Physical
range requires known marker size and calibration; pixel resizing alone cannot
establish metres. Match predictions to truth one-to-one with declared identity
and image-position tolerances, counting duplicates as false positives. Report:

- Detection rate `TP/(TP+FN)`, precision, FP/FN counts, and false positives per
  negative frame, with sample counts and uncertainty intervals.
- X/Y/Z bias, MAE, RMSE, and p95 error; yaw/pitch/roll errors with a declared Euler
  convention and wrapped angles, plus quaternion angular error.
- Pose-valid coverage and failures separately from errors on successful poses.
- Maximum reliable tested range under a declared condition envelope and agreed
  detection/pose thresholds. Synthetic results do not establish hardware range.

### Tracking and instrumentation

Associate marker identities within each camera/session. Preserve track IDs over
a configured short gap, expire them using source time, and reset on source restart.
Flag duplicate marker IDs as ambiguous. Retained tracks must not appear as fresh
observations. Cross-camera association and prediction can wait.

Log every processed frame, including empty/failed frames, with timestamps, frame
number, detections, confidence, pose, track IDs, and processing times. Record run
configuration, software versions, calibration, dataset seed/manifest, and dropped
frames. Retain input media or a reproducible manifest for later investigation.

Measure acquisition, preprocessing, detection, pose estimation, tracking, and
publication with a monotonic performance clock. Publication timing covers the
local callback/queue operation; measure logging overhead separately. Report total
and stage p50/p95/p99 latency with machine, resolution, and sample counts.

### Implementation order and completion

1. **Implemented:** frame/detection contracts and synthetic/recorded providers.
2. **Implemented:** pixel-based ArUco detection, clean-scene calibrated pose,
   per-frame logs/timing hooks, and fixture compatibility tests.
3. **Implemented:** seeded single-marker degradation sweeps, saved-image replay,
   detection/pose benchmarks, run manifests, and aggregate latency reports.
4. **Remaining:** persistent tracking, multi-object benchmark scenes, and shared
   detection-versus-track message agreement.
5. **Remaining:** coordinator scheduling and world-frame adapters, retaining fixture tests.

Acceptance requires the same detector to process synthetic images and recorded
video, reproducible benchmark reports, and logs explaining failures without drone
hardware. Test pixel behavior, pose accuracy, lifecycle errors, tracking expiry,
and evaluation accounting. Changing truth metadata alone must not change output.

Defer bird species classification, ML models, threat classification, behavior
prediction, deterrence selection, and autonomous mission planning.
