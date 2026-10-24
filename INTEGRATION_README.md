# Phase 0: Shared States and Synthetic Loop Integration

## Purpose and scope

This document gives every module team the tasks and contracts needed to build one working synthetic loop. It resolves the transition questions in [state_machine/README.md](state_machine/README.md) and defines the target behavior for this integration milestone. Existing implementation and module READMEs still need to be brought into agreement with it.

**This milestone is purely synthetic. There is no physical drone.** Inputs come from deterministic fixtures, and vehicle movement comes from the kinematic Motion stub. No live sensors, hardware, PX4, ROS, Gazebo, or network transport are required. Synthetic images are allowed; bird observations, metric tracking geometry, and docking alignment remain explicitly synthetic inputs.

The broader semester goals in [START_HERE.md](START_HERE.md) remain background context. A waypoint mission or SITL integration must not block this smaller running-loop milestone. Future hardware integration will require its own interface and safety validation.

This is an implementation plan and shared contract, not a description of completed functionality. This change adds documentation only.

## 1. Who owns the running loop?

**The state-machine team should own the central coordinator and full running loop. The transition function itself should remain separate and pure.**

| Component | Owns | Does not own |
| --- | --- | --- |
| Central coordinator, under `state_machine/` | Constructing module instances; calling them in order; simulation clock; mission context; assembling snapshots; dispatching approved commands; publishing results and logs | Detection, duplicated safety policy, or docking geometry |
| Mission transition function, in `state_machine/state_machine.py` | Next mission state and reason from the current state and validated snapshot | Class construction, scheduling, sleeps, logging, movement, or mutation of inputs |
| Module controllers | Their observations, decisions, progress, or proposed targets | Independent mission loops or changing another module's state |

The coordinator receives results from each class and publishes one complete result per tick. For this milestone, “publish” means expose an immutable local snapshot and append a JSON Lines record. No message broker is needed.

Only the coordinator persists mission state and shared mission context. Docking owns its internal phase; Motion owns its synthetic vehicle pose. Modules return values to the coordinator rather than calling one another's controllers directly.

Suggested future files are `state_machine/run_loop.py`, `state_machine/contracts.py`, and `state_machine/logging_format.py`. These are implementation targets, not existing runnable commands. Do not introduce a second `mission/` package with competing responsibilities.

## 2. Universal state ownership

Keep the eight existing public mission-state names. Treat detailed docking phases and launch progress as subordinate execution status, not competing mission states.

| Owner | State/status | Meaning |
| --- | --- | --- |
| Mission | `IDLE` | Grounded and disarmed at the pad, waiting for launch eligibility |
| Mission | `HOVERING` | Establishing or maintaining a safe airborne hold; includes an explicit internal launch phase |
| Mission | `TRACKING` | Airborne following a synthetic bird target |
| Mission | `DOCKING_INIT` | Return committed; holding while checking permission to approach |
| Mission | `DOCKING_APPROACH` | Docking controller executing approach, alignment, and descent |
| Mission | `DOCKED` | Confirmed touchdown within pad tolerance; disarming and awaiting synthetic battery recovery |
| Mission | `ABORT` | Current action cancelled; establishing hold/disarm and deciding recovery |
| Mission | `EMERGENCY_LAND` | Latched synthetic landing in place; remains latched after touchdown |
| Docking | Existing lowercase `DockingState` values | Local docking progress only |
| Coordinator | Launch phase: `none`, `arming`, `climbing`, `complete` | Progress from launch authorization to an airborne hold |
| Motion | Command type, accepted/rejected, active target, armed/airborne/grounded | Execution status; never a replacement mission enum |

### Important alignment decisions

- `HOVERING` now explicitly includes launch preparation/climb. This refines its earlier README meaning without renaming the public enum. `IDLE -> HOVERING -> TRACKING` ensures tracking does not begin while grounded.
- Docking `idle` means its controller is inactive, not that the vehicle is grounded.
- Docking `complete` reports confirmed pad landing; the mission machine then enters `DOCKED`.
- Docking `abort` reports a local failure; the mission machine selects the system response.
- Navigation's proposed `TAKEOFF`, `WAYPOINT`, `RETURN`, `ALIGN`, and `LAND` mission states are deferred. Navigation generates tracking targets for this milestone.
- Safety actions such as hold or land are overrides/permissions, not another mission-state machine.
- Serialize mission states using uppercase enum values and docking phases using their existing lowercase values. Store them in separate fields.

## 3. One centralized tick

Default update rate: **10 Hz**, with a fixed **0.1-second** simulation timestep. Wall-clock pacing is optional and must not change the result.

1. At simulation time `t`, apply scheduled fixture changes and operator events. Read the Motion feedback left by the previous tick.
2. Read CV and synthetic observations due at `t`. Validate required fields, ranges, timestamps, and freshness. Preserve source times.
3. Safety assesses observations and current mission context, returning permissions, restrictions, and overrides.
4. Docking evaluates return triggers and reports progress/failure for the current docking phase using current feedback.
5. The pure mission function selects at most one transition. The coordinator updates state-entry times, launch context, and return intent. Docking synchronizes its phase with the resulting mission state.
6. Navigation or Docking proposes the relevant target. The coordinator selects launch, hold, disarm, or emergency actions when required by the state.
7. Safety validates the concrete proposed command and the currently active command. Any override replaces the ordinary action in the same tick.
8. Dispatch the permitted command, record Motion's acknowledgement, and advance Motion exactly once by `dt`.
9. Publish the result with pre/post-motion feedback, then advance time to `t + dt`.

If the final command check or Motion acknowledgement reveals a new failure, record it immediately and pass it into the next mission decision. **Replace the unsafe previous target before Motion advances in the current tick.** Merely omitting a new command would allow the old target to continue. If the approved fallback cannot be accepted, stop the synthetic run and report failure.

Arm, takeoff, disarm, and emergency commands are entry actions or explicit lifecycle steps. Tracking targets may change every tick. Docking must not repeatedly restart approach while descending. Safety checks active targets every tick even if the target is unchanged.

CV samples may run at a different rate. Consume samples according to their fixture times, using the latest due observation with its original timestamp. Do not read one 15 Hz sample per 10 Hz tick and relabel its time. Fixture exhaustion is a controlled scenario end, distinct from camera failure.

## 4. Shared contracts

Use one versioned contract, `schema_version = 1`, with identical field names in Python values and serialized logs. A required missing field produces a validation result, not a default “safe” value.

### Conventions

- Simulation time is integer `timestamp_ns`, starting at zero, with `clock = "simulation"`. Use it for all timeouts and freshness checks. Wall time is optional logging metadata only.
- Source observations include `valid`, `reason`, `source_id`, `timestamp_ns`, and `synthetic = true`. Explicitly map CV fixture time to simulation time while retaining its original source metadata.
- Position uses local ENU metres: x east, y north, z up. The fixed pad and ground are at z = 0 for this milestone. Yaw and bearing are degrees, zero east, positive counterclockwise.
- Unavailable optional values are null. Invalid enums, strings used as booleans, non-finite numbers, future timestamps, and out-of-range battery values are invalid inputs.
- Reasons use stable strings such as `bird_lost`, `low_battery`, `cart_moving`, `stale_vehicle`, and `command_rejected`; optional explanatory text is separate.
- Keep observed controller inputs, requested targets, and evaluation truth separate. Decisions consume declared observations, not privileged evaluation truth.

### Snapshot returned to the mission function

| Section | Required information | Producer |
| --- | --- | --- |
| Envelope | Schema version, tick index, simulation timestamp, synthetic flag | Coordinator |
| `vehicle` | Pose, armed/airborne/grounded, active command ID and target, progress, timestamp, validity | Motion |
| `bird` | Detected true/false/null, type, distance, validity, sample time, source and camera ID | CV |
| `tracking_geometry` | Synthetic world-frame bearing and range, validity/time, associated bird sample/source | Synthetic world |
| `battery` | `percent` in [0, 100], validity and timestamp | Synthetic world |
| `environment` | People nearby, cart moving, weather hazard, communication status, camera status, validity/time | Synthetic world and CV |
| `dock` | Fixed pad pose, validity, alignment validity/time, horizontal and yaw error | Synthetic world / Docking |
| `safety` | Launch/tracking/approach/descent/hold permissions; emergency and abort decisions; low/recovered battery flags; ordered reasons | Safety |
| `docking` | `should_dock`, ordered reasons, phase, progress, `landed_on_pad`, failure/reason | Docking |
| `context` | Mission state, state-entry time, launch phase, last known flight-active flag, timers, return latch/reason, operator/reset events, pending command failure | Coordinator |

The coordinator's published result additionally contains the next mission state, transition reason, proposed/permitted commands, Safety decision, Motion acknowledgement, and post-step feedback. Evaluation truth belongs in a separately labeled `synthetic_truth` section.

Use **`battery.percent`** everywhere. Safety derives low, critical, and recovered decisions; other modules do not duplicate battery thresholds. The existing `Battery.percentage` spelling must be aligned during implementation.

A valid empty CV sample means detected is false. Missing camera input means detected is null and validity is false. These have different outcomes. Tracking requires valid detection plus corresponding fresh geometry; bounding boxes are not metric bearing or range measurements. Geometry is an explicit synthetic fixture input.

### Commands and acknowledgements

Each command includes ID, kind, mission state, optional target pose, frame, limits, issue time, expiry time, source, and reason. Safety returns the permitted command or a replacement with its decision/reason. Motion returns the command ID, acceptance/rejection, and reason; later feedback reports progress against that ID.

Acceptance does not mean completion. Reaching an approach target does not mean landing. An unexpected Motion clamp is a command failure requiring recovery/replanning, not successful execution of the original target. Safety and Motion receive the same configured limits.

## 5. Default configuration

These are deterministic synthetic-demo settings, not physical flight recommendations. Keep one configuration and record it in each run log.

| Setting | Default |
| --- | --- |
| Timestep | 0.1 s |
| Low / critical battery | At or below 20% / 10% |
| Launch / recovered battery | At least 40% |
| Continuous valid bird absence before return | 3 s |
| Ordinary hover timeout, excluding launch climb | 10 s |
| Launch timeout | 10 s from launch entry |
| Overall return deadline | 60 s from first committed return, including waits/retries |
| Docking-attempt / alignment-search timeout | 30 s / 5 s |
| Maximum age of required observations | 0.5 s |
| Command validity | 0.2 s; renewed by coordinator after Safety checks |
| Launch / dock approach altitude | 5 m / 5 m |
| Tracking standoff | 15 m |
| Position arrival / horizontal pad tolerance | 0.25 m / 0.25 m |
| Horizontal alignment / yaw tolerance | 0.25 m / 3 degrees |
| Grounded threshold | At or below 0.05 m; docking additionally requires landing settled at z = 0 |
| Horizontal / vertical / yaw speed limits | 5 m/s / 2 m/s / 90 degrees/s |
| Geofence radius / altitude ceiling | 150 m around pad / 30 m |
| Emergency descent speed | 1 m/s |

Timers trigger at or beyond their limit. Fixtures own battery drain/recovery; no charger is modeled. A second-launch scenario must explicitly supply recovery while docked.

## 6. Safety policy and return intent

### Priority and permissions

Safety evaluates the following in order and preserves all active reasons. The first applicable category chooses the response.

1. **Emergency:** During active flight, critical battery, invalid/stale vehicle or battery feedback, communication loss, weather hazard, overall return deadline, or inability to maintain hold requests synthetic landing in place. Motion's internal synthetic pose remains available to execute that fallback even when its published observation is invalid.
2. **Abort:** Operator abort, command rejection/expiration, launch timeout, docking failure/timeout, or loss of approach/descent permission during the applicable docking phase. Cancel the active target and hold if possible; disarm if grounded.
3. **Restrictions and return:** Low battery prohibits launch/tracking but allows docking. People or cart motion prohibit launch/tracking/approach/descent but allow hold. Front-camera failure prohibits launch/tracking but permits docking using independent valid pad/alignment inputs. Missing environment or pad data prohibits the actions requiring those fields. Invalid alignment prohibits descent.
4. **Proceed:** Permit only actions whose required inputs are valid/fresh and whose targets meet configured limits and state requirements.

Launch requires grounded, disarmed, at-pad feedback; battery at least 40%; valid detected bird and geometry; and no blocking condition. Tracking requires valid bird/geometry and no low-battery, people, cart, weather, communication, or camera restriction. Approach requires valid vehicle/pad/environment/battery data, stationary cart, no people, and no emergency. Descent additionally requires fresh valid alignment within tolerance. Hold requires valid vehicle feedback and no emergency condition.

An emergency condition on the ground inhibits launch and disarms; it does not initiate a landing maneuver. An already latched `EMERGENCY_LAND` remains latched. Safety must return explicit actions, never rely on stopping command publication to stop motion.

### Docking requests versus permission to land

Docking computes `should_dock` during active flight for low battery, people, cart motion, camera failure, other restrictions preventing tracking, three seconds of continuous valid bird absence, or ordinary hover timeout. Missing tracking geometry causes immediate hold; if geometry remains unavailable, hover timeout requests return. Weather and communication faults are return causes too, but emergency policy takes precedence.

The coordinator latches return intent when selected. A reappearing bird cannot cancel a committed return. Keep its original start time across aborts and retries so persistent failures cannot restart the deadline forever.

Bird loss immediately stops tracking and starts hold. A fresh detection before the three-second delay can resume tracking if permitted. Invalid camera input requests return immediately rather than being counted as valid absence.

Grounded `IDLE` ignores return requests. Bird absence does not prevent `DOCKED -> IDLE` after battery recovery. Return intent, approach permission, descent permission, and launch eligibility are separate decisions.

## 7. Resolved mission transition table

Check global rules first, then the current state's rows in listed order. Select at most one transition per tick; otherwise remain in the current state. Safety supplies permissions/overrides and Docking supplies return intent/progress; the mission function consumes those results.

An active flight includes an accepted launch sequence or airborne feedback. If feedback becomes invalid, retain the coordinator's last known flight-active flag until valid grounded feedback or explicit reset. Missing data must not make a flight appear grounded.

### Global rules

| From | Condition | Result |
| --- | --- | --- |
| `EMERGENCY_LAND` | Explicit reset, fresh grounded/disarmed at-pad feedback, and no emergency condition | `IDLE`; clear mission context |
| `EMERGENCY_LAND` | Otherwise | Stay latched; continue descent if airborne |
| Any other state | Airborne, or flight-active with unknown grounding, and emergency required | `EMERGENCY_LAND` |
| `HOVERING` during launch | Confirmed grounded and launch permission revoked | `ABORT`; cancel launch and disarm |
| Any other state except `ABORT` | Abort required | `ABORT`; replace active target this tick |
| Any flight state | Unexpected grounding without confirmed pad landing | `ABORT`; disarm |

Flight states are `HOVERING`, `TRACKING`, `DOCKING_INIT`, and `DOCKING_APPROACH`. Expected grounding during initial arming is excluded. Launch permission revoked before lift-off cancels launch through `ABORT`, rather than starting a return flight. If already airborne, use return/emergency policy. Confirmed docking touchdown is handled below. Emergency wins over abort or completion on the same tick.

### Per-state rules

| From | Condition in priority order | To / behavior |
| --- | --- | --- |
| `IDLE` | Eligible bird and launch permitted | `HOVERING`; begin internal arming/climb sequence |
| `IDLE` | Otherwise | Stay grounded/disarmed |
| `HOVERING` | Airborne and either return latched or return requested | `DOCKING_INIT`; latch return and cancel remaining climb |
| `HOVERING` | Launch complete or already established airborne hold, valid bird/geometry, tracking permitted | `TRACKING` |
| `HOVERING` | Otherwise | Continue permitted launch or hold |
| `TRACKING` | Return latched or requested | `DOCKING_INIT`; latch return |
| `TRACKING` | Bird absent, geometry unavailable, or tracking forbidden | `HOVERING`; hold immediately |
| `DOCKING_INIT` | Approach permitted | `DOCKING_APPROACH`; start local `approach` phase |
| `DOCKING_INIT` | Approach blocked | Stay and hold; preserve overall return deadline |
| `DOCKING_APPROACH` | Docking failure or applicable permission lost | `ABORT` |
| `DOCKING_APPROACH` | Confirmed `landed_on_pad` | `DOCKED`; cancel flight target and disarm |
| `DOCKED` | Disarmed, battery recovered, fresh grounded at-pad feedback | `IDLE`; clear flight context |
| `DOCKED` | Otherwise | Stay grounded, finish disarming, await fixture recovery |
| `ABORT` | Grounded/disarmed at pad, failure cleared, operator abort acknowledged if present | `IDLE` |
| `ABORT` | Grounded away from pad | Stay disarmed until an explicit fixture reset restores at-pad conditions |
| `ABORT` | Airborne, hold permitted, at least one tick of abort handling completed | `DOCKING_INIT`; latch return and reset local docking phase |
| `ABORT` | Otherwise | Stay while establishing hold/disarm; emergency rule applies if hold is impossible |

Launch completes only after Motion accepts the takeoff command, reaches launch altitude within tolerance, and reports airborne. Do not begin tracking while climbing. The internal launch phase distinguishes this from a hold entered after interrupted tracking.

An operator abort is an edge-triggered event. It commits return while airborne and requires an explicit acknowledgement before a grounded restart. A recoverable airborne abort returns to docking, never directly to tracking. On recovery, persistent hazards keep the mission in `DOCKING_INIT` until clear or until the return deadline forces emergency descent.

Consume each command-failure event once when entering `ABORT`; retain it in the log without treating the same acknowledgement as a new failure every tick. Leaving `DOCKING_APPROACH` deactivates its controller and clears the local failure for the next attempt. Persistent input hazards are assessed again each tick. Cancel launch progress and its timeout when leaving the launch sequence, so an old launch timeout cannot repeatedly abort a later return.

Emergency touchdown does not release the mission latch, even if Motion auto-disarms. A landing away from the pad requires an explicit scenario operation restoring the vehicle to the pad before reset can succeed; reset must not silently teleport it.

### Timer lifecycle

- Bird-loss time starts on the first valid absent sample and clears on valid detection. Invalid input follows failure policy.
- Ordinary hover time starts when the launch climb completes and holding begins, or when tracking is interrupted; it clears on leaving `HOVERING`.
- Launch time starts once at launch entry, not on each command renewal.
- Return time starts on the first committed return and survives waits/aborts/retries.
- Docking-attempt time starts on each `DOCKING_APPROACH` entry; search time starts on local `search` entry.
- Clear flight timers/latches on confirmed docking followed by `IDLE`, grounded abort recovery to `IDLE`, or an accepted explicit reset.

## 8. Docking phase table

Only run these phases while mission state is `DOCKING_APPROACH`. Elsewhere Docking may evaluate return requests but must not issue flight targets.

| Local phase | Proposed action | Next phase / result |
| --- | --- | --- |
| `idle` | None | On mission docking entry, initialize `approach` |
| `approach` | Move to configured altitude above fixed pad | At approach position/yaw tolerance: `align` if alignment valid, otherwise `search` |
| `search` | Hold above pad, wait for synthetic alignment | Fresh valid alignment: `align`; search timeout: `abort` |
| `align` | Correct horizontal position/yaw at approach altitude | Within alignment tolerances and descent permitted: `descend`; lost alignment: `search` |
| `descend` | Move to pad at ground height | Confirmed pad touchdown: `complete`; lost alignment/descent permission: `abort` |
| `complete` | Report `landed_on_pad`; no flight target | Mission enters `DOCKED` |
| `abort` | Report failure; no docking target | Mission enters `ABORT`; Safety supplies fallback |

Loss of approach permission or docking-attempt timeout aborts from any active phase. The attempt timer continues across repeated `search`/`align` changes. `search` is a fixture wait, not marker detection.

Landing confirmation requires fresh vehicle feedback, grounded status, a settled landing command at z = 0, horizontal pad error within tolerance, and a stationary cart. **Motion's `at_target()` alone cannot confirm landing** because its arrival tolerance can become true while still airborne. Ordinary docking explicitly disarms after confirmed touchdown.

Cart motion is a hazard fixture. Keep the pad fixed for this milestone; block pad descent when cart motion is active. Moving-platform landing is excluded.

## 9. Tasks for each module team

### State machine and integration

- Keep the eight enum values and implement Section 7 with stable reason codes.
- Replace the current lost-bird `TRACKING -> IDLE` behavior with the specified hold/return sequence.
- Keep transition selection pure; pass timers, permissions, and progress as inputs.
- Implement the coordinator, launch lifecycle, return latch, reset lifecycle, shared validation/types, and logging.
- Handle invalid observations without an unhandled exception or accidental launch.
- Update the state-machine README to match this contract during implementation.
- Deliver table-driven transition tests plus the integrated runner and one verified run command.

### CV

See the [CV roadmap](cv/README.md#phase-0-perception-roadmap) for planned image-based
perception, detection contracts, and benchmarks. The tasks below cover the current
synthetic mission-loop adapter.

- Return valid detection, valid absence, and unavailable input distinctly through the existing observation interface.
- Preserve camera/source identity and original fixture timestamps; support the coordinator's explicit clock mapping.
- Keep bird observations separate from independent synthetic alignment inputs.
- Report finite scenario exhaustion distinctly from camera failure.
- Deliver fixtures for bird appearance, brief/prolonged absence, missing frames, and stale samples; no mission decisions or motion calls.

### Safety layer

- Replace unconditional approval with the policy and priorities in Section 6.
- Return action-specific permissions and stable reasons; own battery, freshness, timeout, and limit decisions.
- Validate concrete commands and active targets; produce explicit hold/emergency replacements.
- Inject simulation time instead of using wall time for decisions.
- Deliver tests for simultaneous hazards, stale/invalid inputs, expired commands, and cart motion during descent.

### Docking

- Keep the existing `DockingState` enum and implement Section 8 as a subordinate controller.
- Implement return-request policy separately from permission to approach/descend.
- Return progress/failure and proposed targets to the coordinator; do not build an independent mission loop in `run_docking.py`.
- Use synthetic pad/alignment fixtures with validity, age, units, and frame.
- Confirm touchdown and tolerance before reporting completion; preserve overall return context across attempts.
- Deliver phase-transition, blocked-pad, stale-alignment, timeout, and landing-completion tests.

### Navigation

- Generate tracking targets from declared synthetic geometry, current pose, configured standoff, and limits.
- Return targets; do not call Motion or change mission state.
- Defer the separate waypoint-state proposal for this milestone and update the module README accordingly.
- Deliver deterministic target tests, including unavailable geometry and a bird inside the standoff distance.

### Motion engine

- Adapt the existing `MotionStub` to the shared command/acknowledgement contract.
- Expose pose, grounded/airborne/armed, target progress, and rejection/clamping separately.
- Support replacing active targets before the next movement step; reject ordinary commands during emergency descent.
- Use the shared simulation clock and limits; keep emergency descent idempotent.
- Distinguish command acceptance, arrival tolerance, settled landing, and disarming.
- Update logger references/tests from the missing `mission.logging_format` package to the agreed shared location.
- Deliver approach/descent, cancellation, ordinary disarm, and emergency-latch interaction tests without hardware dependencies.

### Simulation / synthetic fixtures

- Supply deterministic world inputs, tracking geometry, pad/alignment values, hazards, battery changes, and operator events.
- Associate geometry with the relevant bird observation; keep evaluation truth separate.
- Support scheduled changes during a running loop and log the exact injection tick.
- Supply explicit battery recovery and reset events for repeat-cycle scenarios.
- Deliver nominal, blocked-dock, emergency, invalid-input, and repeated-retry scenarios; document expected transitions.

### Runtime and repository documentation

- Keep the only mission loop in the state-machine coordinator. Any future `src/` entry point should delegate to it.
- Establish a reproducible dependency/test setup; the active environment used for the review lacked `pytest`.
- Remove claims of working entry points until verified. `docking/run_docking.py` is currently empty, and the shared logger/coordinator are missing.
- Have each module README link to this contract and describe only its owned states, inputs, outputs, and runnable checks.
- Keep future transport/hardware notes separate from requirements for this synthetic milestone.

## 10. Central logging and review evidence

Use one shared logger supplied by the coordinator. Record scenario ID, schema version, effective configuration, and random seed if applicable at run start.

Each tick records simulation time, previous/next mission state, transition reason, docking phase, launch phase, input validity/ages, return context, requested/permitted command, Safety reasons, Motion acknowledgement, and pre/post-motion feedback. Keep evaluation truth separately labeled.

Log every transition, override, rejected command, and final outcome. Event logs may summarize unchanged holds; the tick stream must retain enough information to reproduce decisions. Distinguish completed, interrupted, and failed scenarios.

## 11. Integration sequence and definition of done

1. Establish shared contracts, configuration, clock, validation, logger, and test dependencies.
2. Implement Safety and the pure mission table using handcrafted snapshots.
3. Implement Docking phases, Navigation target generation, and Motion adapters against those contracts.
4. Integrate the coordinator with simple fixtures, then attach CV through the clock adapter.
5. Demonstrate the full nominal cycle, failure injections, battery recovery, and a second launch.

Nominal mission trace:

`IDLE -> HOVERING (launch) -> TRACKING -> HOVERING (bird loss) -> DOCKING_INIT -> DOCKING_APPROACH -> DOCKED -> IDLE`

Docking logs separately show approach, optional search, alignment, descent, and confirmed completion.

Required acceptance scenarios:

- Every mission and docking-phase transition, default stay, and priority conflict has a passing test.
- Unsafe launch remains grounded; tracking waits for takeoff completion.
- Brief bird loss can resume tracking; prolonged loss commits return; a reappearing bird cannot cancel committed return.
- Low battery requests return; critical battery takes precedence and lands synthetically in place.
- A blocked dock stays in `DOCKING_INIT` without oscillation; cart motion during descent cancels descent before that tick's movement.
- Missing/stale camera, geometry, alignment, environment, vehicle, and battery inputs follow their specified outcomes.
- Search/attempt timeouts abort; repeated retries cannot reset the overall return deadline.
- Rejected/expired commands cannot leave an unsafe previous target active.
- Arrival above ground cannot produce `DOCKED`; landing and ordinary disarming are verified separately.
- Synthetic battery recovery permits `DOCKED -> IDLE` even with no bird present.
- Emergency remains latched after touchdown and only the documented reset releases it.
- Malformed observations yield defined validation/fallback behavior instead of uncaught transition errors.
- Identical fixtures/configuration reproduce the same transition and pose traces at different wall-clock execution speeds.

Completion means a second team member can run the documented synthetic loop from a clean setup, reproduce the cycle and failure scenarios, and inspect logs explaining every transition and command. This demonstrates software behavior in the synthetic model, not physical flight safety.
