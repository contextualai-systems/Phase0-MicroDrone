# Docking — synthetic integration milestone

[Shared integration contract](../INTEGRATION_README.md), especially Sections 4–8,
controls the interface and defaults. Docking owns only its lowercase local phase;
the central coordinator owns the mission, return latch, shared timers, command
selection/dispatch, and logging. Inputs and movement are synthetic for this
milestone. Physical charging, marker pose estimation, and moving-pad landing are
outside its scope.

## Interface

`docking.states` exports `DockingController`, `DockingConfig`, `DockingInput`,
`DockingUpdate`, `PadObservation`, and `compute_return_request`.

Call `controller.update(inputs, now_ns)` once per coordinator tick, using integer
simulation nanoseconds starting at zero. The injected clock must be nonnegative
and monotonic; a programmer supplying an invalid clock gets `ValueError`.
All positions use local ENU: x east, y north, z up, meters; yaw is degrees, zero
east, counterclockwise positive. The fixed pad must have z = 0.

Inputs include:

- The coordinator's `mission_state`, `flight_active`, bird absence/ordinary hover
  start times, and explicit retry/reset events. `hover_started_ns` excludes launch
  climb. Bird absence time clears on detection; invalid camera input follows the
  return policy instead of accumulating absence time.
- Fresh `VehicleFeedback` from `motion_engine.contracts`, preserving its original
  timestamp and command execution status.
- Fresh, valid `PadObservation` with a fixed ENU `Pose`.
- Fresh `SafetyAssessment` from `safety_layer.safety_policy`. Safety owns battery
  thresholds and action permissions. Missing or malformed permissions fail closed.
- Synthetic alignment validity, integer `alignment_timestamp_ns`, `x_error_m`,
  `y_error_m`, `yaw_error_deg`, and a source ID. Alignment error is pad minus
  vehicle position/yaw, never a CV bounding-box interpretation.
- Environment validity/time and the explicit boolean `cart_moving`.

The immutable output supplies `should_dock`, ordered return `reasons`, local
`phase`/`state`, `progress`, `landed_on_pad`, `failure`/`failure_reason`, a stable
transition `reason`, and a proposed Motion `Command` or null. Metadata uses
`schema_version = 1`, `clock = "simulation"`, and `synthetic = true`.
`controller.transitions` is an immutable snapshot of state-change records.

## Return policy and mission lifecycle

`compute_return_request` is separate from approach/descent permissions. During
active flight, it reports Safety's low-battery and tracking restrictions, invalid
or stale camera samples, 3 seconds of continuous valid bird absence, or 10 seconds
of ordinary hover. Unavailable tracking geometry alone causes hold first;
continued unavailability reaches the hover timeout. Grounded IDLE ignores return
requests. Low battery requests docking and does not itself abort a docking attempt.

The coordinator latches the first committed return and preserves its original
start time across retries. A reappearing bird may clear a current return request,
but cannot clear that coordinator-owned latch. Docking neither duplicates battery
thresholds nor owns/resets the overall 60-second return deadline.

Only `DOCKING_APPROACH` activates phases or produces targets. Mission entry starts
one attempt; repeated ticks/command renewals do not restart it. Leaving that mission
state deactivates Docking and clears its local failure/timers. A subsequent entry
starts a fresh local attempt. An explicit `retry_requested` event can also restart
an aborted local attempt after hazards clear; the coordinator must first perform
its ABORT recovery/hold handling. Neither path changes shared return context.

An explicit `reset_requested` event while active is accepted only with fresh,
grounded, disarmed, at-pad feedback, fresh stationary-cart inputs, and no Safety
abort/emergency. It clears the local phase/timers without moving the vehicle or
resetting mission context. Reset acceptance for the entire mission remains the
coordinator's responsibility.

## Section 8 phases

| Phase | Proposed action | Transition |
| --- | --- | --- |
| idle | None | Mission docking entry → approach |
| approach | GOTO_DOCK_APPROACH at configured altitude over pad | At position/altitude/yaw tolerance → align if alignment fresh, otherwise search |
| search | HOVER | Fresh valid alignment → align; search timeout → abort |
| align | GOTO_DOCK_APPROACH correcting horizontal position/yaw at approach altitude | Within horizontal/yaw tolerance and descent allowed → descend; lost alignment → search |
| descend | DESCEND_TO_DOCK to pad at z = 0 | Confirmed pad touchdown → complete; lost alignment, drift, or lost descent permission → abort |
| complete | No flight target; report landed_on_pad | Coordinator enters DOCKED and explicitly disarms |
| abort | No docking target; report failure | Coordinator enters ABORT; Safety supplies fallback |

Any active phase aborts on lost approach permission, an invalid required input,
Safety abort/emergency, or attempt timeout. Timers trigger at or beyond their limit;
search timeout wins over newly available alignment at the boundary. Attempt time
continues across search/align changes. A completed/aborted phase remains latched
until the documented exit, retry, or reset lifecycle.

Proposals reuse `motion_engine.contracts.Command` and `Pose`; they carry ENU frame,
issue/expiry times, source, reason, speed limits, and IDs stable across unchanged
renewals. Changed targets get a new ID. The coordinator must validate every proposal
and the active command with Safety, replacing an unsafe previous target before
Motion steps. A null docking target never means it is safe to leave an old descent
command running. Docking does not dispatch, disarm, or implement fallback commands.

## Defaults and confirmation

Defaults are centralized in `safety_layer/config.py` and injected through
`DockingConfig`: 0.25 m arrival/horizontal tolerance, 3° yaw tolerance, 5 m approach
altitude, 0.5 s observation age, 5 s search timeout, 30 s attempt timeout, and 0.2 s
command validity. Horizontal/vertical limits default to 5/2 m/s. These are synthetic
settings, not validated physical flight parameters.

Completion requires fresh valid vehicle/pad/environment feedback, grounded and
not airborne, settled ordinary landing status (`LANDED` with active `LAND` or
`DESCEND_TO_DOCK` command), vehicle and landing target at z = 0, horizontal pad
error within tolerance, stationary cart, and retained descent permission/alignment.
Arrival (`arrived`/`at_target`) or command acceptance alone cannot confirm landing.
Emergency landing cannot produce ordinary docking completion.

## Synthetic alignment and checks

`docking.alignment.calculate_alignment` accepts `LocalPosition(x_m, y_m, origin_id,
frame="local_enu", yaw_deg=...)` fixtures plus an integer `timestamp_ns`. Both
positions require the same named origin. It returns signed x/y offsets, wrapped
yaw error, horizontal distance, validity, source, and reason. Invalid/missing
positions, nonfinite numbers, mismatched origins/frames, or invalid times yield
unavailable alignment. Freshness is checked by the controller.

```bash
python3 -m unittest discover -s tests -p 'test_docking*.py' -v
python3 -m docking.run_docking --scenario success --log /tmp/docking-success.jsonl
```

`run_docking` is a module diagnostic replay of recorded inputs, not a second
mission loop. It does not move Motion, dispatch targets, or decide mission state.
Fixtures supply mission entry and recorded landing feedback. Other scenarios are
`missing`, `stale`, `low-battery`, and `cart-moving`. Low battery allows completion;
missing/stale alignment and cart motion abort descent. Logs include ENU,
nanoseconds, controller inputs, proposals, and outcomes. Production logging uses
the coordinator's shared logger; the diagnostic JSONL writer is for local checks.
`--log` overwrites the given file; its parent directory must exist. Without it,
records go to stdout. The outcome summary goes to stderr; a successfully replayed
abort fixture exits with code 0.

The tests cover phase transitions, blocked pads, yaw/horizontal drift, invalid or
stale inputs, timeout boundaries, return triggers, retries/reset, and actual
MotionStub feedback through touchdown followed by explicit disarming. Wiring
these values into the central coordinator and running the full mission/retry
scenarios remains integration-team work.

`tests/test_docking_hazard_cancellation.py` connects the real controller,
Safety assessment/command check, and Motion stub in a coordinator test fixture.
It injects cart motion, stale alignment, and revoked descent permission during
descent, both at altitude and inside arrival tolerance just above touchdown.
The checks require an accepted hold before that tick's movement, unchanged pose,
no landing completion, and a latched abort after the hazard clears. A negative
control confirms that a null proposal alone leaves the old descent running.

The fixture explicitly handles a Docking abort with STOP when Safety still
permits the active descent (Safety currently checks pad age separately from
alignment age). This verifies the required coordinator handoff; it does not wire
abort handling into `state_machine/run_loop.py` or create another mission loop.
