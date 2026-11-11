# Motion Engine — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md) · [Shared integration contract](../INTEGRATION_README.md)

## Current milestone: the synthetic loop

For the integration milestone there is no drone and no PX4 in the loop.
Motion's job is the kinematic stub, [`motion_stubs.py`](motion_stubs.py),
driven by the central coordinator through the contract in
[INTEGRATION_README.md](../INTEGRATION_README.md) §3-§5. Motion owns the
synthetic vehicle pose and its execution status, and never a mission state.

### Contract (`motion_engine/contracts.py`)

| Type | Direction | What it carries |
|---|---|---|
| `Command` ([full contract](#motioncommand-contract)) | coordinator → Motion | `command_id`, `kind`, `mission_state`, `target` (`Pose`, local ENU) **or** `velocity` (`Velocity`, for `SET_VELOCITY`), `frame`, optional speed caps, `issued_ns`, `expires_ns`, `source`, `reason` |
| `CommandAck` | Motion → coordinator, immediately | `command_id`, `kind`, `accepted`, stable `reason`, `detail`, `active_command_id` (what is still flying after this), `timestamp_ns` |
| `DroneState` (alias `VehicleFeedback`) | Motion → snapshot `vehicle` section | position/pose, velocity, orientation, frame, flight mode, `armed`, `airborne`, `grounded`, `emergency_descent`, `landing_settled`, active command ID/kind/target, `status`, `arrived`, errors, `timestamp_ns`, `valid`. Full contract [below](#dronestate-contract) |

One coordinator tick:

```python
from motion_engine.contracts import Command, MotionCommandType as K, Pose, SimulationClock
from motion_engine.motion_stubs import MotionStub

clock = SimulationClock()                 # integer ns, starts at 0
motion = MotionStub(logger=shared_logger, clock=clock)

ack = motion.submit(command, clock.timestamp_ns)   # step 8: dispatch, record ack
motion.step(0.1, now_ns=clock.timestamp_ns)        # advance Motion exactly once
vehicle = motion.feedback(clock.timestamp_ns)      # post-motion feedback
clock.advance(0.1)
```

Rules Motion enforces:

- **Accepted is not done.** `status` goes `in_progress` → `arrived`, and for
  landing commands → `landed`. `arrived` uses the 0.25 m arrival tolerance and
  can be true while still airborne, so docking must check `landing_settled`
  (z exactly 0 after a landing command), never `arrived`.
- **Expiry.** Every command except arm/disarm/takeoff/land/emergency needs
  `expires_ns`. Re-sending the active `command_id` with the same request and
  an unexpired `expires_ns` renews it: no restart, no new log line.
  Field-by-field rules: [MotionCommand contract](#motioncommand-contract). If it expires, Motion replaces it with a
  hold at the current pose *before* moving that tick (`status = expired_hold`).
  The one exception is a landing that has already settled exactly on its
  target: it stays `landed` / `landing_settled`, because there is nothing left
  to hold. `landing_settled` needs z = 0 *and* x/y on the target, so a
  descent that touches down short of the pad (horizontal and vertical speeds
  are capped separately) is not settled and holds like any other command.
- **Comm-loss landing.** An expired hold is not forever. If Motion stays in
  `expired_hold` with no newly accepted command for
  `MotionLimits.comm_loss_land_timeout_ns` (10 s, shared with Safety as
  `COMM_LOSS_LAND_TIMEOUT_NS` in `safety_layer/config.py`, read through
  `from_config()`), it lands
  straight down where it is through the ordinary `LAND` path, before that
  tick's movement. Feedback reads `status = comm_loss_landing`,
  `flight_mode = land`, and `status_reason = reason = comm_loss_landing`
  during the descent; after touchdown `status` becomes `landed` but
  `status_reason` and `reason` stay `comm_loss_landing` (until the next
  accepted command) so a returning coordinator knows the landing was
  Motion's (it is usually off the pad). An expired `SET_VELOCITY` follows
  the same path: its hold ends velocity mode, and the same timeout applies.
  Logged once as event `comm_loss_landing`, `decision = land_in_place`, with
  `held_s`, `timeout_s`, `since_ns` and the start/target pose. The timer
  starts when the hold starts, runs on simulation time only (`step()`
  without `now_ns` never escalates), and only an airborne, armed vehicle
  escalates: a settled landing stays `landed`, and a grounded or disarmed
  vehicle stays put. It is **not** a latch: any command `submit()` accepts
  cancels it (accepted already means valid, unexpired and through Safety, so
  the coordinator is back); late or replayed commands are rejected as usual
  and cannot cancel it. `EMERGENCY_LAND` still overrides it and latches. Like
  `LAND`, it does not auto-disarm; the coordinator disarms on `grounded`.
  10 s matches the ordinary hover timeout, is 50x the 0.2 s command
  validity (so a dropped renewal or a 5 s loop stall never triggers it), and
  even from the 30 m ceiling at 2 m/s puts the drone on the ground within
  25 s, inside the 60 s return deadline. Motion has no battery reading, so
  this bound on unattended hover is its only battery protection once Safety
  is unreachable. On PX4 this is the link-loss failsafe (`COM_OF_LOSS_T` ->
  `COM_OBL_RC_ACT`, `COM_DL_LOSS_T` -> `NAV_DLL_ACT`, both set to Land).
- **Replacement.** An accepted command replaces the active target
  immediately, so a hold sent this tick stops the drone this tick.
- **No silent clamps.** A target above the ceiling or outside the geofence is
  rejected (`target_outside_limits`), not clamped. The direct methods still
  clamp, for scripts.
- **Emergency latch.** After `EMERGENCY_LAND` every other command is rejected
  (`emergency_descent_active`), repeats are idempotent, emergency never
  expires, and touchdown auto-disarms.
- **Velocity control (`SET_VELOCITY`).** See [below](#velocity-command-set_velocity).
- **Bad input is a result, not an exception.** Missing target or expiry, NaN
  or inf, wrong frame or schema, future timestamps and reused IDs each come
  back as a rejected ack with a stable reason (`contracts.Reason`). A NaN,
  inf or non-numeric `dt` passed to `step()` is ignored and logged
  (`bad_dt`, `decision = ignored_bad_dt`).

### Velocity command (`SET_VELOCITY`)

Motion owns velocity control. A `SET_VELOCITY` command carries
`velocity=Velocity(vx, vy, vz, yaw_rate_deg_s=None)` instead of a `target`:

| Field | Unit / frame | Notes |
|---|---|---|
| `vx`, `vy`, `vz` | m/s, local ENU (east, north, up) | finite; `hypot(vx, vy)` ≤ horizontal limit (5 m/s), `abs(vz)` ≤ vertical limit (2 m/s), and ≤ the command's own `max_*_speed_mps` caps if given |
| `yaw_rate_deg_s` | deg/s, CCW + | optional; `None` keeps the current heading; `abs()` ≤ 90 deg/s |

```python
from motion_engine.contracts import Command, MotionCommandType as K, Velocity

cmd = Command(command_id="vel-1", kind=K.SET_VELOCITY, mission_state="TRACKING",
              issued_ns=now_ns, expires_ns=now_ns + 200_000_000,   # required
              source="navigation", reason="follow",
              velocity=Velocity(2.0, 1.0, 0.0, yaw_rate_deg_s=10.0))
```

- **Dead-man expiry.** `expires_ns` is required (`missing_expiry`). Renew by
  re-sending the same ID and velocity with a later expiry (`renewed`). If it
  is not renewed, the next `step()` replaces it with a hold in place before
  moving (`status = expired_hold`, `status_reason = command_expired`).
- **Rejections** (stable `Reason`, never an exception, never scaled down):
  `missing_velocity` (no `velocity`), `invalid_command` (NaN/inf, a `target`
  as well as a `velocity`, or a `velocity` on any other kind),
  `velocity_outside_limits` (too fast; `detail` says which axis),
  `not_airborne` (take off first; prevents sliding along the pad),
  `not_armed`, `emergency_descent_active`.
- **Boundary hold.** A velocity has no target, so Motion checks every step:
  if the next step would cross the altitude ceiling, the altitude floor or the
  geofence radius, the drone moves exactly to that boundary and holds there.
  Feedback reports `status = boundary_hold` with `status_reason` =
  `altitude_ceiling_boundary` / `altitude_floor_boundary` /
  `geofence_boundary`, and Motion logs `velocity_boundary`
  (`decision = hold_at_boundary`). The velocity command is then finished:
  renewing its ID is rejected (`command_id_reused`), so the coordinator must
  send a new command. This is a reported stop, not a silent clamp.
- **Emergency latch wins.** While `EMERGENCY_LAND` is latched, `SET_VELOCITY`
  is rejected (`emergency_descent_active`); an emergency during velocity mode
  replaces it immediately.
- **Safety permission.** Safety maps `SET_VELOCITY` to the `tracking`
  permission, the same as `MOVE_TO` and `TRACK_TARGET`, so velocity is
  blocked whenever tracking is (low battery, people nearby, and so on) and
  replaced with a HOVER.
- **Safety's replacements.** Safety clears the velocity when it substitutes
  a HOVER, LAND, EMERGENCY_LAND or DISARM, because Motion rejects a velocity
  on any other kind. `safety_adapter.check_for_motion()` clears it as well,
  as a second line of defence, so Safety's stop is always accepted even if
  that changes. It never changes Safety's decision. If a kind is ever left
  unmapped, Safety refuses it as `unsupported_kind` and holds; both cases are
  covered in `tests/test_safety_adapter.py`.

### DroneState contract

`motion_engine.contracts.DroneState` (also importable as `VehicleFeedback`,
the same class) is the formal vehicle-state contract required by
[START_HERE](../START_HERE.md) ("Vehicle state: timestamp, position,
velocity, orientation, coordinate frame, flight mode, validity").

**Compatibility: no downstream rename needed.** `VehicleFeedback` stays as a
permanent alias of `DroneState` (the same class object, not a subclass), so
`from motion_engine.contracts import VehicleFeedback`, keyword construction
without the new fields, and `isinstance(x, VehicleFeedback)` all keep
working. Docking (`docking/run_docking.py`,
`docking/states/docking_controller.py`) uses all three and passes unchanged.
Teams can switch to the `DroneState` name whenever they next touch that code.

1. **Purpose.** Motion's single, per-tick report of the vehicle's
   *estimated* state (where it is, how it is moving, what mode it is in) plus
   its progress on the active command. Estimated state and commanded values
   (`target`, `target_velocity`) are separate fields, never mixed.
2. **Publisher.** Motion Engine: `MotionStub.feedback(now_ns)`. In Phase 1
   it is filled from PX4 (`/fmu/out/vehicle_local_position`,
   `vehicle_attitude`, `vehicle_status`, `vehicle_land_detected`).
3. **Subscribers.** The coordinator (`state_machine/run_loop.py`, which puts
   `to_dict()` in the snapshot's `vehicle` section and reads `pose`,
   `armed`, `airborne`, `grounded`, `active_kind`, `target`), Docking
   (`DockingController`: `valid`, `landing_settled`, `grounded`, `status`),
   the waypoint executor (`arrived`, `active_command_id`), the trace checker,
   and the shared log.
4. **Fields.**

   | Field | Type | Unit / frame | Meaning |
   |---|---|---|---|
   | `schema_version` | int | | always `1` |
   | `timestamp_ns` | int | simulation ns, ≥ 0 | time this state is true |
   | `valid` / `reason` | bool / str or null | | validity; `reason` explains an invalid state |
   | `source_id` / `synthetic` | str / bool | | `"motion_stub"` / `true` in Phase 0 |
   | `frame` | str | | `"local_enu"`: origin at pad, x east, y north, z up |
   | `pose` (`position` property) | `{x, y, z, yaw_deg}` | m, deg (0 = east, CCW +) | estimated position and heading |
   | `velocity` | `{vx, vy, vz, yaw_rate_deg_s}` | m/s ENU, deg/s | estimated velocity (displacement over the last step) |
   | `orientation` | `{roll_deg, pitch_deg, yaw_deg}` | deg | attitude; roll = pitch = 0 in the kinematic stub, yaw = `pose.yaw_deg` |
   | `flight_mode` | str | | `disarmed`, `on_ground`, `takeoff`, `position`, `velocity`, `hold`, `land`, `emergency_land` |
   | `armed`, `airborne`, `grounded` | bool | grounded ⇔ z ≤ 0.05 m | |
   | `emergency_descent` | bool | | emergency latch active |
   | `landing_settled` | bool | | landing finished exactly at z = 0 on its x/y |
   | `active_command_id`, `active_kind` | str or null | | what Motion is executing |
   | `target` | `Pose` or null | m, deg | commanded position target |
   | `target_velocity` | `Velocity` or null | m/s, deg/s | commanded velocity setpoint (`SET_VELOCITY`) |
   | `status` | str | | `none`, `in_progress`, `arrived`, `landed`, `expired_hold`, `boundary_hold`, `comm_loss_landing`, `emergency_descent` |
   | `status_reason` | str or null | | `command_expired` for `expired_hold`; the limit for `boundary_hold`; `comm_loss_landing` during a comm-loss landing and after its touchdown (`landed`) until the next accepted command |
   | `arrived` | bool | 0.25 m / 3 deg | within arrival tolerance (not "landed") |
   | `distance_to_target_m`, `horizontal_error_m`, `vertical_error_m`, `yaw_error_deg` | float or null | m, deg | progress against `target` |

5. **Update rate.** Once per coordinator tick (10 Hz, 0.1 s simulation
   time), read after `motion.step()`. `feedback()` is read-only, so extra
   reads are harmless.
6. **Validation rules** (`DroneState.validate()` returns a stable `Reason`
   or `None`, never raises): `schema_version == 1`
   (`unsupported_schema`); `timestamp_ns` an int ≥ 0 (`invalid_timestamp`);
   `frame == "local_enu"` (`unsupported_frame`); `velocity`, `flight_mode`
   and `status` present (`missing_field`); pose, velocity, orientation,
   target, target velocity and error values finite, enums valid and flags
   real booleans (`invalid_state`); and consistency (`inconsistent_state`):
   exactly one of `airborne` / `grounded`, `landing_settled` implies
   `grounded`, airborne implies armed, `flight_mode == "disarmed"` exactly
   when not armed, `emergency_descent` exactly when `flight_mode ==
   "emergency_land"`, and `orientation.yaw_deg == pose.yaw_deg`. An invalid
   state must carry a `reason` (`missing_field`).
7. **Compatibility.** `to_dict()` keeps every key it had as
   `VehicleFeedback` and only adds `frame`, `velocity`, `orientation`,
   `flight_mode`, `target_velocity` and `status_reason`. The new constructor
   fields have defaults, so existing keyword construction still works;
   `validate()` reports a missing `velocity` / `flight_mode` as
   `missing_field` instead of inventing a value.
8. **Example payload** (`DroneState.to_dict()`, 0.1 s into a
   `SET_VELOCITY`; `DroneState.from_dict()` reads it back and
   `test_drone_state.py` checks it round-trips and validates):

```json
{
  "schema_version": 1,
  "timestamp_ns": 3100000000,
  "valid": true,
  "reason": null,
  "source_id": "motion_stub",
  "synthetic": true,
  "frame": "local_enu",
  "pose": {"x": 0.2, "y": 0.1, "z": 5.0, "yaw_deg": 1.0},
  "velocity": {"vx": 2.0, "vy": 1.0, "vz": 0.0, "yaw_rate_deg_s": 10.0},
  "orientation": {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 1.0},
  "flight_mode": "velocity",
  "armed": true,
  "airborne": true,
  "grounded": false,
  "emergency_descent": false,
  "landing_settled": false,
  "active_command_id": "vel-1",
  "active_kind": "SET_VELOCITY",
  "target": null,
  "target_velocity": {"vx": 2.0, "vy": 1.0, "vz": 0.0, "yaw_rate_deg_s": 10.0},
  "status": "in_progress",
  "status_reason": null,
  "arrived": false,
  "distance_to_target_m": null,
  "horizontal_error_m": null,
  "vertical_error_m": null,
  "yaw_error_deg": null
}
```

### Driven by the state machine

[`state_machine/run_loop.py`](../state_machine/run_loop.py) is the coordinator.
Each tick it calls `step()` for the mission state, sends the matching Motion
command through `submit()`, then calls `motion.step(0.1, now_ns)` and
`motion.feedback()`. The mapping (`wanted_command()`) belongs to the state
machine; this is what Motion sees and guarantees for each state:

| Mission state | Command Motion receives | What Motion does |
|---|---|---|
| `IDLE` | nothing | stays on the pad, disarmed |
| `HOVERING` (launch from `IDLE`) | `ARM` + `TAKEOFF` to 5 m, once each (`arm-N`, `takeoff-N`) | climbs in place at 2 m/s; takeoff has no expiry. The mission stays in `HOVERING`, and nothing else is sent, until the climb reaches the full 5 m |
| `HOVERING` (in the air) | `HOVER`, renewed every tick | holds the pose it had when the hover was accepted |
| `TRACKING` | nothing yet (no Navigation target) | keeps flying the last command, which is always a `HOVER` because `TRACKING` only starts after the launch climb has finished; no longer renewed, it expires into a hold in place, and lands in place after 10 s of that hold (comm-loss rule) |
| `DOCKING_INIT` | `HOVER`, renewed | holds |
| `DOCKING_APPROACH` | `DESCEND_TO_DOCK` to the pad, renewed | flies to the pad (x/y) while descending; `status` → `landed` and `landing_settled = True` once exactly on it |
| `ABORT` | `STOP`, renewed | holds where it is (mid-descent included) |
| `DOCKED` | `DISARM` once `grounded` and `armed` | disarms; still `landed` / `landing_settled` |
| `EMERGENCY_LAND` | `EMERGENCY_LAND`, once | descends in place at 1 m/s, latches, auto-disarms on touchdown |

Renewed commands keep their ID (`sm-N`) with a new 0.2 s expiry; a different
command gets the next ID. If the state machine stops renewing, the normal
expiry rule applies: a hold in place, unless the drone has already landed,
and a comm-loss landing in place if the hold lasts 10 s.

[`tests/test_state_machine_alignment.py`](tests/test_state_machine_alignment.py)
runs every demo in [`state_machine/demos.py`](../state_machine/demos.py)
through a recorded `MotionStub` and checks that Motion accepts every command,
the trace checker finds nothing, docking demos are `landing_settled` on the
pad *before* the state machine says `DOCKED`, the emergency demo lands
straight down, and the blocked-dock demo holds still. (In the demos,
`landed_on_pad` is scripted, not read from Motion, so "settled before
`DOCKED`" is a check that the scripts stay realistic.) If the state machine
changes how it drives Motion, that file is where it shows up.

Limits default to the INTEGRATION_README §5 table (`MotionLimits`): 5 / 2 m/s,
90 deg/s, 150 m geofence, 30 m ceiling, 0.25 m / 3 deg tolerances, 1 m/s
emergency descent. Pass the same `MotionLimits` that Safety uses.

Logging goes through any object with the agreed
`log(module, event, *, fields, state, truth, decision, t)` method
(`contracts.LogSink`). To write into the shared logger in
[`universal_log/Universal_log.py`](../universal_log/Universal_log.py), the same
one CV uses, pass `UniversalLogSink` from
[`universal_logging.py`](universal_logging.py):

```python
from motion_engine.universal_logging import UniversalLogSink

motion = MotionStub(logger=UniversalLogSink(), clock=clock)
motion.set_log_context(state=state.value)   # coordinator, every tick
```

Each record becomes `State` = the coordinator's mission state (`IDLE` until
one is set, as in CV) and `Details` = `module`, `event`, `t`, `fields`,
`synthetic_truth`, `decision` (the same keys as `LogEntry.to_dict()`).
UniversalLog's outer timestamp is wall-clock emission time; Motion's clock
reading stays in `Details["t"]` and is the one to use for ordering. CV and
Motion records land in the same `universal_log/logged_states_<run>.log`.
`contracts.MemoryLogSink` remains the in-memory sink for tests. Motion no
longer imports the old `mission.logging_format`.

**Integrated runs (follow-up, not a dependency).** `state_machine/run_loop.py`
still builds `MotionStub` without a logger, so Motion's records are dropped in
integrated runs, the same as before this change. The State Machine team has
been asked to add an optional
`run_world(..., motion_logger=None)` that passes the logger to `MotionStub`
and calls `set_log_context(state=...)` each tick. Motion needs no change for
that: `MotionStub(logger=...)` and `set_log_context()` already exist.

### Shared limits (`MotionLimits.from_config()`)

Speed, geofence, ceiling and tolerance numbers live in one place,
[`safety_layer/config.py`](../safety_layer/config.py). The coordinator builds
Motion's limits from it:

```python
motion = MotionStub(clock=clock, limits=MotionLimits.from_config())
```

`state_machine/run_loop.py` does exactly this. `MotionLimits()` defaults are
kept for scripts, and [`tests/test_limits_config.py`](tests/test_limits_config.py)
fails if they ever disagree with Safety's config, so change both or neither.

### Waypoint executor (`motion_engine/waypoints.py`)

Motion owns waypoint navigation as a *mechanism*; Navigation or the mission
layer decides *which* waypoints to fly. `WaypointExecutor` takes a list of
`Pose`s, validates the whole route against the shared limits before anything
flies, then issues one renewed `MOVE_TO` per waypoint through the normal
Safety -> `submit()` path and reports `WaypointProgress` (index, reached,
total, distance to the current waypoint, status, reason).

```python
from motion_engine.waypoints import WaypointExecutor
from navigation import build_route

executor = WaypointExecutor()                       # limits from Safety's config
executor.load("patrol-1", build_route([(20, 0), (20, 15), (0, 15)], altitude_m=5.0), now_ns)
# each tick:
cmd = executor.command(state.value, now_ns)         # None once complete/failed
if cmd is not None:
    cmd, _ = check_for_motion(cmd, assessment, now_ns)
    executor.on_ack(motion.submit(cmd, now_ns))
motion.step(0.1, now_ns=now_ns)
progress = executor.update(motion.feedback(now_ns))
```

- A waypoint counts as reached only when Motion reports `arrived` for the
  executor's own command ID. A Safety hold or an expiry is not arrival; the
  executor re-issues that waypoint under a new ID next tick.
- A rejected command fails the route with Motion's reason
  (`command_rejected`, detail `waypoint N: <reason>`). Retrying, holding or
  aborting is the mission layer's decision.
- After the last waypoint `command()` returns `None`; the coordinator chooses
  what comes next (normally `HOVER`).

**Open question for Jeff/team:** Jeff's Phase 0 notes list waypoint
navigation under Motion and START_HERE's semester demo includes waypoints, but
[INTEGRATION_README §2](../INTEGRATION_README.md) defers waypoint mission
states for the synthetic-loop milestone. The executor is therefore available
and tested but **not yet wired into `state_machine/run_loop.py`**. Once the
team agrees which document wins, wiring it in is a coordinator change only.

### Tracking targets come from Navigation

Motion no longer turns bird bearing/distance into a standoff pose (the old
`MotionStub.track_target()` was removed). Navigation computes the pose with
[`navigation.tracking.standoff_target`](../navigation/tracking.py) and the
coordinator sends it as a `TRACK_TARGET` command, which Motion flies via
`track_position()`. Motion never sees a bird.

### Runnable checks

```bash
py -3 -m pytest motion_engine/tests/test_motion_stubs.py motion_engine/tests/test_motion_contract.py motion_engine/tests/test_universal_logging.py
py -3 -m pytest motion_engine/tests/test_velocity_command.py motion_engine/tests/test_drone_state.py
py -3 -m pytest motion_engine/tests/test_waypoints.py motion_engine/tests/test_limits_config.py tests/test_navigation.py
py -3 -m pytest motion_engine/tests/test_state_machine_alignment.py   # Motion as driven by state_machine/run_loop.py
py -3 motion_engine/synthetic/generate_motion_data.py      # synthetic CSVs, see synthetic/README.md
```

`test_motion_contract.py` covers the README's Motion deliverables: approach
and descent, cancellation, ordinary disarm, renewal, rejected input, and
identical traces from identical inputs. Expiry and the emergency latch are
covered in every flight phase by `test_failure_injection.py` below.

### Failure injection

```bash
py -3 -m pytest motion_engine/tests/test_failure_injection.py -v
```

Injects the START_HERE "definition of done" failures into a running stub and
checks Motion's outcome for each:

| Section | Failures injected | Outcome checked |
|---|---|---|
| A. Command loss | renewals stop mid-move/track/approach/yaw/descent; one dropped renewal; late renewal after expiry; lifecycle and emergency commands with no renewal | holds within one 0.2 s validity window and never moves again; one dropped tick is tolerated; an expired ID cannot be revived; takeoff/land/emergency still complete |
| B. Loop stalls / clock | 1-5 s coordinator stall (incl. one 5 s `dt`); clock running backwards | expires *before* moving; a backwards clock cannot revive a target |
| C. Stale / replayed | late already-expired command; replayed old ID; same ID with a new target | rejected with a stable reason, current target keeps flying |
| D. Geofence | bird walking out of the fence; ceiling and fence edges; out-of-limit target through Safety | never leaves the fence; Safety's hold applies before that tick's movement |
| E. Emergency | `EMERGENCY_LAND` during climb, tracking, approach, descent, hover; on the pad; disarmed | descends in place at 1 m/s, latches, auto-disarms |
| F. Safety hazards | cart moving during descent; hold forbidden; low battery; critical battery mid-approach | descent frozen that tick and resumed under a new ID; lands in place; return still allowed; emergency beats docking |
| G. Garbage input | 19 kinds of malformed command submitted every tick | the flown path is identical to a clean run |
| H. Not a landing | link lost 0.2 m above the pad (inside arrival tolerance) | never `landing_settled`, disarm refused |
| I. Bad `dt` | 0, negative, NaN, inf, non-numeric | no movement; NaN/inf/non-numeric are logged `ignored_bad_dt`; expiry still runs, and the next good tick resumes normally |
| J. Comm-loss landing | link never comes back; comes back at 9.5 s; comes back mid-descent; late/replayed traffic; emergency mid-descent; settled on pad; armed on the ground; disarmed; 3 s stall with a 2 s timeout | holds exactly 10 s then lands straight down (`comm_loss_landing`); a fresh accepted command resets the timer or cancels the landing, stale ones cannot; emergency still latches; landed/grounded/disarmed never escalate |

Every scenario is recorded and run through the trace checker below, so no
test can pass while breaking a speed, fence or latch rule along the way.

### Trace checker

[`trace_checker.py`](trace_checker.py) walks a whole run tick by tick and
checks 15 rules from INTEGRATION_README §5/§8: finite pose, monotonic time,
ground/ceiling/geofence, horizontal/vertical/yaw rate limits, no motion
while disarmed, airborne/grounded flags, no landing above the ground,
emergency descent in place at 1 m/s, latch release only on the ground, no
motion after expiry, and that rejected commands never change the active
command. It also reports path length, peak speeds, time airborne, landing
error from the pad, and rejected commands by reason.

```bash
py -3 -m motion_engine.trace_checker --demo                          # record + check a nominal mission
py -3 -m motion_engine.trace_checker --demo --save motion_engine/motion_logs/demo.jsonl
py -3 -m motion_engine.trace_checker motion_engine/motion_logs/*.csv  # the synthetic CSVs (pose-only rules)
py -3 -m motion_engine.trace_checker run.jsonl --json
py -3 -m pytest motion_engine/tests/test_trace_checker.py
```

Exit code 0 = all clean, 1 = violations, 2 = unreadable file. Record your
own run by wrapping the stub in `TraceRecorder` (example in the module
docstring).

## MotionCommand contract

The lead's interface list calls this contract **MotionCommand**. In code it is
two types in [`contracts.py`](contracts.py): `Command` (the request) and
`CommandAck` (Motion's answer). There is no class named `MotionCommand` in
the contract; `motion_stubs.MotionCommand` is an unrelated internal history
record. Progress after acceptance is reported separately through
`DroneState` (alias `VehicleFeedback`, see [DroneState contract](#dronestate-contract)).

This is a draft. The shape follows [INTEGRATION_README](../INTEGRATION_README.md)
section 4; the names match the code today.

### Purpose

Give the coordinator one way to ask Motion to do something, and one
immediate, stable answer: accepted (and now flying it) or rejected (and why,
with the previous command still flying). `accepted` means "executing", not
"done".

### Publisher and subscriber

- **`Command` publisher:** the coordinator
  ([`state_machine/run_loop.py`](../state_machine/run_loop.py)), after
  Safety's command check (`safety_adapter.check_for_motion()`). Commands come
  from `launch_drone()` (`arm-N`, `takeoff-N`), `send_command()` (`sm-N`, from
  `wanted_command()` including Docking's proposed target), or a Safety
  replacement of one of those (`<id>-safety`, `source = "safety"`, no target).
  The waypoint executor builds `MOVE_TO` commands the same way but is not
  wired into the coordinator yet.
- **`Command` subscriber:** `MotionStub.submit(command, now_ns)`.
- **`CommandAck` publisher:** `MotionStub.submit()`, as its return value, on
  the same tick.
- **`CommandAck` subscribers:** the coordinator (`run_world()`'s `commands`
  list, and `MissionStatus.pending_command_failure` when rejected) and
  `WaypointExecutor.on_ack()`.

### `Command` fields

| Field | Type | Units / values | Meaning |
|---|---|---|---|
| `schema_version` | int | always `1` | Contract version |
| `command_id` | str | non-empty | Identifies this request; feedback and acks refer to it. Reusing it means "renew" (see below) |
| `kind` | str (`MotionCommandType`) | one of `ARM`, `DISARM`, `TAKEOFF`, `MOVE_TO`, `HOVER`, `SET_YAW`, `TRACK_TARGET`, `SET_VELOCITY`, `GOTO_DOCK_APPROACH`, `DESCEND_TO_DOCK`, `LAND`, `EMERGENCY_LAND`, `STOP` | What to do. `SET_ORIENTATION` exists in the enum but is not a contract kind |
| `mission_state` | str | mission state value, e.g. `"DOCKING_APPROACH"` | The coordinator's state when it sent the command. Motion only checks it is a string |
| `target` | `Pose` or null | `x`, `y`, `z` in metres, local ENU (x east, y north, z up, pad and ground at z = 0); `yaw_deg` in degrees, 0 = east, counter-clockwise positive | Required for `TAKEOFF` (only `z` used: climb in place), `MOVE_TO`, `TRACK_TARGET`, `SET_YAW` (only `yaw_deg` used), `GOTO_DOCK_APPROACH` (the approach point itself) and `DESCEND_TO_DOCK` (the pad pose). Must be null for `SET_VELOCITY`; ignored by the other kinds |
| `velocity` | `Velocity` or null | `vx`, `vy`, `vz` in m/s, local ENU (east, north, up); `yaw_rate_deg_s` in deg/s, counter-clockwise positive, or null to keep the current heading | Required for `SET_VELOCITY` (see [Velocity command](#velocity-command-set_velocity)); must be null for every other kind |
| `frame` | str | always `"local_enu"` | Frame of `target` |
| `max_horizontal_speed_mps` | float or null | m/s, > 0 | Optional cap for this command. Can only lower the configured limit; a larger value has no effect. For `SET_VELOCITY` a setpoint faster than the cap is rejected, not scaled down |
| `max_vertical_speed_mps` | float or null | m/s, > 0 | Same, vertical |
| `issued_ns` | int | ns, simulation clock, >= 0 | When the coordinator issued it |
| `expires_ns` | int or null | ns, simulation clock, > `issued_ns` | When it stops being valid. Null only for `ARM`, `DISARM`, `TAKEOFF`, `LAND`, `EMERGENCY_LAND` |
| `source` | str | e.g. `"state_machine"`, `"safety"` | Who built it. Motion only checks it is a string |
| `reason` | str | free text | Why it was sent (`step()`'s reason, `"renew"`, or Safety's reason code). Motion only checks it is a string |

### `CommandAck` fields

| Field | Type | Units / values | Meaning |
|---|---|---|---|
| `schema_version` | int | always `1` | Contract version |
| `command_id` | str | | The ID of the command being answered |
| `kind` | str or null | `MotionCommandType` value | The command's kind |
| `accepted` | bool | | True = Motion is now executing it (or renewed it) |
| `reason` | str | a `contracts.Reason` value (table below) | Stable code; compare against the constants, never free text |
| `detail` | str or null | free text | Extra human-readable detail on a few rejections, otherwise null |
| `active_command_id` | str or null | | What Motion is executing *after* handling this command. On a rejection this is the previous command, still flying. Null when nothing is active, or when the active target came from an expiry hold or the emergency auto-disarm |
| `timestamp_ns` | int | ns, simulation clock | The `now_ns` passed to `submit()` |

### Update rate

- The coordinator runs at 10 Hz. Each 0.1 s tick it submits at most one
  command, before `motion.step(0.1)`. The launch tick is the exception: it
  sends `ARM` and `TAKEOFF` together and nothing else.
- Renewal: while the mission wants the same kind and target, the coordinator
  re-sends the **same** `command_id` every tick with `issued_ns = now` and
  `expires_ns = now + 0.2 s`, so one missed tick is tolerated. Motion answers
  `renewed`: no restart and no new log line.
- A different kind or target gets a new ID (`sm-N+1`). `ARM`, `TAKEOFF`,
  `DISARM` and `EMERGENCY_LAND` are sent once with no expiry.
- If renewals stop, the command expires at `expires_ns` and Motion holds in
  place before moving that tick (`status = expired_hold`), unless it is a
  landing already settled on its target.
- The ack is synchronous: it is `submit()`'s return value on the same tick.

### Validation rules

`submit()` checks a command in this order and returns the first rejection.
Rows 3 to 12 are `Command.validate()`, which checks the command on its own;
the rest depend on Motion's current state. Every rejection is an ack with
`accepted = false`, never an exception, and leaves the previous command
active.

| # | Check | Ack `reason` | `detail` |
|---|---|---|---|
| 1 | Not a `contracts.Command` | `invalid_command` | `not a contracts.Command` |
| 2 | `now_ns` is not an int | `invalid_command` | `now_ns must be an int` |
| 3 | `schema_version` is not 1 | `unsupported_schema` | |
| 4 | `command_id` is not a non-empty string; `mission_state`, `source` or `reason` is not a string | `invalid_command` | |
| 5 | `kind` is not a `MotionCommandType` value, or is `SET_ORIENTATION` | `unsupported_kind` | |
| 6 | `frame` is not `"local_enu"` | `unsupported_frame` | |
| 7 | `issued_ns` is not an int (bools rejected) or is negative | `invalid_command` | |
| 8 | `expires_ns` is null for a kind other than `ARM`, `DISARM`, `TAKEOFF`, `LAND`, `EMERGENCY_LAND` | `missing_expiry` | |
| 9 | `expires_ns` is set but not an int, or not later than `issued_ns` | `invalid_command` | |
| 10 | A target kind has no `target` / the target is not a `Pose` or has a NaN, inf or non-numeric value | `missing_target` / `invalid_command` | |
| 11 | `SET_VELOCITY` has no `velocity` / the velocity is not a `Velocity` or has a NaN, inf or non-numeric value, or the command also has a `target`; or a non-`SET_VELOCITY` kind carries a `velocity` | `missing_velocity` / `invalid_command` | |
| 12 | A speed cap is set but is not a finite number > 0 | `invalid_command` | |
| 13 | `issued_ns` is later than `now_ns` | `future_timestamp` | |
| 14 | `now_ns >= expires_ns` (never for `EMERGENCY_LAND`) | `command_expired` | |
| 15 | Same ID as the active command but a different kind, target, velocity or speed cap | `command_id_reused` | `same id, different request` |
| 16 | Same ID as the active command, same request: **accepted** | `renewed` | |
| 17 | ID was accepted before (including an active command that has since expired, been replaced or stopped at a velocity boundary) | `command_id_reused` | |
| 18 | Emergency landing latched: an `EMERGENCY_LAND` under a new ID is **accepted**; anything else is rejected | `already_emergency_landing` / `emergency_descent_active` | |
| 19 | Disarmed, and the kind is not `ARM` or `DISARM` | `not_armed` | |
| 20 | `DISARM` while airborne (z > 0.05 m) | `airborne` | |
| 21 | `SET_VELOCITY` while not airborne (prevents sliding along the pad) | `not_airborne` | `take off before flying a velocity` |
| 22 | `SET_VELOCITY` faster than the shared limits or the command's own caps: `hypot(vx, vy)` > horizontal, `abs(vz)` > vertical, `abs(yaw_rate_deg_s)` > yaw rate. Never scaled down | `velocity_outside_limits` | which axis, e.g. `horizontal 6.000 m/s > 5.0 m/s` |
| 23 | Target outside the shared limits: above the ceiling, below the ground, or outside the geofence radius around home (`TAKEOFF` checks its altitude at the current x/y). Never clamped | `target_outside_limits` | |
| 24 | Otherwise: **accepted** | `accepted` | |

Notes, all checked against the code:

- Every ack `reason` appears above: `accepted`, `renewed`,
  `already_emergency_landing`, `invalid_command`, `unsupported_schema`,
  `unsupported_frame`, `unsupported_kind`, `missing_target`,
  `missing_velocity`, `missing_expiry`, `future_timestamp`,
  `command_id_reused`, `command_expired`, `not_armed`, `airborne`,
  `emergency_descent_active`, `not_airborne`, `velocity_outside_limits`,
  `target_outside_limits`.
- The other `contracts.Reason` values are **never** an ack `reason`:
  - Boundary-hold reasons, reported in `DroneState.status_reason` with
    `status = boundary_hold` (and as the log `decision`) when a
    `SET_VELOCITY` reaches a limit after it was accepted:
    `geofence_boundary`, `altitude_ceiling_boundary`,
    `altitude_floor_boundary`. (`command_expired` is also the
    `status_reason` of an `expired_hold`.)
  - `DroneState.validate()` results, describing a malformed vehicle state
    rather than a command: `invalid_state`, `invalid_timestamp`,
    `missing_field`, `inconsistent_state` (see
    [DroneState contract](#dronestate-contract)).
  - `comm_loss_landing`: Motion started a landing on its own after a
    sustained expired hold. Reported in `DroneState.status_reason` (and
    `reason`) and as a log event, never in an ack.
- A renewal only needs the same ID and request and an unexpired
  `expires_ns`; Motion does not require the new expiry to be later than the
  old one.
- Only accepted IDs are remembered. A rejected ID can be sent again, and is
  judged afresh.
- `kind` may be the enum or its string (`"HOVER"`), so a command built from
  JSON validates the same way. `velocity` must be a `Velocity`, though: a
  plain dict is rejected as `invalid_command`.
- Motion logs each accepted and each rejected command once (event = the kind
  in lowercase, `fields.command_id`, `decision = rejected_<reason>` on a
  rejection). Renewals and repeated `EMERGENCY_LAND`s are not logged.

### Example payloads

[`tests/test_motion_command_contract_doc.py`](tests/test_motion_command_contract_doc.py)
reads the four JSON blocks below from this file, builds each `Command` and
`CommandAck`, replays them through a `MotionStub` and checks the acks match
exactly, so the examples cannot drift from the code.

An accepted command: the pad descent sent in the `full_cycle` demo at
6300 ms. `reason` is `"renew"` because `step()` gave no reason that tick (the
mission was already in `DOCKING_APPROACH`; Docking changed its proposal).

<!-- motion-command-example: accepted_command -->
```json
{
  "schema_version": 1,
  "command_id": "sm-3",
  "kind": "DESCEND_TO_DOCK",
  "mission_state": "DOCKING_APPROACH",
  "target": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
  "velocity": null,
  "frame": "local_enu",
  "max_horizontal_speed_mps": null,
  "max_vertical_speed_mps": null,
  "issued_ns": 6300000000,
  "expires_ns": 6500000000,
  "source": "state_machine",
  "reason": "renew"
}
```

Its ack:

<!-- motion-command-example: accepted_ack -->
```json
{
  "schema_version": 1,
  "command_id": "sm-3",
  "kind": "DESCEND_TO_DOCK",
  "accepted": true,
  "reason": "accepted",
  "detail": null,
  "active_command_id": "sm-3",
  "timestamp_ns": 6300000000
}
```

A rejected command (made up for this example; the coordinator does not send
`MOVE_TO` yet): a `MOVE_TO` above the 30 m ceiling at 6400 ms, while `sm-3`
is flying.

<!-- motion-command-example: rejected_command -->
```json
{
  "schema_version": 1,
  "command_id": "example-1",
  "kind": "MOVE_TO",
  "mission_state": "DOCKING_APPROACH",
  "target": {"x": 0.0, "y": 0.0, "z": 40.0, "yaw_deg": 0.0},
  "velocity": null,
  "frame": "local_enu",
  "max_horizontal_speed_mps": null,
  "max_vertical_speed_mps": null,
  "issued_ns": 6400000000,
  "expires_ns": 6600000000,
  "source": "example",
  "reason": "example"
}
```

Its ack. `active_command_id` is still `sm-3`: the descent keeps flying.

<!-- motion-command-example: rejected_ack -->
```json
{
  "schema_version": 1,
  "command_id": "example-1",
  "kind": "MOVE_TO",
  "accepted": false,
  "reason": "target_outside_limits",
  "detail": null,
  "active_command_id": "sm-3",
  "timestamp_ns": 6400000000
}
```

The next tick's renewal of `sm-3` (`issued_ns` 6400000000, `expires_ns`
6600000000, otherwise identical) is acknowledged with `accepted: true`,
`reason: "renewed"`.

### Open questions

- Name: should the code type be renamed `MotionCommand` to match the
  interface list? That would clash with the internal history record in
  `motion_stubs.py`, which would need renaming first.
- `reason` on a command is free text, and the coordinator sends `"renew"`
  on the first send of a new command when `step()` gave no reason. Should it
  be a stable code, as INTEGRATION_README section 4 asks for elsewhere?
- `active_command_id` is null during an expiry hold, although Motion is
  holding. Should a hold report the expired command's ID, or its own?

## Phase-1 (PX4) goals

The original PX4 plan. It comes after the synthetic milestone above.

### Purpose

Translate permitted high-level movement targets into PX4 external-control commands. PX4 supplies low-level flight stabilization in simulation. Building a replacement flight PID or directly driving motors is not required for this semester.

### Inputs and outputs

- Inputs: navigation/docking targets, estimated vehicle state, safety decisions.
- Outputs: bounded PX4 position or velocity targets and yaw commands; command/status logs.
- Define units, coordinate frames, command expiration, and external-control lifecycle before connecting modules.
- Log requested and sent targets separately from observed vehicle motion.

### First tasks

1. Establish a PX4 connection and read status/telemetry.
2. Reproduce takeoff, hold, and landing in the stock simulation.
3. Define a bounded command interface with navigation and safety.
4. Handle rejected commands, stale requests, and loss of the control connection.
5. Execute waypoint and docking targets through the same interface.

Maintain the command stream required by the selected PX4 interface and verify its loss-of-control behavior. Start from the [PX4 offboard example](https://docs.px4.io/main/en/ros2/offboard_control), using documentation matching the pinned release.

### Acceptance evidence

A repeatable flight script takes off, holds, and lands while recording targets and telemetry. Tests demonstrate bounds enforcement, command expiration, and the chosen abort behavior.

A controlled mission abort and motor termination are distinct actions. Do not implement one ambiguous “kill switch” for both.

The PX4 side (offboard connection, SITL flight script) is not part of the
synthetic milestone. Each `MotionStub` method has a `# PHASE-1 HOOK:` comment
naming the PX4 command it becomes; `grep -n "PHASE-1 HOOK" motion_engine/motion_stubs.py`
is the to-do list.
