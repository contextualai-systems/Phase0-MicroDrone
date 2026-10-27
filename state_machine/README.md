# State Machine — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Decide the drone's current mission state each tick of the synthetic simulation loop. The state machine reads the shared observation produced by CV, Safety, and Docking, selects the next state, and records why. Motion acts on the chosen state; the state machine never moves the vehicle or detects anything itself.

Phase 0 is purely synthetic: all inputs are generated values, not sensor or simulator data. The state machine is written so that Phase 1 can supply real observations without changing its logic.

## States

| State | Where | Meaning | Motion command sent by `run_loop.py` |
|---|---|---|---|
| `IDLE` | On the dock | On the dock or grounded, waiting for a reason to launch | None |
| `TRACKING` | Air | Airborne and following a detected bird | None yet (needs a tracking target; Navigation has no code) |
| `HOVERING` | Launching from the dock, or air | Launching (arming and climbing), or airborne and holding position (bird lost, temporarily unsafe, or waiting to dock) | `ARM` + `TAKEOFF` once when coming from `IDLE`; `HOVER` once the climb has finished |
| `DOCKING_INIT` | Air | Docking requested; checking that the dock can be approached | `HOVER` while waiting for approach permission |
| `DOCKING_APPROACH` | Air → landing on the dock | Moving toward and descending onto the dock | Docking's proposed command: `GOTO_DOCK_APPROACH` (approaching and aligning), `HOVER` (searching for the pad), `DESCEND_TO_DOCK`. `HOVER` when Docking proposes nothing |
| `DOCKED` | On the dock | Landed on the dock | `DISARM`, once the drone is on the ground |
| `ABORT` | Air | Current action cancelled; deciding whether to recover or land | `STOP` (hold in place) |
| `EMERGENCY_LAND` | Air → landing where it is | Synthetic only. Land immediately at the current position | `EMERGENCY_LAND`, once |

These names are part of the Phase 0 → Phase 1 interface contract and must not change without team agreement.

- `IDLE` and `DOCKED` are both on the dock. `DOCKED` means just landed with the battery not yet recovered; it becomes `IDLE` once the battery has recovered and Motion's feedback shows the drone disarmed and grounded at the pad.
- There is no separate takeoff state: `HOVERING` includes the launch ([integration contract](../INTEGRATION_README.md) section 2). A launch goes `IDLE → HOVERING → TRACKING`. `MissionStatus` now reports the launch progress (`launch_phase`, see "MissionStatus"), but `step()` does not use it yet, so `TRACKING` can still begin during the climb.
- In Phase 0, "the dock" is a stationary pad in simulation. The golf-cart dock is the long-term product (see [Start Here](../START_HERE.md)).
- The Motion command for each state is chosen by `wanted_command()` in `run_loop.py`. The mapping is a proposal and needs agreement with the Motion team. Command names are Motion's `MotionCommandType` ([`motion_engine/contracts.py`](../motion_engine/contracts.py)). The state machine only chooses the state; the loop issues the command.

## Inputs and outputs

- Input: the current state and one observation per tick (see the observation format below).
- Output: the next state and a transition reason. `run_loop.py` records them every tick in a `MissionStatus`, with the rest of the mission's status (see "MissionStatus"), and logs each transition.
- The state machine reads Docking's `should_dock`, `phase` and `landed_on_pad`; it does not re-evaluate docking triggers itself. Likewise, it reads the Safety layer's decisions (`permissions`, `emergency`, `abort` and `battery_recovered`) rather than re-deriving safety rules, so it never checks whether the dock is clear itself.
- Those decisions are worked out every tick by Safety's `assess()` and Docking's controller, which `run_loop.py` calls before `step()` (see "Safety integration" and "Docking integration" below).

Proposed interface:

```python
next_state, reason = step(current_state, observation)
```

`step()` has no side effects, which keeps it deterministic and easy to unit test.

### Observation format (draft, to be agreed with all modules)

What `step()` receives on one tick of the `full_cycle` demo (5000 ms: low battery, cart moving):

```json
{
  "timestamp_ns": 5000000000,
  "bird":        { "detected": false, "type": null, "distance_m": null, "valid": true, "timestamp_ns": 5000000000 },
  "battery":     { "percent": 18, "valid": true, "timestamp_ns": 5000000000 },
  "environment": { "people_nearby": false, "cart_moving": true, "weather": "clear",
                   "camera_ok": true, "comm_ok": true, "valid": true, "timestamp_ns": 5000000000 },
  "dock":        { "pad_pose": { "x": 0, "y": 0, "z": 0, "yaw_deg": 0 }, "alignment_valid": true,
                   "valid": true, "timestamp_ns": 5000000000 },
  "vehicle":     { "armed": true, "airborne": true, "grounded": false, "pose": { "x": 0.0, "y": 0.0, "z": 5.0, "yaw_deg": 0.0 },
                   "valid": true, "timestamp_ns": 4900000000, "...": "the rest of Motion's VehicleFeedback" },
  "context":     { "flight_active": true, "return_started_ns": null },
  "safety":      { "permissions": { "launch": false, "tracking": false, "approach": false, "descent": false, "hold": true },
                   "emergency": false, "abort": false, "battery_low": true, "battery_critical": false,
                   "battery_recovered": false, "reasons": ["low_battery", "cart_moving"], "timestamp_ns": 5000000000 },
  "docking":     { "should_dock": true, "reasons": ["low_battery", "cart_moving"], "phase": "idle",
                   "reason": "docking_inactive", "landed_on_pad": false, "proposed_target": null,
                   "...": "the rest of Docking's DockingUpdate" }
}
```

| Section | Comes from |
|---|---|
| `bird`, `battery`, `environment`, `dock` | The demo scenario (`demos.py`), stamped with the current time. `bird` will come from CV once it is connected. `dock.pad_pose` is the fixed pad (`PAD` in `run_loop.py`). |
| `vehicle` | Motion's feedback from the previous tick (`VehicleFeedback.to_dict()`) |
| `context` | `run_loop.py`: `flight_active` is true while the drone is armed or airborne; `return_started_ns` is the return latch (see "Docking integration") |
| `safety` | Safety's `assess()`, worked out from the sections above |
| `docking` | Docking's controller (`DockingUpdate.to_dict()`). Its `reasons` are why it wants to return; its `reason` is why its phase changed |

`step()` reads `bird`, `safety` and `docking`, plus `vehicle`, `dock.pad_pose` and `timestamp_ns` for `DOCKED → IDLE`. The other sections are what Safety and Docking need to decide.

## Docking triggers

Docking may be requested for any of the following. Docking computes `should_dock`; the state machine decides what that means in the current state.

1. Low battery
2. No bird detected
3. People approaching
4. Cart moving
5. Weather hazard
6. Camera failure
7. Communication loss

## Transitions (draft)

Rules are checked in priority order and the first match wins:

1. **Global overrides.** In any airborne state, Safety's `emergency` goes to `EMERGENCY_LAND`, then Safety's `abort` goes to `ABORT` (except from `ABORT` itself).
2. **Per-state rules.** From the table below.
3. **Default.** No match means the state is unchanged.

| From | Condition | To |
|---|---|---|
| `IDLE` | `should_dock` | Ignored; stays `IDLE` (a grounded drone ignores return requests) |
| `IDLE` | Bird detected and `launch` permitted | `HOVERING` (start the launch) |
| `TRACKING` | `should_dock` | `DOCKING_INIT` |
| `TRACKING` | `tracking` not permitted | `HOVERING` |
| `TRACKING` | Bird lost | `HOVERING` |
| `HOVERING` | `should_dock` or hover timeout | `DOCKING_INIT` |
| `HOVERING` | Bird detected and `tracking` permitted | `TRACKING` |
| `DOCKING_INIT` | Safety permits the approach (`approach`) | `DOCKING_APPROACH` |
| `DOCKING_INIT` | Approach not permitted | Stay and hold |
| `DOCKING_APPROACH` | Docking phase `abort`, approach permission lost, or descent permission lost while descending | `ABORT` |
| `DOCKING_APPROACH` | Docking confirms `landed_on_pad` | `DOCKED` |
| `DOCKED` | Safety reports `battery_recovered`, and Motion's fresh feedback shows the drone disarmed and grounded within 0.25 m of the pad | `IDLE` |
| `ABORT` | Safety permits holding (`hold`) | `DOCKING_INIT` (retry docking) |
| `ABORT` | Holding not permitted | Stay in `ABORT` |
| `ABORT` | Not recoverable | `EMERGENCY_LAND` |
| Any airborne state | Safety `emergency` (e.g. critical battery) | `EMERGENCY_LAND` |
| Any airborne state except `ABORT` | Safety `abort` (e.g. operator abort) | `ABORT` |

The cart-motion rule in the [main README](../README.md) applies: the drone never lands on a moving cart. While the cart is moving, docking waits in a safe state.

Docking rules follow the [integration contract](../INTEGRATION_README.md) (sections 6–8):

- Safety decides whether the approach and descent are permitted. People nearby or a moving cart block both; the drone holds in `DOCKING_INIT` until the approach is permitted.
- Docking's local `phase` (`idle`, `approach`, `search`, `align`, `descend`, `complete`, `abort`; see [`docking/states/docking_state.py`](../docking/states/docking_state.py)) only runs while the mission is in `DOCKING_APPROACH`.
- Docking reports `landed_on_pad` once touchdown is confirmed, and phase `abort` when the attempt fails.

The `safety` field names come from Safety's `SafetyAssessment` (PR #61). The `docking` field names come from Docking's `DockingUpdate` (PR #68).

### Open questions

- Which conditions lead to docking, which to abort, and which to emergency landing (for example, low vs. critical battery, or communication loss during approach)?
- Does losing the bird start docking immediately or after a hover timeout?
- What is the priority order when several triggers are active in the same tick?
- How does the system leave `EMERGENCY_LAND`: terminal, or manual reset to `IDLE`?
- Which module owns the overall "safe" decision? Proposed: Safety provides it and the state machine consumes it.

## Logging

Use the shared Phase 0 logging format:

```
[timestamp] [module] [event] [state] [synthetic_truth] [decision]
```

The state machine logs the current state every tick and logs every transition with the previous state, the new state, and the reason:

```
[12:03:22.120] STATE: TRACKING
[12:03:25.480] STATE: transition TRACKING -> DOCKING_INIT (reason=low_battery)
```

## First tasks

1. Agree on the states, transition table, and open questions with Docking, Safety, and Motion.
2. Agree on the shared observation format and thresholds (low/critical battery, landing tolerance, hover timeout).
3. Implement the `State` enum and `step()`.
4. Emit log lines using the shared logger.
5. Write unit tests: one per transition, one per docking trigger, and cases with several triggers at once.
6. Integrate into the simulation loop between Docking and Motion.
7. Verify that changing values during a running simulation produces the expected transitions.
8. Document the state machine's update rate for the Phase 1 interface contract.

## Acceptance evidence

Every transition in the agreed table is covered by a passing test. Each docking trigger, injected during a running synthetic simulation, produces the expected state change with a logged reason. Invalid or missing observation fields produce a defined state rather than an unhandled error.

## Phase 1 notes

Phase 1 must implement the same states and respect them in motion control. Real observations replace synthetic ones through the same observation format; the transition logic should not need to change.

Simulation loop:
Pretend 3D coordinates
CV: Feed birds into camera (Bird? Safetycheck & Dockcheck : Do nothing) Predefined states for images (Kind of bird, distance away) - Elias C
Dock (battery power, people show up, cart is ready to move, no birds, weather conditions, camera failure, lose communication)? Initiate Docking : Do nothing) - Elias N
Safety layer (Safetycheck? Move : Don’t Move/Possible dock check) - Jermaine
Motion ( Move to new X,Y, Z; Leave comments in code for any place a real movement or orientation command might need to be) - Connor
State Machine - Mykolas
Universal logging - Will
Part of Phase 0 Deliverable is what is needed for Phase 1.


Be able to, in a running simulation, test each of these values changing.


## Safety integration

`run_loop.py` asks Safety what is allowed every tick, before `step()`:

1. It builds the snapshot: the scenario's `bird`, `battery`, `environment` and `dock` facts, each stamped with the current time; Motion's feedback as `vehicle`; and `context` with `flight_active` and `return_started_ns`.
2. It calls Safety's `assess(snapshot, now_ns)` ([`safety_layer/safety_policy.py`](../safety_layer/safety_policy.py)) and puts the answer in `snapshot["safety"]`, where `step()` reads it.

Safety fails closed: a missing section, or one older than 0.5 s, counts as not permitted. Its emergency rules (critical battery, lost comms, bad weather, stale data) only apply while `flight_active` is true.

`context.return_started_ns` is for Safety's 60 s return deadline ([integration contract](../INTEGRATION_README.md) sections 5 and 6). Safety does not read it yet: the deadline is still a TODO in `safety_policy.py`, and the field name is a proposal to agree with the Safety team.

Every command then passes through `check_for_motion()` ([`motion_engine/safety_adapter.py`](../motion_engine/safety_adapter.py)) before it reaches Motion. If Safety does not permit it, Safety sends a replacement instead: `HOVER`, `LAND` if holding is not permitted either, or `EMERGENCY_LAND` during an emergency. A replacement's command id ends in `-safety`.

## Docking integration

`run_loop.py` runs Docking's controller ([`docking/states/docking_controller.py`](../docking/states/docking_controller.py)) every tick, after `assess()` and before `step()`:

1. It updates two timers that Docking needs but does not own:
   - `bird_absent`: when the bird went missing. Set on the first tick with a valid sample and no bird, cleared when the bird is seen.
   - `hover_started`: when ordinary hovering began. Set on the tick the mission is in `HOVERING` with the launch climb finished, cleared on the tick it leaves `HOVERING`. It is updated after `step()`, so Docking reads the value from the end of the previous tick.
2. It builds a `DockingInput` from the snapshot (mission state, Motion's feedback, the pad, Safety's assessment, alignment, cart, bird and the two timers). The alignment errors are the pad's pose minus the drone's, because both are known exactly in the synthetic world.
3. It calls `update(inputs, now_ns)` and puts `to_dict()` in `snapshot["docking"]`, where `step()` reads `should_dock`, `phase` and `landed_on_pad`.

Docking asks to return 3 s after the bird is lost, after 10 s of ordinary hovering, or straight away for a Safety restriction such as low battery or a moving cart. While the mission is in `DOCKING_APPROACH`, it flies the approach, searches for and aligns with the pad, descends, and confirms the landing from Motion's feedback. `run_loop.py` sends Docking's proposed command in that state, after Safety's check. Docking proposes nothing on the tick the mission enters `DOCKING_APPROACH` (it was told the state before `step()`), after landing, and after a failure; the loop holds with `HOVER` then.

Leaving `DOCKING_APPROACH` resets Docking, so `ABORT → DOCKING_INIT → DOCKING_APPROACH` starts a fresh attempt.

The return latch: `return_started` is set on the first `DOCKING_INIT` and keeps that time through aborts and retries, so a retry cannot restart the 60 s return deadline. It clears when the mission returns to `IDLE`. The loop passes it to Safety as `context.return_started_ns`.

## Motion integration

`run_loop.py` drives Motion's simulated drone (`MotionStub`) from the mission state. `step()` still only decides the state; it never sends a command.

Each tick, after `step()`:

1. On `IDLE → HOVERING`, `launch_drone()` sends `ARM` and `TAKEOFF` once.
2. `wanted_command(state, vehicle, proposal)` picks the command for the current state (the last column of the States table), using Motion's feedback from the previous tick and, in `DOCKING_APPROACH`, Docking's proposed command.
3. `send_command()` sends it, after Safety's command check:
   - `HOVER`, `STOP`, `GOTO_DOCK_APPROACH` and `DESCEND_TO_DOCK` are valid for 0.2 s and are renewed every tick with the same command id. A different command gets a new id.
   - `DISARM` and `EMERGENCY_LAND` are sent once.
4. `motion.step()` moves the drone by one tick, and `motion.feedback()` reads its new status.

Nothing is sent while the takeoff is still climbing, because a new command would replace the takeoff.

Not connected yet:

- `TRACKING` sends no command, because there is no Navigation target yet.
- A command that Motion rejects is printed, not handled. `context` does not yet report it to Safety (`pending_command_failure`), and has no `operator_abort`.
- The bird is still scripted by the demo scenarios; CV is not connected yet.
- The 60 s return deadline does not fire yet (see "Safety integration"), so a drone that can never line up with the pad keeps retrying until the run ends.

## MissionStatus

`MissionStatus` ([`contracts.py`](contracts.py)) is the state machine's output: one record per tick saying what the mission is doing and why. It is the mission status: "current state, transition reason, completion or failure" and the coordinator's `context` in the [integration contract](../INTEGRATION_README.md) section 4. Motion's `VehicleFeedback` describes the drone; `MissionStatus` describes the mission.

This is a draft. Field names marked "Proposal" need team agreement.

### Purpose

Give every module, the log and the tests the mission's status in one defined format: the state and why it changed, plus the mission's own context (launch progress, timers, the return latch, command failures and how the run ended).

### Fields

| Field | Type | Meaning | Name from |
|---|---|---|---|
| `timestamp_ns` | int | Simulation time of this tick | Contract section 4 |
| `tick` | int | Tick number, starting at 0 | Proposal |
| `previous_state` | `State` | Mission state at the start of the tick | Proposal |
| `state` | `State` | Mission state after `step()` | Proposal |
| `reason` | str or null | Why the state changed (`step()`'s reason); null when it did not change | Proposal |
| `state_entered_ns` | int | When the current state was entered | Proposal |
| `launch_phase` | str | `none`, `climbing` or `complete` (see below) | Values from contract section 2; field name is a proposal |
| `flight_active` | bool | Whether the drone is armed or airborne after this tick's move | The same flag Safety gets as `context.flight_active` on the next tick |
| `bird_absent_since_ns` | int or null | First tick of the current bird absence; null while the bird is seen | Docking's `DockingInput` |
| `hover_started_ns` | int or null | When ordinary hovering began (not the launch climb); null outside it | Docking's `DockingInput` |
| `return_started_ns` | int or null | The return latch: when the first return began. Kept through aborts and retries, cleared in `IDLE` | Same as `context.return_started_ns` (a proposal) |
| `return_reason` | str or null | Why the return started: Docking's first return reason (for example `low_battery`), or `step()`'s reason if Docking gave none | Proposal |
| `pending_command_failure` | str or null | Motion's reason when it rejected this tick's command | Proposal |
| `outcome` | str or null | Last tick only: `completed`, `interrupted` or `failed` (see below); null on every other tick | Values from contract section 10; the rule is a proposal |
| `schema_version` | int | Always 1 | Contract section 4 |
| `clock` | str | Always `"simulation"` | Contract section 4 |
| `synthetic` | bool | Always true | Contract section 4 |

`launch_phase` is `climbing` from the launch until the takeoff reaches its 5 m altitude, then `complete` until the mission leaves `HOVERING`. Everything else is `none`, including an ordinary hover after the bird is lost.

The demos reach `complete` once `step()` waits for the climb before `TRACKING` ([integration contract](../INTEGRATION_README.md) section 7, not implemented yet). Until then, `test_launch_phase_reaches_complete` covers it with a launch where the bird leaves straight away, so the mission stays in `HOVERING` for the whole climb.

`outcome` (proposal): back on the pad (`IDLE` or `DOCKED`) is `completed`, `ABORT` or `EMERGENCY_LAND` is `failed`, and any other state means time ran out mid-mission, so `interrupted`. In the demos, `blocked_dock` is `interrupted`, `emergency` and `comm_loss` are `failed`, and the rest are `completed`.

### Units

- All times are integer nanoseconds on the simulation clock, starting at 0 (`clock = "simulation"`). Wall time is never used.
- `to_dict()` writes `state` and `previous_state` as uppercase strings (`"HOVERING"`, contract section 2). `launch_phase` and `outcome` are lowercase strings. Unset values are null.

### Update rate

10 Hz: one status per 0.1 s tick, so a 10 s demo produces 100 statuses (`tick` 0 to 99).

### Validation rules

Every status follows these rules in all seven demos; `test_each_tick_has_a_mission_status` checks them on every tick. The class itself does not check them.

- `timestamp_ns` equals `tick` × 100 ms.
- `previous_state` equals the previous tick's `state` (`IDLE` on tick 0).
- `reason` is set exactly when `state` differs from `previous_state`.
- `state_entered_ns` is no later than `timestamp_ns`, and equals it on a tick where the state changed.
- `launch_phase` is `none` outside `HOVERING`.
- `hover_started_ns` is null outside `HOVERING`.
- Every time field is null or between 0 and `timestamp_ns`.
- `return_started_ns` and `return_reason` are set together. Once set, `return_started_ns` does not change until `IDLE`, where both are null.
- `outcome` is null on every tick except the last.

### Example payload

Tick 50 of `full_cycle` (5000 ms): the battery is low and the cart is moving, so the mission commits to returning home.

```json
{
  "timestamp_ns": 5000000000,
  "tick": 50,
  "previous_state": "HOVERING",
  "state": "DOCKING_INIT",
  "reason": "Should dock",
  "state_entered_ns": 5000000000,
  "launch_phase": "none",
  "flight_active": true,
  "bird_absent_since_ns": 4000000000,
  "hover_started_ns": null,
  "return_started_ns": 5000000000,
  "return_reason": "low_battery",
  "pending_command_failure": null,
  "outcome": null,
  "schema_version": 1,
  "clock": "simulation",
  "synthetic": true
}
```

### Open questions

- `arming`: the contract lists it as a launch phase, but `launch_drone()` sends `ARM` and `TAKEOFF` on the same tick, so a launch goes straight to `climbing`. Keep `arming` for a Motion that arms first, or drop it?
- `outcome`: is the rule above right? For example, should an emergency landing that worked count as `failed`?
- `reason`: the contract asks for stable codes such as `bird_lost` (section 4), but `step()` returns sentences such as "Bird not detected". Switch to codes and keep the sentence as separate detail?
- `return_reason`: one reason, or all of Docking's reasons (for example `low_battery` and `cart_moving` together)?

## Running

Code lives in this folder:

- `state_machine.py`: the `State` enum and `step()`.
- `run_loop.py`: the 10 Hz synthetic loop. It runs a scripted demo through `step()`, sends the matching commands to Motion, and builds a `MissionStatus` every tick.
- `contracts.py`: `MissionStatus`, the mission's status for one tick (see "MissionStatus").
- `demos.py`: the scripted demo scenarios. They script facts about the world (bird, battery, cart, people, comms, pad alignment), not Safety's or Docking's answers.
- `tests/`: unit tests for `step()`, one end-to-end test per demo, and tests of the `MissionStatus` records (the validation rules on every demo, and the launch phase).

Run a demo from the repo root:

```bash
python3 -m state_machine.run_loop full_cycle
```

It prints every state transition with its time and reason. With no name, it runs `full_cycle`.

| Demo | Length | What happens |
|---|---|---|
| `full_cycle` | 10 s | Launch for a bird, lose it, return on low battery, wait for the cart, land, recharge |
| `docking_retry` | 15 s | The pad alignment is lost; Docking searches above the pad, gives up after 5 s and the mission aborts. The retry finds the alignment and lands |
| `emergency` | 9 s | The battery drops to 8% while tracking; Safety raises an emergency and the drone lands in place |
| `blocked_dock` | 5.5 s | The cart keeps moving, so Safety never allows the approach; the drone holds in `DOCKING_INIT` |
| `not_aligned` | 11 s | The drone reaches the pad before it can line up; Docking hovers above it until the alignment is valid, then lands |
| `cart_during_descent` | 12 s | The cart starts moving mid-descent; the mission aborts, waits for the cart to stop, then Docking climbs back and lands |
| `comm_loss` | 9 s | Communication is lost while tracking; Safety raises an emergency and the drone lands in place |

When the bird is lost and nothing else calls the drone home, it hovers for 3 s before Docking asks to return. A Safety restriction, such as low battery or the moving cart in `blocked_dock`, calls it home straight away.

The command paces the run in real time, so a 10-second demo takes 10 seconds. It also writes the transitions to a new log file in `universal_log/`; do not commit those files.

Run the tests:

```bash
python3 -m pytest state_machine -v
```

The tests call `run_world()` without real-time pacing or logging, so they finish immediately and write no files.

`run_world(duration_ms, scenario=...)` returns a dict:

- `transitions`: `(time_ms, from_state, to_state, reason)` for each state change.
- `commands`: `(time_ms, kind, accepted, reason)` for each new command sent to Motion. A renewed command is listed once.
- `vehicle`: Motion's feedback after the last tick (`grounded`, `armed`, `airborne`, `pose`, …).
- `statuses`: one `MissionStatus` per tick (see "MissionStatus").
