# State Machine — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Decide the drone's current mission state each tick of the synthetic simulation loop. The state machine reads the shared observation produced by CV, Safety, and Docking, selects the next state, and records why. Motion acts on the chosen state; the state machine never moves the vehicle or detects anything itself.

Phase 0 is purely synthetic: all inputs are generated values, not sensor or simulator data. The state machine is written so that Phase 1 can supply real observations without changing its logic.

## States

| State | Where | Meaning | Motion action (proposed) |
|---|---|---|---|
| `IDLE` | On the dock | On the dock or grounded, waiting for a reason to launch | None (disarmed) |
| `TRACKING` | Air | Airborne and following a detected bird | `track_target()` every tick |
| `HOVERING` | Launching from the dock, or air | Launching (arming and climbing), or airborne and holding position (bird lost, temporarily unsafe, or waiting to dock) | `arm()` + `takeoff()` when coming from `IDLE`, otherwise `hover()` |
| `DOCKING_INIT` | Air | Docking requested; checking that the dock can be approached | `hover()` while waiting for approach permission |
| `DOCKING_APPROACH` | Air → landing on the dock | Moving toward and descending onto the dock | `goto_dock_approach()`, then `descend_to_dock()` |
| `DOCKED` | On the dock | Landed on the dock | `disarm()` |
| `ABORT` | Air | Current action cancelled; deciding whether to recover or land | `stop()` (hold in place) |
| `EMERGENCY_LAND` | Air → landing where it is | Synthetic only. Land immediately at the current position | `emergency_land()` |

These names are part of the Phase 0 → Phase 1 interface contract and must not change without team agreement.

- `IDLE` and `DOCKED` are both on the dock. `DOCKED` means just landed with the battery not yet recovered; it becomes `IDLE` once the battery has recovered.
- There is no separate takeoff state: `HOVERING` includes the launch ([integration contract](../INTEGRATION_README.md) section 2). A launch goes `IDLE → HOVERING → TRACKING`. Holding `TRACKING` back until the climb finishes needs a launch-progress input that the team has not agreed yet.
- In Phase 0, "the dock" is a stationary pad in simulation. The golf-cart dock is the long-term product (see [Start Here](../START_HERE.md)).
- Motion actions are proposed and need agreement with the Motion team. Command names are from [`motion_engine/motion_stubs.py`](../motion_engine/motion_stubs.py). The state machine only chooses the state; the simulation loop issues the command.

## Inputs and outputs

- Input: the current state and one observation per tick (see the observation format below).
- Output: the next state and a transition reason, written to the observation's `state` field and to the log.
- The state machine reads Docking's `should_dock`, `reason`, `phase` and `landed_on_pad`; it does not re-evaluate docking triggers itself. Likewise, it reads the Safety layer's decisions (`permissions`, `emergency`, `abort` and `battery_recovered`) rather than re-deriving safety rules, so it never checks whether the dock is clear itself.

Proposed interface:

```python
next_state, reason = step(current_state, observation)
```

`step()` has no side effects, which keeps it deterministic and easy to unit test.

### Observation format (draft, to be agreed with all modules)

```json
{
  "timestamp": "...",
  "bird":    { "type": "hawk", "distance_m": 32.5, "detected": true },
  "battery": { "percent": 42, "low": false },
  "safety":  { "permissions": { "launch": true, "tracking": true, "approach": true, "descent": true, "hold": true },
               "emergency": false, "abort": false,
               "battery_low": false, "battery_critical": false, "battery_recovered": true, "reasons": [] },
  "docking": { "should_dock": false, "reason": null, "phase": "idle", "landed_on_pad": false },  "state":   "TRACKING"
}
```

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
| `DOCKED` | Safety reports `battery_recovered` | `IDLE` |
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

The `safety` field names come from Safety's `SafetyAssessment` (PR #61). The Docking field names `phase` and `landed_on_pad` are proposed; confirm them with the Docking team.

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

## Running

Code lives in this folder:

- `state_machine.py`: the `State` enum and `step()`.
- `run_loop.py`: the 10 Hz synthetic loop, which runs a scripted demo scenario through `step()`.
- `tests/`: unit tests for `step()` and the demo run.

Run from the repo root:

```bash
python3 -m state_machine.run_loop
```

Prints every state transition in the demo scenario, with its time and reason.

```bash
python3 -m pytest state_machine -v
```

Runs the state machine tests.

The loop uses a simulation clock, so 10 simulated seconds finish almost instantly. Call `run_world(10000, real_time=True)` to pace it in real time instead.
