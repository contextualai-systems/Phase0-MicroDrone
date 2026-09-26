# State Machine — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Decide the drone's current mission state each tick of the synthetic simulation loop. The state machine reads the shared observation produced by CV, Safety, and Docking, selects the next state, and records why. Motion acts on the chosen state; the state machine never moves the vehicle or detects anything itself.

Phase 0 is purely synthetic: all inputs are generated values, not sensor or simulator data. The state machine is written so that Phase 1 can supply real observations without changing its logic.

## States

| State | Meaning |
|---|---|
| `IDLE` | On the dock or grounded, waiting for a reason to launch |
| `TRACKING` | Airborne and following a detected bird |
| `HOVERING` | Airborne and holding position (bird lost, temporarily unsafe, or waiting to dock) |
| `DOCKING_INIT` | Docking requested; checking that the dock can be approached |
| `DOCKING_APPROACH` | Moving toward and descending onto the dock |
| `DOCKED` | Landed on the dock |
| `ABORT` | Current action cancelled; deciding whether to recover or land |
| `EMERGENCY_LAND` | Synthetic only. Land immediately at the current position |

These names are part of the Phase 0 → Phase 1 interface contract and must not change without team agreement.

## Inputs and outputs

- Input: the current state and one observation per tick (see the observation format below).
- Output: the next state and a transition reason, written to the observation's `state` field and to the log.
- The state machine reads Docking's `should_dock` and `reason`; it does not re-evaluate docking triggers itself. Likewise, it reads the Safety layer's decision rather than re-deriving safety rules.

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
  "safety":  { "people_nearby": false, "cart_moving": false,
               "weather": "clear", "camera_ok": true, "comm_ok": true },
  "docking": { "should_dock": false, "reason": null },
  "state":   "TRACKING"
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

1. **Global overrides.** Conditions that apply in any airborne state, such as critical battery, go to `EMERGENCY_LAND`.
2. **Per-state rules.** From the table below.
3. **Default.** No match means the state is unchanged.

| From | Condition | To |
|---|---|---|
| `IDLE` | `should_dock` | `DOCKING_INIT` |
| `IDLE` | Bird detected and safe | `TRACKING` |
| `TRACKING` | `should_dock` | `DOCKING_INIT` |
| `TRACKING` | Unsafe | `HOVERING` |
| `TRACKING` | Bird lost | `HOVERING` |
| `HOVERING` | `should_dock` or hover timeout | `DOCKING_INIT` |
| `HOVERING` | Bird re-detected and safe | `TRACKING` |
| `DOCKING_INIT` | Dock blocked (cart moving, people nearby) | `HOVERING` |
| `DOCKING_INIT` | Path clear | `DOCKING_APPROACH` |
| `DOCKING_APPROACH` | Unsafe during approach | `ABORT` |
| `DOCKING_APPROACH` | Within landing tolerance of the dock | `DOCKED` |
| `DOCKED` | Battery recovered and no docking triggers active | `IDLE` |
| `ABORT` | Recoverable | `HOVERING` |
| `ABORT` | Not recoverable | `EMERGENCY_LAND` |
| Any airborne state | Critical battery | `EMERGENCY_LAND` |

The cart-motion rule in the [main README](../README.md) applies: the drone never lands on a moving cart. While the cart is moving, docking waits in a safe state.

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

Implementation files and run commands have not yet been created. Proposed location: `src/state_machine/`, with tests in `tests/test_state_machine.py`.
