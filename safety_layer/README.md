# Safety Layer — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Enforce application-level constraints on simulated mission commands. Phase 0 is
fully synthetic: no PX4, no Gazebo, pretend 3D coordinates and predefined states.
Simulation tests demonstrate behavior under specified conditions; they do not
establish hardware safety.

Motion/navigation leads implementation. CV publishes observations only;
simulation supplies failure fixtures, and the safety layer owns all safety decisions.

## Inputs and outputs

- Inputs: the `safety` section of each observation — people_nearby, cart_moving,
  weather, camera_ok, comm_ok — plus the current state and requested target.
  Battery status is owned by the docking module.
- Outputs: permitted action or override, reason, timestamp, and event logs.
- Every movement request passes through safety before motion acts on it.
- Missing or stale required inputs must have explicit behavior: unknown is treated
  as unsafe, never as safe.

## Implementation

- `safety_layer/safety_decision.py` — `SafetyDecision` (allowed, action, reason,
  timestamp) and `check_request()`, a Phase-0 placeholder that approves all requests.
- `tests/test_safety_decision.py` — run with
  `python -m unittest tests.test_safety_decision -v`

## First tasks

1. Lock the Safety → State Machine contract: which states a decision may produce.
2. Define controlled abort behavior for each flight state.
3. Define thresholds for each safety field, including which weather values are safe.
4. Reject or constrain invalid targets before transmission.
5. Inject command loss, camera failure, and stale inputs in simulation.
6. Adopt the shared logging format once finalized.

Distinguish a controlled abort (such as hold or land under specified conditions)
from motor termination. Document when each action is available.

Bird type and distance are not safety inputs. Distance cannot be derived from a
2D image, so no safety rule should depend on it.

## Acceptance evidence

Each scenario records the input condition, expected response, observed response,
and timing. Agree thresholds and pass/fail criteria before testing. Verify that
ordinary mission commands cannot override an active constraint.