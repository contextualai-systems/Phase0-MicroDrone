# Safety Layer — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Enforce application-level constraints on simulated mission commands from the first flight milestone. PX4's own failsafes remain active. Simulation tests demonstrate behavior under specified conditions; they do not establish hardware safety.

Motion/navigation leads implementation. CV publishes observations only; simulation supplies failure fixtures, and the safety layer owns all safety decisions.

## Inputs and outputs

- Inputs: vehicle-state freshness, requested targets, configured limits/geofence, simulated battery status, operator abort, and docking/vision validity.
- Outputs: permitted action or override, reason, timestamp, and event logs.
- All mission and docking targets pass through enforcement before being sent to PX4.
- Missing or stale required inputs must have explicit behavior.

## First tasks

1. Define position/velocity limits, geofence, and state/command timeouts.
2. Define controlled abort behavior for each flight state.
3. Reject or constrain invalid targets before transmission.
4. Inject stale telemetry, command loss, and low battery in simulation.
5. Test missing or stale synthetic inputs during docking with the docking team.

Distinguish a controlled abort (such as hold or land under specified conditions) from motor termination. Document when each action is available. Avoid blanket rules that disable stabilization while demanding a controlled landing.

Use explicitly labeled synthetic inputs for observation-related tests. A CV stub does not demonstrate real-world detection capability.

## Acceptance evidence

Each scenario records the input condition, expected response, observed response, and timing. Agree thresholds and pass/fail criteria before testing. Verify that ordinary mission commands cannot override an active constraint.

Implementation files and run commands have not yet been created.
