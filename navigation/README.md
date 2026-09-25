# Navigation — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Maintain the application's view of vehicle state and manage a simple waypoint mission with return-to-pad behavior. Use PX4 estimated state initially; custom state estimation is not required.

## Inputs and outputs

- Inputs: timestamped PX4 state, mission configuration, safety status, docking progress.
- Outputs: motion targets, docking requests, mission state, and transition logs.
- State includes position, velocity, orientation, frame, timestamp, flight mode, and validity.
- Keep estimated state, command targets, and simulator ground truth distinct. Ground truth is useful for evaluation.

## First tasks

1. Define state and target contracts with motion, CV, docking, and safety.
2. Record telemetry with explicit units and coordinate frames.
3. Test mission transitions using fake telemetry.
4. Execute a small waypoint route in simulation.
5. Hand control to docking on return and handle completion or abort.

Candidate states: IDLE, TAKEOFF, WAYPOINT, RETURN, ALIGN, LAND, ABORT. Agree which module owns each transition; docking owns its internal alignment/descent sequence.

## Acceptance evidence

The mission follows a configured route and returns to the pad area. Logs explain every state change. Invalid or stale telemetry produces a defined response rather than continued use of old state.

Implementation files and run commands have not yet been created.
