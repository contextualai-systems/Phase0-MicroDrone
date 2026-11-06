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

## Implemented

Navigation decides *where* to go; Motion executes. Nothing here calls Motion
or changes mission state.

| File | What it does |
|---|---|
| [`tracking.py`](tracking.py) | `standoff_target(vehicle_pose, bearing_deg, distance_m, standoff_m=15)` turns synthetic tracking geometry into the pose Motion flies as `TRACK_TARGET` (moved out of `MotionStub`) |
| [`routes.py`](routes.py) | `build_route(points, altitude_m, start)` turns a list of points into waypoint poses, facing the direction of travel, for Motion's [`WaypointExecutor`](../motion_engine/waypoints.py) |

```bash
py -3 -m pytest tests/test_navigation.py motion_engine/tests/test_waypoints.py
```

The candidate waypoint mission states above (`TAKEOFF`, `WAYPOINT`, ...) are
still deferred by [INTEGRATION_README §2](../INTEGRATION_README.md); whether
waypoint missions are in this milestone is an open question for Jeff and the
team.
