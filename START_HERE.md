# Start Here — Phase 0 MicroDrone, RCOS Fall 2026

## Working proposal

Phase 0 builds an open-source simulation and reusable robotics foundation for a future microdrone platform. The broader product concept is a drone that operates from a golf cart dock; this semester focuses on software and simulation, with no physical flight or hardware integration.

**Semester demonstration:** a simulated drone takes off, follows waypoints, returns to a stationary landing pad, with flight logs and tested safety overrides.

This guide defines the authoritative Phase 0 scope for the RCOS team. It describes proposed work, not completed functionality. The repository currently contains documentation and placeholder files; a reproducible runtime and dependency setup still need to be established.

## Phase 0 scope

| Workstream | Required work |
| --- | --- |
| Simulation | Worlds, existing vehicle models, simulated sensors, and repeatable fixtures |
| CV | Synthetic frames, preprocessing, logging, and a simple detection stub |
| Motion and navigation | Generic movement commands, waypoint logic, and state logging |
| Safety and docking | Constraints, state transitions, and alignment tests using explicit synthetic inputs |

The demonstration and tests use public dependencies, synthetic inputs, and documented test logic. Physical construction and hardware integration are outside Phase 0.

**CV completion does not require a working detector.** 2D marker detection is a stretch goal. Pose estimation belongs in Phase 1; further work will be scoped separately.

The [Student Engineering Packet V3](docs/STUDENT%20ENGINEERING%20PACKET%20V3.pdf) describes a broader system and contains conflicting publication, ownership, hardware, and student-deliverable requirements. Use this working proposal for semester planning; the project stakeholders still need to reconcile those requirements. This guide does not resolve ownership or licensing agreements. The repository includes an [MIT license](LICENSE.md); third-party dependencies and assets retain their own licenses.

## Proposed stack

| Tool | Purpose |
| --- | --- |
| Gazebo | World, vehicle physics, and simulated camera/sensor data |
| PX4 SITL (software in the loop) | Autopilot running on a development computer |
| ROS 2 | Communication between perception, navigation, and PX4 |
| Python + OpenCV | Application logic, preprocessing, and synthetic detection output |

The candidate environment is Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and a pinned compatible PX4 release. Validate this combination on a reference machine and record exact versions before onboarding everyone. It is not yet a tested repository setup.

Start with an existing PX4 quadrotor model. PX4 provides low-level stabilization; RCOS commands positions, velocities, and yaw through an external-control interface. Custom motor control and a replacement flight PID are not required for the demonstration.

See the [simulation guide](simulation/README.md) for setup acceptance criteria and upstream documentation.

## Architecture and shared interfaces

The intended data flow is:

1. Gazebo supplies simulated sensor data; PX4 supplies estimated vehicle state.
2. CV publishes timestamped synthetic detection-stub observations; it does not choose actions.
3. Navigation manages the mission and requests docking when appropriate.
4. Docking produces alignment/descent targets from vehicle state and documented synthetic pad/alignment fixtures. CV stub boxes are not 3D position measurements.
5. Safety validates or overrides requested actions.
6. Motion sends permitted targets to PX4; telemetry feeds back into the system.

PX4 failsafes remain active alongside application-level constraints. CV observations do not directly command motors.

Before implementation, agree on these contracts:

| Interface | Minimum information |
| --- | --- |
| Vehicle state | Timestamp, position, velocity, orientation, coordinate frame, flight mode, validity |
| Vision observation | Timestamp, camera ID, image dimensions, stub label/bounding box, validity, synthetic-source flag |
| Motion request | Position or velocity target, yaw, frame, limits, expiration |
| Safety decision | Allowed action or override, reason, timestamp |
| Mission/docking status | Current state, transition reason, completion or failure |

Specify units and coordinate conversions explicitly, including camera frames and PX4 conventions. Yaw is one component of orientation. Keep commanded targets separate from estimated vehicle state, and keep simulation ground truth separate from the state used by the controller.

## Teams and first deliverables

| Workstream | Responsibilities | First deliverable |
| --- | --- | --- |
| Computer Vision | Synthetic frames, preprocessing, detection stub, observation logs | Repeatable timestamped stub output, including empty/invalid cases |
| Motion and Navigation | PX4 telemetry, bounded commands, waypoint mission, docking execution | Scripted takeoff, hold, and landing with logs and an abort path |
| Simulation and integration | Shared environment, sensors, landing pad, reproducible launch | A second member reproduces the stock simulation and reads telemetry/camera data |

Safety and docking are shared integration responsibilities. Motion/navigation leads command enforcement and docking states; CV only publishes observations for other modules to consume; simulation supplies scenarios and injected failures. Assign an issue owner for each deliverable.

Simulation starts immediately, but does not block offline CV, interface design, or mission-state tests using fake telemetry.

## Semester milestones

| Month | Required outcome |
| --- | --- |
| September | Agree scope/interfaces; reproduce stock simulation; read telemetry and camera frames; publish synthetic detection observations; demonstrate basic flight and abort handling |
| October | Execute waypoints; integrate timestamped stub observations; enforce command limits and geofence behavior |
| November | Return to the stationary pad, align, and descend; test missing synthetic inputs, stale telemetry, command loss, and simulated low battery |
| December | Repeat end-to-end demonstrations; report landing error and success rate; document setup, limitations, and handoff |

For this semester, **docking means landing within an agreed tolerance on a stationary pad using documented synthetic alignment inputs**. It does not include charging, magnetic engagement, or landing on a moving cart.

Stretch work: 2D marker detection. It is not required for the integrated demonstration, which must remain runnable with synthetic fixtures.

## Definition of done

- Another member can reproduce the documented environment and demonstration.
- The mission completes takeoff, waypoints, return, alignment, and landing.
- Logs capture estimated state, requested commands, observations, and safety/state transitions.
- Missing synthetic inputs, stale data, command loss, geofence violations, and low-battery scenarios have defined, tested outcomes.
- Landing tolerance, timeouts, command limits, trial count, and success criteria are agreed before final testing; results include failures.
- Simulation results are reported as simulation evidence, not real-world safety validation.

## Module guides

The [original README archive](docs/archive/original-readmes/INDEX.md) preserves the previous central and module documentation for historical reference.

- [Computer Vision](cv/README.md)
- [Motion Engine](motion_engine/README.md)
- [Navigation](navigation/README.md)
- [Safety Layer](safety_layer/README.md)
- [Docking](docking/README.md)
- [Simulation](simulation/README.md)

The existing `src/` and `tests/` directories are placeholders for implementation and verification. Hardware and electronics directories are not Phase 0 build assignments.

## First pre-code meeting

1. Agree the demonstration and record unresolved boundary questions.
2. Assign the reference simulation setup and a second person to reproduce it.
3. Define shared messages, coordinate frames, and failure behavior.
4. Assign the first deliverables above as issues with acceptance criteria.
5. Schedule an early integrated demonstration and reconcile older onboarding documents.

There is no project install or launch command yet. Add one only after it has been verified from a clean checkout.
