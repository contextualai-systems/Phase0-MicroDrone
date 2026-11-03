# Phase-0 Autonomous MicroDrone Wildlife Deterrence System

### CONTEXTUAL·AI SYSTEMS — RCOS Fall 2026

![CONTEXTUAL·AI SYSTEMS Logo](assets/assets/banner/ContextualAI%20logo.png)

![Phase-0 QR Code](assets/qr/QR%20Code%20ContextualAI%20Systems.png)

---

## Recruitment materials

- [RPI Recruitment Follow-Up Presentation (PDF)](assets/presentations/RPI_Recruitment_Follow_Up_Presentation.pdf)
- [Student Engineering Packet V3 (PDF)](docs/STUDENT%20ENGINEERING%20PACKET%20V3.pdf)
- [Student Engineering Packet V3 (DOCX)](docs/STUDENT%20ENGINEERING%20PACKET%20V3.docx)

The engineering packet and recruitment materials provide broader project
context. [START_HERE.md](START_HERE.md) is authoritative for the current
semester scope.

## Project scope

Phase 0 is a software and synthetic-simulation project. It does not require
physical flight or hardware integration. Current code includes individual
synthetic CV, motion, safety, docking, and mission-state components; it is not
yet a fully integrated runtime. The proposed PX4/Gazebo environment also has
not been validated as a repository setup.

See [START_HERE.md](START_HERE.md) for scope and [INTEGRATION_README.md](INTEGRATION_README.md)
for the target shared-loop contract.

## Repository structure

```text
Phase0-MicroDrone/
├── assets/                 # Recruitment materials and project images
├── cv/                     # Synthetic camera input and observations
├── docking/                # Docking state and controller
├── docs/                   # Engineering packet and archived READMEs
├── motion_engine/          # Motion contract, stub, and synthetic data
├── navigation/             # Navigation workstream documentation
├── safety_layer/           # Safety policy and command checks
├── simulation/             # Simulation workstream documentation
├── state_machine/          # Mission transitions and scripted demo
├── tests/                  # Component tests
├── universal_log/          # Structured log helper
├── INTEGRATION_README.md   # Target integrated-loop contract
└── START_HERE.md           # Authoritative Phase 0 scope
```

The architecture image is [Drone Centric Architecture](assets/assets/architecture/Drone%20Centric%20Architecture.png).

## Getting started

There is no repository-wide dependency manifest or integrated application
entry point yet. Clone the repository and run the component demo:

```sh
git clone https://github.com/UtopianComet/Phase0-MicroDrone.git
cd Phase0-MicroDrone
python -m state_machine.run_loop
```

The state-machine tests use `pytest` (install it if needed):

```sh
python -m pip install pytest
python -m pytest state_machine -v
```

Follow the relevant [module guides](START_HERE.md#module-guides) for available
component-specific requirements and tests. Some workstreams do not yet have
runnable code. The scripted state-machine demo does not connect CV, Motion,
Safety, Docking, PX4, ROS 2, or Gazebo into a single runtime.

## Contributing

Module test workflows and local commands: [CI guide](docs/CI.md).

Fork the repository, create a feature branch, and submit a pull request. Follow
the contribution guidance in [CONTRIBUTING.md](CONTRIBUTING.md). Run relevant
synthetic tests for changes to safety or movement behavior; simulation results
are not evidence of physical-flight safety.

## RCOS workstreams

- Computer Vision
- Motion Engine
- Navigation
- Safety Layer
- Docking
- Simulation and integration

## Contact

**Jeffrey N. Barat**
Founder — CONTEXTUAL·AI SYSTEMS

RPI RCOS Project Lead

Palm Beach Gardens, FL
