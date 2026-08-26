# Contributing to Phase‑0 MicroDrone
Thank you for your interest in contributing to the Phase‑0 Autonomous MicroDrone Project!  
This is an RCOS Fall 2026 initiative to design, build, and test a real autonomous wildlife‑deterrence drone.  
All contributions—code, documentation, simulation assets, electronics work, and testing—are welcome.

---

## Project Overview
The Phase‑0 MicroDrone is a roof‑mounted autonomous deterrence drone that:
- Detects approaching birds
- Classifies species
- Selects deterrence behaviors
- Executes multi‑axis motion
- Operates safely around humans
- Docks autonomously

The full technical specification is available in `/docs/Student Engineering Packet.pdf`.

---

## How to Get Started

### 1. Fork the Repository
Create your own fork of the repo:
https://github.com/contextualai-systems/Phase0-MicroDrone

### 2. Clone Your Fork
git clone https://github.com/<your-username>/Phase0-MicroDrone.git  
cd Phase0-MicroDrone

### 3. Create a Branch
Use descriptive branch names:
git checkout -b feature-motion-engine  
git checkout -b fix-camera-init  
git checkout -b docs-update

### 4. Install Dependencies
A requirements file or Docker container will be provided.  
Simulation instructions for Gazebo/Webots will also be included.

---

## Project Structure
Phase0-MicroDrone/  
├── docs/                 ← Engineering Packet, pitch deck, diagrams  
├── src/                  ← Python code (CV pipeline, motion engine, safety layer)  
├── simulation/           ← Gazebo/Webots worlds, models, scripts  
├── hardware/             ← Motors, sensors, Jetson pinouts  
├── electronics/          ← Wiring diagrams, GPIO maps  
└── tests/                ← Unit tests for motion, safety, CV

---

## Good First Issues
Beginner‑friendly tasks are listed under **Issues**:

Examples:
- Add logging to CV pipeline
- Write documentation for motion engine
- Create simple OpenCV script
- Build basic tilt/rotation behavior in simulation
- Add interior‑sensor mock data
- Improve simulation environment assets
- Create unit tests for safety rules

---

## Code Guidelines

### Python
- Follow PEP‑8 style guidelines
- Use clear, descriptive variable names
- Add comments for non‑obvious logic
- Keep functions small and modular

### Motion Engine / Safety Layer
- Use deterministic logic
- Document all thresholds and parameters
- Include test cases in `/tests`

### Computer Vision
- Keep CV scripts in `src/cv/`
- Include sample images or mock data when possible
- Log intermediate results for debugging

### Simulation
- Place world files and models in `simulation/`
- Document how to run your simulation scenario

---

## Testing
Before submitting a pull request:
- Run unit tests in `/tests`
- Test motion behaviors in simulation
- Validate safety rules (no‑launch zones, interior mode, human detection)
- Confirm code runs on Jetson Nano if applicable

---

## Pull Requests
All contributions must be submitted through a Pull Request (PR).

### PR Requirements:
- Clear description of the change
- Reference any related Issue
- Screenshots or logs if relevant
- Tests included when possible
- No breaking changes without discussion

### PR Review Process:
- Maintainers review for correctness and clarity
- Requested changes must be addressed
- Once approved, PR will be merged into `main`

---

## Collaboration Expectations
- Be kind, supportive, and curious
- Ask questions early
- Attend weekly RCOS stand‑ups
- Coordinate with other sub‑teams (CV, Motion, Safety, Docking, Simulation)
- Document everything you build

---

## Contact
**Jeffrey Barat**  
Founder, ContextualAI Systems  
Palm Beach Gardens, FL

---

Thank you for contributing to a real autonomous system.  
Your work will help build the Phase‑0 MicroDrone prototype this semester.
