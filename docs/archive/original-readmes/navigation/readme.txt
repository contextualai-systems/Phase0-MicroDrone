# Navigation Module — Phase‑0 MicroDrone

## Purpose
The Navigation module defines the drone’s internal understanding of its position and movement.  
Phase‑0 focuses on a **basic state vector**, **dead‑reckoning stub**, and **logging**.

## Responsibilities (Phase‑0)
- Maintain a simple state vector (x, y, z, heading)
- Provide dead‑reckoning placeholder logic
- Log navigation updates
- Output state vector to Motion Engine

## Inputs
- Motion commands (from Motion Engine)
- Detection stub (from CV)
- Safety constraints (from Safety Layer)

## Outputs
- Updated state vector
- Navigation logs

## File Structure
- `run_navigation.py` — main entry point
- `state/` — state vector structure
- `dead_reckoning/` — placeholder logic
- `utils/` — logging helpers

## Good First Issues
- Create basic state vector structure
- Add dead‑reckoning placeholder logic
- Add navigation logging

## Future Phases
- Real IMU + GPS integration (Phase‑1)
- Vision‑based navigation (Phase‑2)
- Obstacle avoidance (Phase‑2)
- Full autonomy loop (Phase‑3)
