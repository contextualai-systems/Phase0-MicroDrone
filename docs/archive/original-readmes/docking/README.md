# Docking Module — Phase‑0 MicroDrone

## Purpose
The Docking module defines how the drone aligns with and returns to its dock.  
Phase‑0 focuses on **alignment logic**, **docking states**, and **event logging** — no real hardware yet.

## Responsibilities (Phase‑0)
- Provide docking state machine
- Provide alignment stub logic
- Log docking events
- Output docking status to Navigation + Safety

## Inputs
- State vector (from Navigation)
- Detection stub (from CV)
- Safety constraints (from Safety Layer)

## Outputs
- Docking state updates
- Docking logs

## File Structure
- `run_docking.py` — main entry point
- `states/` — docking state machine
- `alignment/` — alignment stub logic
- `utils/` — logging helpers

## Good First Issues
- Add docking state structure
- Add alignment stub logic
- Add docking event logging
- 

## Future Phases
- Real docking pad hardware (Phase‑1)
- Vision‑based alignment (Phase‑2)
- Autonomous docking + charging (Phase‑3)


