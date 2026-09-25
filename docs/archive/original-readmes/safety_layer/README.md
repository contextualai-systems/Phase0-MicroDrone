
# Safety Layer Module — Phase‑0 MicroDrone

## Purpose
The Safety Layer enforces constraints that prevent unsafe behavior.  
In Phase‑0, this module provides **safety stubs**, **battery checks**, and **emergency stop logic**.

## Responsibilities (Phase‑0)
- Provide battery check stub
- Provide emergency stop stub
- Enforce basic safety constraints
- Log safety events

## Inputs
- State vector (from Navigation)
- Motor command stub (from Motion Engine)
- Detection stub (from CV)

## Outputs
- Safety‑validated motor command stub
- Safety logs

## File Structure
- `run_safety.py` — main entry point
- `checks/` — battery + emergency stop stubs
- `constraints/` — safety rules
- `utils/` — logging helpers

## Good First Issues
- Add battery check stub
- Add emergency stop stub
- Add safety event logging

## Future Phases
- Real battery telemetry (Phase‑1)
- Obstacle‑avoidance safety (Phase‑2)
- Full safety override system (Phase‑3)
