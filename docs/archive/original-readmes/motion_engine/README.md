# Motion Engine Module — Phase‑0 MicroDrone

## Purpose
The Motion Engine module defines how the drone *would* move — without actually controlling hardware in Phase‑0.  
This semester focuses on **motor command stubs**, **PID structure**, and **test harnesses**.

## Responsibilities (Phase‑0)
- Provide motor command stubs (no real ESC output)
- Define PID controller structure
- Log motor commands for debugging
- Provide a test harness for simulated movement

## Inputs
- Desired motion command (from Navigation)
- State vector (from Navigation)
- Safety constraints (from Safety Layer)

## Outputs
- Motor command stub (placeholder values)
- Logged motor activity

## File Structure
- `run_motion.py` — main entry point
- `pid/` — PID controller structure
- `commands/` — motor command stubs
- `utils/` — logging + test harness

## Good First Issues
- Add PID parameter placeholders
- Create a motor command logging function
- Add a simple test harness for simulated movement

## Future Phases
- Real ESC control (Phase‑1)
- Multi‑axis stabilization (Phase‑2)
- Full motion control loop (Phase‑3)


