# Motion Engine Module — Phase‑0 MicroDrone

The Motion Engine executes all drone movement behaviors. 
It converts high‑level commands (rise, tilt, rotate) into safe, 
bounded motion primitives.

## What This Module Does
- Implements rise/tilt/rotate primitives
- Provides interior micro‑motion behaviors
- Enforces safety constraints from the Safety Layer
- Interfaces with the simulation controller

## How to Run
```bash
python motion_engine/run_motion.py

File Structure
run_motion.py — main entry point

primitives/ — rise, tilt, rotate, hover

controllers/ — PID or rule‑based control

utils/ — math helpers, smoothing

Good First Issues
Add a new motion primitive

Improve rotation stability

Add smoothing to tilt behavior

Add logging for motion commands

Contributing
Keep primitives modular

Add diagrams for new behaviors

Test in simulation before merging
