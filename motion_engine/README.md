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


