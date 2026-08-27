# Safety Layer Module — Phase‑0 MicroDrone

The Safety Layer ensures all drone actions are safe. 
It prevents collisions, enforces no‑launch zones, and 
handles interior mode logic.

## What This Module Does
- Human detection + proximity rules
- No‑launch zone enforcement
- Interior mode logic (cart detection)
- Failsafe triggers (stop, hover, return)

## How to Run
```bash
python safety_layer/run_safety.py

File Structure
rules/ — safety rules + thresholds

detectors/ — human + interior detection

failsafes/ — emergency behaviors

utils/ — geometry + distance helpers

Good First Issues
Add a new safety rule

Improve human‑distance thresholding

Add interior‑mode test cases

Add logging for rule violations

Contributing
Keep rules transparent and documented

Add unit tests for each rule

Validate changes in simulation
