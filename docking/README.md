# Docking — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md)

## Purpose

Return to and land on a stationary pad in simulation. “Docking” means landing within an agreed tolerance; charging, magnetic contact, and moving-platform landing are outside the required demonstration.

Motion/navigation owns the docking sequence, CV only publishes timestamped observations, and simulation owns the pad and evaluation scenario.

## Inputs and outputs

- Inputs: estimated vehicle state, documented synthetic pad/alignment inputs, known pad configuration, safety constraints.
- Outputs: alignment/descent targets through motion and safety, docking state, completion/failure status, and logs.
- Define units and frames for synthetic alignment fixtures. Do not interpret a CV stub bounding box as a measured pad position.

## First tasks

1. Define approach, search, align, descend, complete, and abort transitions.
2. Return to a known pad vicinity using navigation.
3. Use known pad coordinates and synthetic alignment fixtures to exercise alignment logic.
4. Define limits, alignment tolerance, detection freshness, and descent conditions.
5. Handle missing/stale fixture inputs and aborts with safety.
6. Measure landing position error over repeated trials.

The required demonstration uses synthetic alignment inputs, not visual pose estimation. Marker detection is optional CV stretch work; pose estimation belongs in Phase 1.

## Acceptance evidence

The drone approaches, aligns, and lands within the agreed tolerance. Logs show observations and transition reasons. Tests include missing/stale synthetic inputs and low-battery behavior. A land command alone does not count as confirmed landing; define completion using vehicle status and evaluation data.

Implementation files and run commands have not yet been created.
