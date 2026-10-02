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

## Starter structure

- `states/docking_state.py` — docking state names, exported as `DockingState`.
- `alignment/` — placeholder package for synthetic alignment logic.
- `utils/` — placeholder package for docking event logging.
- `run_docking.py` — placeholder entry point.

## Basic state controller

`states/docking_controller.py` provides `DockingController`, `DockingConfig`,
`DockingInput`, and `DockingUpdate`, exported from `docking.states`. The controller
starts in `IDLE` and advances at most one transition per `update(inputs, now_s)`.
It uses synthetic inputs and emits state decisions only; it sends no flight commands.

| Current state | Condition | Next state |
| --- | --- | --- |
| IDLE | Docking requested | APPROACH |
| APPROACH | Navigation reports near pad | SEARCH |
| SEARCH | Valid, fresh alignment | ALIGN |
| ALIGN | Horizontal error within tolerance | DESCEND |
| ALIGN | Missing, invalid, or stale alignment | SEARCH |
| DESCEND | Missing, invalid, or stale alignment | ABORT |
| DESCEND | Horizontal error exceeds tolerance | ALIGN |
| DESCEND | Within tolerance and landing confirmed | COMPLETE |
| Any active state | Safety supplies an abort reason | ABORT |

Otherwise, the state stays unchanged. Safety abort takes priority over progress.
`COMPLETE` and `ABORT` are terminal; create a new controller for another attempt.

### Synthetic input contract

- `north_error_m` and `east_error_m` are pad position minus vehicle position in
  meters, using the same local North-East-Down frame and origin. Horizontal error
  is the Euclidean distance; the controller does not calculate vertical targets.
- `alignment_timestamp_s` and `now_s` use the same simulation clock, in seconds.
  Times must be nonnegative; updates cannot move backwards. Future-dated,
  nonfinite, missing, or explicitly invalid alignment is unavailable.
- `DockingConfig` defaults to a **0.1 m** horizontal tolerance and **0.5 s** maximum
  alignment age, with inclusive boundaries. These are configurable test defaults,
  not validated flight thresholds; both must be finite and positive.
- Navigation supplies `docking_requested` and `near_pad` from validated state.
  `landing_confirmed` must represent fresh vehicle landing status plus landing
  evaluation, never just an issued land command. It is considered only in DESCEND
  with valid, fresh alignment inside tolerance.
- Safety supplies `abort_reason` (for example, `"low battery"` or `"operator abort"`).
  It owns battery thresholds and the actual hold/land/abort response. Consumers
  must stop requesting descent whenever the controller leaves DESCEND and pass
  all resulting motion targets through safety enforcement.

Each update returns the previous state, resulting state, reason, and timestamp.
`controller.transitions` contains an immutable snapshot of state-change records
for inspection or logging. Waiting updates are returned but not added to that history.

```python
from docking.states import DockingController, DockingInput

controller = DockingController()
result = controller.update(DockingInput(docking_requested=True), now_s=0.0)
print(result.state.value, result.reason)  # approach Docking requested
```

Run the state and controller tests from the repository root:

```bash
python3 -m unittest discover -s tests -p 'test_docking*.py' -v
```

Motion targets, navigation/safety integration, file logging, search timeouts,
and a runnable simulation sequence remain future work. The alignment and utils
packages and `run_docking.py` remain placeholders.
