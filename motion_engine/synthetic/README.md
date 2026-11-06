# Synthetic motion data

Simple, hand-readable flight data for the Motion Engine: the drone going up,
down, left, right, forward, back, and spinning in place.

```bash
py -3 motion_engine/synthetic/generate_motion_data.py              # all scenarios -> motion_engine/motion_logs/
py -3 motion_engine/synthetic/generate_motion_data.py rotate square
py -3 motion_engine/synthetic/generate_motion_data.py --noise 0.02 # add a little sensor jitter
py -3 motion_engine/synthetic/generate_motion_data.py random_walk --seed 7  # a different random walk
py -3 motion_engine/synthetic/generate_motion_data.py --no-meta    # bare CSVs, no metadata lines
```

## Scenarios

Output goes to `motion_engine/motion_logs/` (one CSV per scenario + `overview.png`).


| File | What the drone does |
|---|---|
| `up_down.csv` | Climbs 2 m, pauses, comes back down |
| `left_right.csv` | Takes off, slides 3 m left, 3 m back right, lands |
| `forward_backward.csv` | Takes off, flies 3 m forward, 3 m back, lands |
| `rotate.csv` | Takes off, spins a full 360° left, then 360° right, lands |
| `square.csv` | Flies a 3 m square, turning 90° left at each corner |
| `full_tour.csv` | Every direction once, in order |
| `drift.csv` | Takes off and *tries* to hover while a steady wind (0.3 m/s east, 0.15 m/s north) pushes it away |
| `random_walk.csv` | Takes off, makes 12 random moves (picked from `--seed`), lands |

To make a new one, add a list of `(Direction, seconds)` to `SCENARIOS` in the script.
To add wind to a scenario, give it an entry in `DRIFT_MPS`.

`drift` and `random_walk` are for robustness testing: `drift` checks that a
consumer does not assume "commanded hover" means "not moving", and
`random_walk` produces an unpredictable but exactly reproducible path (same
`--seed` gives the same file).

## The model (deliberately simple)

- Constant speed while a direction is held: **1 m/s** horizontal, **0.5 m/s** vertical, **45°/s** yaw. Stops instantly when the move ends.
- Wind (`drift` only) adds a constant velocity while airborne. It moves the drone even though the commanded direction stays `hover`.
- Forward/back/left/right are **relative to where the drone faces** — after a 90° yaw-left, "forward" goes north.
- Positions are `motion_engine.contracts.Pose` values. There are no motor/throttle columns: Phase 0 never commands motors (PX4 stabilises), and the only command shape Motion accepts is `contracts.Command`. `Direction` is just this script's scenario vocabulary.
- Frame matches `motion_stubs.py`: x = east, y = north, z = up (m); yaw 0 = east, counter-clockwise positive.

## Metadata header

Each CSV starts with `# key=value` lines describing how it was made:

```text
# schema_version=1
# scenario=drift
# rate_hz=10.0
# dt_s=0.1
# noise_m=0.0
# seed=0
# drift_mps=0.3,0.15
# moves=3
# duration_s=14
# frame=local_enu
# clock=simulation
# synthetic=true
# generator=motion_engine/synthetic/generate_motion_data.py
t_s,timestamp_ns,step,direction,x_m,y_m,z_m,yaw_deg,vx_mps,...
```

Plain `csv.DictReader` does not skip `#` lines, so read the files with the
helper in the script or with pandas:

```python
from generate_motion_data import load_csv          # motion_engine/synthetic on sys.path
meta, rows = load_csv("motion_engine/motion_logs/square.csv")
meta["scenario"], meta["rate_hz"]                   # ('square', '10.0')

import pandas as pd
df = pd.read_csv("motion_engine/motion_logs/square.csv", comment="#")
```

## Columns

| Column | Meaning |
|---|---|
| `t_s` | time (s), 10 samples per second by default |
| `timestamp_ns` | the same time as an integer number of simulation nanoseconds from 0 -- the shared clock in [INTEGRATION_README.md](../../INTEGRATION_README.md) section 4 |
| `step` | index of the move in the scenario |
| `direction` | the `Direction` being held (`hover`, `up`, `yaw_left`, …) |
| `x_m`, `y_m`, `z_m` | position (m) |
| `yaw_deg` | heading, 0–360 |
| `vx_mps`, `vy_mps`, `vz_mps`, `yaw_rate_dps` | the drone's actual velocity this sample (world frame, includes wind) |

## Expected output

Running the script with no arguments prints one line per scenario:

```text
up_down            121 rows,  12.0 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
left_right         131 rows,  13.0 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
forward_backward   131 rows,  13.0 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
rotate             231 rows,  23.0 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
square             261 rows,  26.0 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
full_tour          281 rows,  28.0 s, ends at x=+0.00 y=+0.00 z=+0.00 yaw=0
drift              141 rows,  14.0 s, ends at x=+4.17 y=+2.08 z=+0.00 yaw=0
random_walk        226 rows,  22.5 s, ends at x=+4.50 y=-1.00 z=+0.00 yaw=292
plot -> motion_engine/motion_logs/overview.png
```

Every hand-written scenario ends back at the origin; `drift` and
`random_walk` do not, which is the point. The last rows of `square.csv`
show the landing (descending at 0.5 m/s, then stopped on the ground):

```text
t_s,timestamp_ns,step,direction,x_m,y_m,z_m,yaw_deg,vx_mps,vy_mps,vz_mps,yaw_rate_dps
25.9,25900000000,11,down,0.0,0.0,0.05,0.0,0.0,0.0,-0.5,0.0
26.0,26000000000,12,hover,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
```

`overview.png` (written when matplotlib is installed) has one row per
scenario: x/y/z position against time on the left, heading on the right.
To plot a single file yourself:

```python
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("motion_engine/motion_logs/drift.csv", comment="#")
fig, (pos, top) = plt.subplots(1, 2, figsize=(10, 3.5))
for col in ("x_m", "y_m", "z_m"):
    pos.plot(df.t_s, df[col], label=col)
pos.set_xlabel("time (s)"); pos.set_ylabel("m"); pos.legend()
top.plot(df.x_m, df.y_m); top.set_xlabel("x east (m)"); top.set_ylabel("y north (m)")
top.set_aspect("equal"); top.set_title("ground track")
plt.tight_layout(); plt.show()
```
