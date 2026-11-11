"""Synthetic motion data for the Motion Engine -- simple, readable scenarios.

Each scenario is a short list of (direction, seconds) moves -- "go up for 2 s,
go left for 3 s, spin right for 4 s" -- played through a very simple model:

* The drone moves at a constant speed while a direction is held
  (1 m/s sideways/forward, 0.5 m/s up/down, 45 deg/s yaw) and stops dead
  when the move ends. No drag, no wind, no overshoot -- easy to read.
* Positions are `motion_engine.contracts.Pose` values, the same type the
  shared contract and `MotionStub` use. There are no motor/throttle columns:
  Phase 0 never commands motors (PX4 does stabilisation), so the only
  command shape in the Motion Engine is `contracts.Command`.

Frame (same as motion_stubs.py): x = east, y = north, z = up (metres),
yaw in degrees, 0 = facing east, counter-clockwise positive.
FORWARD/BACKWARD/LEFT/RIGHT are relative to where the drone is facing, so
after a 90 deg yaw-left, "forward" flies north instead of east.

Usage (from the repo root):
    py -3 motion_engine/synthetic/generate_motion_data.py            # all scenarios
    py -3 motion_engine/synthetic/generate_motion_data.py --noise 0.02
    py -3 motion_engine/synthetic/generate_motion_data.py --rate 20 --out some/folder
    py -3 motion_engine/synthetic/generate_motion_data.py random_walk --seed 7

Outputs one CSV per scenario, plus a PNG plot if matplotlib is installed,
into motion_engine/motion_logs/ (change with --out).

Each CSV starts with a few `# key=value` metadata lines (scenario, rate,
noise, seed, ...). Read them back with `load_csv()` below, or with pandas:
`pd.read_csv(path, comment="#")`. Pass --no-meta for a bare CSV.

Time is written twice: `t_s` (float seconds, for eyes and plots) and
`timestamp_ns` (integer simulation nanoseconds starting at 0), which is the
clock every module uses under INTEGRATION_README.md section 4.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
from pathlib import Path
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

_MOTION_ENGINE_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _MOTION_ENGINE_ROOT.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from motion_engine.contracts import Pose  # noqa: E402


class Direction(Enum):
    """The scenario vocabulary: what the drone does for one move.

    A description of a scripted move, not a command type -- the commands
    Motion accepts are `contracts.Command`. FORWARD/BACKWARD/LEFT/RIGHT are
    relative to the heading; YAW_* turn in place.
    """

    HOVER = "hover"
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"
    DOWN = "down"
    YAW_LEFT = "yaw_left"
    YAW_RIGHT = "yaw_right"


D = Direction

# --------------------------------------------------------------------------
# Speeds -- change these to make the drone faster or slower.
# --------------------------------------------------------------------------
HORIZONTAL_SPEED_MPS = 1.0   # forward / backward / left / right
VERTICAL_SPEED_MPS = 0.5     # up / down
YAW_RATE_DPS = 45.0          # yaw_left / yaw_right

# --------------------------------------------------------------------------
# Scenarios -- each is a list of (direction, how many seconds to hold it).
# Every scenario starts on the ground at (0, 0, 0) facing east.
# --------------------------------------------------------------------------
SCENARIOS: Dict[str, List[Tuple[Direction, float]]] = {
    # Climb 2 m, pause, come back down.
    "up_down": [
        (D.HOVER, 1), (D.UP, 4), (D.HOVER, 2), (D.DOWN, 4), (D.HOVER, 1),
    ],
    # Take off, slide 3 m left, 3 m back right, land.
    "left_right": [
        (D.UP, 2), (D.HOVER, 1), (D.LEFT, 3), (D.HOVER, 1),
        (D.RIGHT, 3), (D.HOVER, 1), (D.DOWN, 2),
    ],
    # Take off, fly 3 m forward, 3 m back, land.
    "forward_backward": [
        (D.UP, 2), (D.HOVER, 1), (D.FORWARD, 3), (D.HOVER, 1),
        (D.BACKWARD, 3), (D.HOVER, 1), (D.DOWN, 2),
    ],
    # Take off, spin a full circle left (8 s x 45 deg/s = 360), then right, land.
    "rotate": [
        (D.UP, 2), (D.HOVER, 1), (D.YAW_LEFT, 8), (D.HOVER, 1),
        (D.YAW_RIGHT, 8), (D.HOVER, 1), (D.DOWN, 2),
    ],
    # Fly a 3 m square by turning 90 deg left at each corner -- shows that
    # "forward" follows the heading.
    "square": [
        (D.UP, 2), (D.HOVER, 1),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.FORWARD, 3), (D.YAW_LEFT, 2),
        (D.HOVER, 1), (D.DOWN, 2),
    ],
    # Everything once, in order.
    "full_tour": [
        (D.HOVER, 1), (D.UP, 4), (D.HOVER, 1),
        (D.FORWARD, 2), (D.BACKWARD, 2), (D.LEFT, 2), (D.RIGHT, 2),
        (D.UP, 2), (D.DOWN, 2),
        (D.YAW_LEFT, 2), (D.YAW_RIGHT, 2),
        (D.HOVER, 1), (D.DOWN, 4), (D.HOVER, 1),
    ],
    # Take off and try to hover while a steady wind pushes the drone
    # (see DRIFT_MPS) -- the command says "hover", the position says otherwise.
    # Useful for checking that consumers do not assume commanded == actual.
    "drift": [
        (D.UP, 2), (D.HOVER, 10), (D.DOWN, 2),
    ],
}

#: Constant wind (east, north) in m/s applied to scenarios listed here.
#: It moves the drone even though the commanded direction is unchanged, and
#: it is included in vx_mps / vy_mps, which are the drone's *actual* velocity.
DRIFT_MPS: Dict[str, Tuple[float, float]] = {
    "drift": (0.3, 0.15),
}

#: Random-walk length and the hold times it picks from (seconds).
RANDOM_WALK_MOVES = 12
RANDOM_WALK_HOLDS = (0.5, 1.0, 1.5, 2.0)


def random_walk(seed: int) -> List[Tuple[Direction, float]]:
    """Take off, make RANDOM_WALK_MOVES random moves, land.

    Deterministic for a given seed, so a robustness test that fails can be
    replayed exactly with `--seed`.
    """
    rng = random.Random(seed)
    choices = [D.FORWARD, D.BACKWARD, D.LEFT, D.RIGHT, D.YAW_LEFT, D.YAW_RIGHT, D.HOVER]
    moves: List[Tuple[Direction, float]] = [(D.UP, 2), (D.HOVER, 1)]
    for _ in range(RANDOM_WALK_MOVES):
        moves.append((rng.choice(choices), rng.choice(RANDOM_WALK_HOLDS)))
    moves += [(D.HOVER, 1), (D.DOWN, 2)]
    return moves


#: Scenarios built from the seed at run time instead of written out by hand.
SEEDED_SCENARIOS: Dict[str, Callable[[int], List[Tuple[Direction, float]]]] = {
    "random_walk": random_walk,
}

ALL_SCENARIOS: List[str] = list(SCENARIOS) + list(SEEDED_SCENARIOS)


def scenario_moves(name: str, seed: int = 0) -> List[Tuple[Direction, float]]:
    """The (direction, seconds) list for any scenario name."""
    if name in SEEDED_SCENARIOS:
        return SEEDED_SCENARIOS[name](seed)
    return SCENARIOS[name]

COLUMNS = [
    "t_s", "timestamp_ns", "step", "direction",
    "x_m", "y_m", "z_m", "yaw_deg",
    "vx_mps", "vy_mps", "vz_mps", "yaw_rate_dps",
]


def _velocity(direction: Direction, yaw_deg: float) -> Tuple[float, float, float, float]:
    """World-frame (vx, vy, vz, yaw_rate) for holding `direction` at `yaw_deg`."""
    fwd, left, up, yaw_rate = 0.0, 0.0, 0.0, 0.0
    if direction == D.FORWARD:
        fwd = HORIZONTAL_SPEED_MPS
    elif direction == D.BACKWARD:
        fwd = -HORIZONTAL_SPEED_MPS
    elif direction == D.LEFT:
        left = HORIZONTAL_SPEED_MPS
    elif direction == D.RIGHT:
        left = -HORIZONTAL_SPEED_MPS
    elif direction == D.UP:
        up = VERTICAL_SPEED_MPS
    elif direction == D.DOWN:
        up = -VERTICAL_SPEED_MPS
    elif direction == D.YAW_LEFT:
        yaw_rate = YAW_RATE_DPS
    elif direction == D.YAW_RIGHT:
        yaw_rate = -YAW_RATE_DPS
    # Rotate body (forward, left) into world (east, north).
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return fwd * c - left * s, fwd * s + left * c, up, yaw_rate


def generate(moves: List[Tuple[Direction, float]], rate_hz: float = 10.0,
             noise_m: float = 0.0, seed: int = 0,
             drift_mps: Tuple[float, float] = (0.0, 0.0)) -> List[dict]:
    """Play `moves` through the simple model; return one dict per sample.

    `drift_mps` is a constant (east, north) wind added to every sample,
    including the ones where the drone is sitting on the ground at the start
    or end -- except that it cannot slide along the ground (z == 0).
    """
    rng = random.Random(seed)
    dt = 1.0 / rate_hz
    dt_ns = int(round(1e9 / rate_hz))
    # Ground-truth pose in the contract's ENU frame. Yaw is kept unwrapped
    # here (wrapped only when written) so long spins stay exact.
    pose = Pose(0.0, 0.0, 0.0, 0.0)
    tick = 0
    rows: List[dict] = []

    def emit(step: int, direction: Direction, v: Tuple[float, float, float, float]) -> None:
        n = (lambda: rng.gauss(0.0, noise_m)) if noise_m > 0 else (lambda: 0.0)
        rows.append({
            "t_s": round(tick * dt, 3), "timestamp_ns": tick * dt_ns,
            "step": step, "direction": direction.value,
            "x_m": round(pose.x + n(), 4) + 0.0, "y_m": round(pose.y + n(), 4) + 0.0,
            "z_m": round(max(0.0, pose.z + n()), 4), "yaw_deg": round(pose.yaw_deg % 360.0, 2),
            "vx_mps": round(v[0], 4) + 0.0, "vy_mps": round(v[1], 4) + 0.0,
            "vz_mps": round(v[2], 4), "yaw_rate_dps": round(v[3], 2),
        })

    for step, (direction, seconds) in enumerate(moves):
        ticks = int(round(seconds * rate_hz))
        for _ in range(ticks):
            v = _velocity(direction, pose.yaw_deg)
            if pose.z > 0.0:   # wind only moves the drone once it is off the ground
                v = (v[0] + drift_mps[0], v[1] + drift_mps[1], v[2], v[3])
            emit(step, direction, v)
            pose = Pose(
                pose.x + v[0] * dt,
                pose.y + v[1] * dt,
                max(0.0, pose.z + v[2] * dt),   # can't go through the ground
                pose.yaw_deg + v[3] * dt,
            )
            tick += 1
    # Final resting sample so the last position is in the file.
    emit(len(moves), D.HOVER, (0.0, 0.0, 0.0, 0.0))
    return rows


def metadata(name: str, moves: List[Tuple[Direction, float]], rate_hz: float,
             noise_m: float, seed: int, drift_mps: Tuple[float, float]) -> Dict[str, object]:
    """The `# key=value` lines written above the CSV header."""
    return {
        "schema_version": 1,
        "scenario": name,
        "rate_hz": rate_hz,
        "dt_s": round(1.0 / rate_hz, 6),
        "noise_m": noise_m,
        "seed": seed,
        "drift_mps": f"{drift_mps[0]},{drift_mps[1]}",
        "moves": len(moves),
        "duration_s": round(sum(s for _, s in moves), 3),
        "frame": "local_enu",
        "clock": "simulation",
        "synthetic": "true",
        "generator": "motion_engine/synthetic/generate_motion_data.py",
    }


def write_csv(rows: List[dict], path: Path,
              meta: Optional[Dict[str, object]] = None) -> None:
    with path.open("w", newline="") as f:
        for key, value in (meta or {}).items():
            f.write(f"# {key}={value}\n")
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)


def load_csv(path: Path) -> Tuple[Dict[str, str], List[Dict[str, str]]]:
    """Read a CSV written by this script: returns (metadata, rows).

    Values come back as strings, like any `csv.DictReader`. Files written
    with --no-meta (or before the metadata header existed) just return an
    empty metadata dict.
    """
    meta: Dict[str, str] = {}
    with Path(path).open(newline="") as f:
        lines = f.read().splitlines()
    body = []
    for line in lines:
        if line.startswith("#") and not body:
            key, _, value = line[1:].strip().partition("=")
            meta[key] = value
        else:
            body.append(line)
    return meta, list(csv.DictReader(body))


def plot(all_rows: Dict[str, List[dict]], path: Path) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    names = list(all_rows)
    # squeeze=False keeps axes 2-D even when only one scenario is plotted.
    fig, axes = plt.subplots(len(names), 2, figsize=(11, 2.6 * len(names)), squeeze=False)
    for i, name in enumerate(names):
        r = all_rows[name]
        t = [row["t_s"] for row in r]
        a = axes[i][0]
        for key, label in (("x_m", "x (east)"), ("y_m", "y (north)"), ("z_m", "z (up)")):
            a.plot(t, [row[key] for row in r], label=label)
        a.set_title(f"{name}: position")
        a.set_ylabel("m")
        a.legend(fontsize=7, loc="upper right")
        b = axes[i][1]
        b.plot(t, [row["yaw_deg"] for row in r], color="tab:purple")
        b.set_title(f"{name}: heading")
        b.set_ylabel("yaw (deg)")
        b.set_ylim(-10, 370)
    for a in axes[-1]:
        a.set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, default=_MOTION_ENGINE_ROOT / "motion_logs")
    p.add_argument("--rate", type=float, default=10.0, help="samples per second (default 10)")
    p.add_argument("--noise", type=float, default=0.0,
                   help="std-dev of position noise in metres (default 0 = perfectly clean)")
    p.add_argument("--seed", type=int, default=0,
                   help="seeds the noise and the random_walk scenario (default 0)")
    p.add_argument("--no-meta", action="store_true",
                   help="omit the '# key=value' metadata lines at the top of each CSV")
    p.add_argument("scenarios", nargs="*", help=f"subset of: {', '.join(ALL_SCENARIOS)}")
    args = p.parse_args()

    names = args.scenarios or ALL_SCENARIOS
    unknown = [n for n in names if n not in ALL_SCENARIOS]
    if unknown:
        sys.exit(f"Unknown scenario(s): {unknown}. Choose from {ALL_SCENARIOS}")

    args.out.mkdir(parents=True, exist_ok=True)
    all_rows = {}
    for name in names:
        moves = scenario_moves(name, args.seed)
        drift = DRIFT_MPS.get(name, (0.0, 0.0))
        rows = generate(moves, args.rate, args.noise, args.seed, drift)
        meta = None if args.no_meta else metadata(name, moves, args.rate, args.noise, args.seed, drift)
        write_csv(rows, args.out / f"{name}.csv", meta)
        all_rows[name] = rows
        end = rows[-1]
        print(f"{name:17s} {len(rows):4d} rows, {end['t_s']:5.1f} s, ends at "
              f"x={end['x_m']:+.2f} y={end['y_m']:+.2f} z={end['z_m']:+.2f} yaw={end['yaw_deg']:.0f}")
    if plot(all_rows, args.out / "overview.png"):
        print(f"plot -> {args.out / 'overview.png'}")


if __name__ == "__main__":
    main()
