# Safety Layer — Phase 0

[Phase 0 scope: Start Here](../START_HERE.md) ·
[Integration contract](../INTEGRATION_README.md)

## What this does

Safety is the referee. Each tick it says what the drone is allowed to do.
Nothing moves unless Safety permits it.

Safety does **not** move the drone, choose mission states, or decide when to
come home. It grants or refuses permission, and always says why.

Phase 0 is fully synthetic — no drone, no PX4, no hardware. Passing tests show
the software behaves as specified, not that a real drone would be safe.

## The two checks

| Function | Question it answers |
|---|---|
| `assess(snapshot, now_ns)` | Is `launch` / `tracking` / `approach` / `descent` / `hold` allowed right now? |
| `check_command(command, assessment, now_ns)` | Is this specific order safe to send? |

An unsafe order gets a **replacement** (hold, or emergency land), never a
silent refusal — withholding a command would leave the drone flying toward its
old target. These run at steps 3 and 7 of each tick.

## How decisions are made

First level that applies wins:

1. **Emergency** — airborne with critical battery, unknown position/battery,
   lost signal, bad weather, or a trip home that has run 60 s or more
   (`context.return_started_ns`, kept through retries) → land in place,
   replace any order.
2. **Abort** — operator stop or rejected command → cancel target; hold still allowed.
3. **Restrictions** — block only what the hazard affects:

   | Hazard | Blocks |
   |---|---|
   | Low battery (≤20%) | launch, tracking |
   | People nearby / cart moving | launch, tracking, approach, descent |
   | Camera failure | launch, tracking |
   | Missing pad data | approach, descent |
   | Bad alignment | descent |
   | Unknown position | everything |

4. **Proceed** — permit the rest.

**Anything unknown counts as unsafe.** Missing, invalid, or stale (>0.5 s) data
blocks whatever depends on it. A grounded emergency inhibits launch rather than
landing; holding survives an abort, because holding is how the drone stops.

### Launch checks

Launch is only checked on the ground; in the air it is always blocked, with
no reason added (Docking copies Safety's reasons, so launch-only ones stay
out). On the ground, takeoff needs all of these (Integration README Section 6):

| Needs | Reason if not |
|---|---|
| Grounded (`vehicle.grounded` true) | `not_grounded` |
| Motors off (`vehicle.armed` false) | `already_armed` |
| Within 0.25 m of `dock.pad_pose` | `not_at_pad` |
| A fresh, valid bird sample | `missing_bird` |
| The bird detected | `no_bird` |
| Signal OK | `comm_loss` |
| Safe weather | `weather_hazard` |
| Battery at least 40%, plus everything in the Restrictions table | (see above) |

Lost signal and bad weather are emergencies in the air, but on the ground
they only stop takeoff; nothing is landed. Bird tracking geometry is in the
contract but not yet in the snapshot, so it is not checked.

## Files

`config.py` (all threshold numbers) · `safety_policy.py` (situation check) ·
`command_check.py` (order check) · `contracts.py` (`SafetyEvent`, the
report format)

```
python -m unittest tests.test_safety_policy tests.test_command_check tests.test_safety_event -v
```

Simulation time is passed in as `now_ns`; nothing here reads the real clock.

## Safety owns

Battery thresholds, freshness, timeouts, and movement limits. Docking asks
whether to return; Safety decides whether returning is permitted.

## SafetyEvent

`SafetyEvent` ([`contracts.py`](contracts.py)) is Safety's report for one tick:
what the drone may do, what Safety did with each order, and why. It is one of
the six shared formats for Phase 0, next to Motion's `Command` and
`VehicleFeedback`, CV's `DetectionResult` and the state machine's
`MissionStatus`. It records decisions; it never makes them.

This is a draft. Field names need team agreement.

### Purpose

Give every module, the shared log and the tests Safety's decisions in one
fixed format, so nobody re-derives safety rules or invents their own record,
and so a run can be reconstructed later from the log.

### Publisher and subscribers

- **Publisher:** Safety. The coordinator (`run_loop.py`) builds it each tick
  with `SafetyEvent.from_checks(assessment, [(order, decision), ...])` after
  calling `assess()` and `check_command()`.
- **Subscribers:** the shared log (`universal_log`), the state machine and
  Docking (which already read the same permissions from `SafetyAssessment`),
  and tests.

### Fields

| Field | Type | Meaning |
|---|---|---|
| `timestamp_ns` | int | Simulation time of this tick |
| `permissions` | dict | `launch`, `tracking`, `approach`, `descent`, `hold`: each true or false |
| `emergency` | bool | Land in place now |
| `abort` | bool | Cancel the current action and hold (disarm if grounded) |
| `battery_low` | bool | 20% or less |
| `battery_critical` | bool | 10% or less |
| `battery_recovered` | bool | 40% or more, enough to launch again |
| `reasons` | list of str | Every active reason code, most serious first (e.g. `critical_battery`, `cart_moving`) |
| `commands` | list | One entry per order checked this tick (empty if none) |
| `schema_version` | int | Always 1 |
| `clock` | str | Always `"simulation"` |
| `synthetic` | bool | Always true |
| `source` | str | Always `"safety"` |

Each `commands` entry:

| Field | Type | Meaning |
|---|---|---|
| `requested_id` / `requested_kind` | str | The order that was asked for, e.g. `TRACK_TARGET` |
| `allowed` | bool | True if it was sent unchanged |
| `sent_id` / `sent_kind` | str | What actually went to Motion, e.g. `HOVER` if replaced |
| `reasons` | list of str | Why it was replaced; empty if allowed |

### Units

All times are integer nanoseconds on the simulation clock. Order kinds are
Motion's uppercase names (`HOVER`). Reasons are lowercase codes.

### Update rate

10 Hz: one event per 0.1 s tick. A launch tick has two `commands` (ARM and
TAKEOFF); a tick with nothing to send has none.

### Validation rules

`SafetyEvent.validate()` returns a problem code, or `None` if the event is
fine. `tests/test_safety_event.py` checks every tick of every demo against
these rules.

- `permissions` has exactly the five actions, each a real true/false.
- An emergency allows nothing, not even hold.
- `emergency` and `abort` are never both true (emergency outranks abort).
- `battery_critical` implies `battery_low`.
- An emergency or abort always has at least one reason.
- An allowed order is sent unchanged with no reasons; a replaced order has a
  different `sent_id` and at least one reason.

### Example payload

Tick 50 of `full_cycle` (5000 ms), the same tick as the `MissionStatus`
example: low battery and a moving cart, so only hold is allowed, and the
state machine's hover goes through.

```json
{
  "timestamp_ns": 5000000000,
  "permissions": {"launch": false, "tracking": false, "approach": false, "descent": false, "hold": true},
  "emergency": false,
  "abort": false,
  "battery_low": true,
  "battery_critical": false,
  "battery_recovered": false,
  "reasons": ["low_battery", "cart_moving"],
  "commands": [
    {"requested_id": "sm-1", "requested_kind": "HOVER", "allowed": true,
     "sent_id": "sm-1", "sent_kind": "HOVER", "reasons": []}
  ],
  "schema_version": 1,
  "clock": "simulation",
  "synthetic": true,
  "source": "safety"
}
```

### Open questions

- Should the coordinator log an event every tick, or only when something
  changes? Every tick is complete but long.
- In the `emergency` demo the state machine asks for `EMERGENCY_LAND`, and
  Safety "replaces" it with an identical `EMERGENCY_LAND` under a new name.
  Should Safety pass an `EMERGENCY_LAND` through unchanged during an emergency?

## Not done yet

- **Field names unconfirmed** — taken from Integration README Section 4; the
  shared contracts module doesn't exist. Wrong names mean `assess()` reads empty
  sections and blocks everything. Needs the state machine team to confirm.
- Launch and docking timeouts need coordinator-tracked time. (The 60 s return
  deadline is done.)
- Logging not wired up; weather strings are a guess; `Command` is provisional,
  since Motion already has `MotionCommand` with different fields.
