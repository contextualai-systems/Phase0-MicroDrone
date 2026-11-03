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

## Files

`config.py` (all threshold numbers) · `safety_policy.py` (situation check) ·
`command_check.py` (order check) · `safety_decision.py` (original placeholder)


```
python -m unittest tests.test_safety_policy tests.test_command_check tests.test_safety_decision -v
```

32 tests. Simulation time is passed in as `now_ns`; nothing here reads the real
clock.

## Safety owns

Battery thresholds, freshness, timeouts, and movement limits. Docking asks
whether to return; Safety decides whether returning is permitted.

## Not done yet

- **Field names unconfirmed** — taken from Integration README Section 4; the
  shared contracts module doesn't exist. Wrong names mean `assess()` reads empty
  sections and blocks everything. Needs the state machine team to confirm.
- Launch and docking timeouts need coordinator-tracked time. (The 60 s return
  deadline is done.)
- Logging not wired up; weather strings are a guess; `Command` is provisional,
  since Motion already has `MotionCommand` with different fields.
