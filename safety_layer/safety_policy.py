"""The safety referee.

Every tick (10 times per pretend second), this looks at what's happening
right now and says which actions the drone is allowed to do.

It checks problems in order of how bad they are. The first matching level
decides the response:

    1. Emergency    -> something is really wrong, land immediately
    2. Abort        -> cancel what we're doing and stop in place
    3. Restrictions -> only block the actions the problem actually affects
    4. Proceed      -> everything else is fine, go ahead

Safety does NOT move the drone and does NOT decide what the drone should do
next. It only says what is allowed. Other people's code decides the rest.

The current time is passed in, not looked up, so tests always get the same
answers no matter how fast the computer runs.
"""

from dataclasses import dataclass, field
from typing import Dict, Tuple

from safety_layer import config


# The five things the drone might want to do, which we answer yes/no for.
ACTIONS = ("launch", "tracking", "approach", "descent", "hold")


@dataclass(frozen=True)
class SafetyAssessment:
    """Safety's answer for one moment in time.

    "frozen" means nobody can change the answer after it's made,
    so the log always shows what safety really said.
    """

    permissions: Dict[str, bool]     # for each action: True = allowed, False = blocked
    emergency: bool = False          # True = land right now
    abort: bool = False              # True = cancel what we're doing and stop
    battery_low: bool = False        # battery is getting low
    battery_critical: bool = False   # battery is nearly dead
    battery_recovered: bool = False  # battery is charged enough to take off
    grounded: bool = False           # True = the drone is on the ground
    disarm: bool = False             # True = shut the motors off where it sits
    reasons: Tuple[str, ...] = field(default_factory=tuple)  # short words explaining why
    timestamp_ns: int = 0            # what time this answer was given

    def allows(self, action: str) -> bool:
        """Is this action allowed? Unknown actions are treated as a no."""
        return self.permissions.get(action, False)


def _fresh(section, now_ns) -> bool:
    """Can we trust this piece of information?

    It has to be there, marked as good, and recent. Old information is
    dangerous: the drone could have moved since then.
    """
    # Is it missing, or marked as broken?
    if not isinstance(section, dict) or not section.get("valid"):
        return False

    stamp = section.get("timestamp_ns")

    # No time on it, or a time in the future? Something is wrong with it.
    if not isinstance(stamp, int) or stamp > now_ns:
        return False

    # Finally: is it recent enough?
    age_ns = now_ns - stamp
    return age_ns <= config.MAX_OBSERVATION_AGE_S * config.NS_PER_S


def assess(snapshot: dict, now_ns: int) -> SafetyAssessment:
    """Look at what's happening and decide what the drone may do.

    snapshot: everything we know right now, in the team's shared format
    now_ns:   the current pretend time

    Big rule: if we don't know something, we say no. We never guess that
    things are fine.
    """
    reasons = []      # short words explaining our answer
    blocked = set()   # actions we are NOT allowing
    emergency = False
    abort = False
    disarm = False

    # Pull out the parts we care about. If a part is missing we use an
    # empty box instead, so the code below doesn't crash.
    vehicle = snapshot.get("vehicle") or {}          # where the drone is
    battery = snapshot.get("battery") or {}          # how much charge is left
    environment = snapshot.get("environment") or {}  # people, cart, weather, camera, signal
    dock = snapshot.get("dock") or {}                # the landing pad
    context = snapshot.get("context") or {}          # what the drone is up to

    # Can we trust each piece?
    vehicle_ok = _fresh(vehicle, now_ns)
    battery_ok = _fresh(battery, now_ns)
    environment_ok = _fresh(environment, now_ns)

    # Is the drone currently in the air? Rules are different on the ground.
    # If we're not told, assume it's flying: treating a flying drone as
    # grounded would send "disarm" instead of "land", which Motion refuses
    # in the air, so the drone would keep going.
    flight_active = (
        context.get("flight_active") is not False
        or vehicle.get("airborne") is True
    )

    # Work out the battery situation once, so we don't repeat ourselves.
    percent = battery.get("percent")
    battery_known = battery_ok and isinstance(percent, (int, float))
    battery_critical = battery_known and percent <= config.BATTERY_CRITICAL_PCT
    battery_low = battery_known and percent <= config.BATTERY_LOW_PCT
    battery_recovered = battery_known and percent >= config.BATTERY_RECOVERED_PCT

    # ---------- LEVEL 1: EMERGENCY ----------
    # The same five conditions matter in the air and on the ground, but the
    # response is different, so we work out which ones apply first.
    serious = []
    if battery_critical:
        serious.append("critical_battery")
    if not vehicle_ok:
        # We don't know where it is anymore.
        serious.append("stale_vehicle")
    if not battery_ok:
        # We don't know how much charge is left.
        serious.append("stale_battery")
    if environment.get("comm_ok") is not True:
        # Nobody can send it orders anymore.
        serious.append("comm_loss")
    if environment_ok and environment.get("weather") not in config.SAFE_WEATHER:
        serious.append("weather_hazard")
    # TODO: also land if the 60-second trip home runs out of time.
    #       Waiting on the coordinator to track that timer.

    if serious:
        reasons.extend(serious)
        if flight_active:
            # In the air: land immediately.
            emergency = True
        else:
            # On the ground: there is nothing to land, so shut the motors
            # off and don't take off. The integration plan is explicit that
            # a ground emergency must not start a landing manoeuvre.
            disarm = True
            blocked.add("launch")

    # ---------- LEVEL 2: ABORT ----------
    # Less serious than an emergency, so we skip this if it's already one.
    if not emergency:
        if context.get("operator_abort"):
            # A person pressed the stop button.
            abort = True
            reasons.append("operator_abort")
        if context.get("pending_command_failure"):
            # The last order didn't go through, so stop and rethink.
            abort = True
            reasons.append("command_rejected")
        # TODO: also abort on launch and docking timeouts, once we have timers.

        # An abort on the ground means shut the motors off, not hover.
        if abort and not flight_active:
            disarm = True

    # ---------- LEVEL 3: RESTRICTIONS ----------
    # Here we block only the actions each problem actually affects,
    # instead of shutting everything down.

    if battery_low:
        # Not enough charge to fly around, but enough to come home and land.
        blocked.update(("launch", "tracking"))
        reasons.append("low_battery")

    if not environment_ok:
        # We can't see what's going on around the drone, so no flying.
        blocked.update(("launch", "tracking", "approach", "descent"))
        reasons.append("missing_environment")
    else:
        # Note: "is not False" means anything other than a clear "no" counts
        # as a problem. A missing or misspelled field blocks the action
        # instead of quietly slipping through.
        if environment.get("people_nearby") is not False:
            blocked.update(("launch", "tracking", "approach", "descent"))
            reasons.append("people_nearby")

        if environment.get("cart_moving") is not False:
            # Can't land on a moving cart.
            blocked.update(("launch", "tracking", "approach", "descent"))
            reasons.append("cart_moving")

        if environment.get("camera_ok") is not True:
            # Can't chase a bird it can't see, but it can still land,
            # because landing uses the pad sensors instead.
            blocked.update(("launch", "tracking"))
            reasons.append("camera_failure")

    if not _fresh(dock, now_ns):
        # We don't know where the landing pad is.
        blocked.update(("approach", "descent"))
        reasons.append("missing_pad")
    elif dock.get("alignment_valid") is not True:
        # We know where the pad is, but the drone isn't lined up with it,
        # so it can fly over but must not come down.
        blocked.add("descent")
        reasons.append("invalid_alignment")

    if not vehicle_ok:
        # No idea where the drone is: nothing is safe.
        blocked.update(ACTIONS)
        reasons.append("stale_vehicle")

    if not battery_ok:
        blocked.update(("launch", "tracking", "approach", "descent"))
        reasons.append("stale_battery")

    # Taking off needs a well-charged battery, not just a not-low one.
    if not battery_recovered:
        blocked.add("launch")

    # An emergency stops everything, including holding still.
    # An abort stops flying around, but holding still is still allowed,
    # because the drone has to stop somehow.
    if emergency:
        blocked.update(ACTIONS)
    elif abort:
        blocked.update(("launch", "tracking", "approach", "descent"))

    # ---------- LEVEL 4: PROCEED ----------
    # Anything we didn't block is allowed.
    permissions = {action: action not in blocked for action in ACTIONS}

    return SafetyAssessment(
        permissions=permissions,
        emergency=emergency,
        abort=abort,
        battery_low=battery_low,
        battery_critical=battery_critical,
        battery_recovered=battery_recovered,
        grounded=not flight_active,
        disarm=disarm,
        # dict.fromkeys keeps the order but removes repeats, so the same
        # reason doesn't show up twice.
        reasons=tuple(dict.fromkeys(reasons)),
        timestamp_ns=now_ns,
    )


# STILL TO DO:
# - Send our answers to the team's shared log (universal_log).
