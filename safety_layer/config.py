"""All the safety numbers in one place.

These numbers come from the team's INTEGRATION_README (Section 5).
They are made-up numbers for the pretend simulation, not real drone limits.

Keeping them here means everyone uses the same numbers. If the team changes
a rule, we change it once here instead of hunting through the code.
"""

# --- Battery, in percent ---
BATTERY_LOW_PCT = 20.0        # 20% or less: too low to fly around, but can still land
BATTERY_CRITICAL_PCT = 10.0   # 10% or less: emergency, land right now
BATTERY_RECOVERED_PCT = 40.0  # need at least 40% before taking off again

# --- How old information can be, in seconds ---
MAX_OBSERVATION_AGE_S = 0.5   # older than half a second = too old to trust
COMMAND_VALIDITY_S = 0.2      # an order goes stale after this long
TIMESTEP_S = 0.1              # the loop runs 10 times per pretend second

# --- Speed and space limits ---
MAX_HORIZONTAL_SPEED_MPS = 5.0   # how fast it may fly sideways
MAX_VERTICAL_SPEED_MPS = 2.0     # how fast it may go up or down
MAX_YAW_RATE_DPS = 90.0          # how fast it may spin
GEOFENCE_RADIUS_M = 150.0        # invisible fence: stay within 150m of the pad
ALTITUDE_CEILING_M = 30.0        # don't go higher than this
GROUNDED_THRESHOLD_M = 0.05      # below this height, it counts as on the ground

# --- Weather we're willing to fly in ---
# TODO: ask the team exactly which weather words the simulation will send.
SAFE_WEATHER = ("clear", "cloudy")

# Timestamps are in nanoseconds, so this helps convert to seconds.
NS_PER_S = 1_000_000_000

# Docking / return defaults shared with INTEGRATION_README Sections 5 and 8.
DOCK_APPROACH_ALTITUDE_M = 5.0
POSITION_ARRIVAL_TOLERANCE_M = 0.25
DOCK_ALIGNMENT_TOLERANCE_M = 0.25
DOCK_YAW_TOLERANCE_DEG = 3.0
DOCK_ATTEMPT_TIMEOUT_NS = 30 * NS_PER_S
DOCK_SEARCH_TIMEOUT_NS = 5 * NS_PER_S
BIRD_ABSENCE_RETURN_NS = 3 * NS_PER_S
HOVER_RETURN_TIMEOUT_NS = 10 * NS_PER_S
