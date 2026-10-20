# Scripted demo scenarios for run_loop.py.
#
# Each scenario is a list of tuples, where each tuple contains:
# - The time step in milliseconds.
# - The changes to apply to the world at that time (only the listed fields change).
#
# Scenarios script facts about the world (bird, battery, environment, dock), not Safety's or Docking's answers:
# Safety's assess() works out the permissions, and Docking works out when to return and how to land, every tick.
#
# Times must be multiples of the tick length (100 ms).

BIRD_SEEN = {"detected": True, "type": "hawk", "distance_m": 30.0}
BIRD_GONE = {"detected": False, "type": None, "distance_m": None}


# Full cycle: launch for a bird, lose it, return on low battery, wait for the cart, land, recharge.
# Run for 10000 ms.
# IDLE -> HOVERING -> TRACKING -> HOVERING -> DOCKING_INIT -> DOCKING_APPROACH -> DOCKED -> IDLE
DEMO_SCENARIO = [
    (1000, {"bird": {"detected": True, "type": "hawk", "distance_m": 30.0}}),
    (4000, {"bird": {"detected": False, "type": None, "distance_m": None}}),
    # Low battery and the cart starts moving: Safety allows only hold
    (5000, {"battery": {"percent": 18}, "environment": {"cart_moving": True}}),
    # The cart stops: Safety allows approach and descent again (low battery still blocks launch and tracking)
    (6000, {"environment": {"cart_moving": False}}),
    (9500, {"battery": {"percent": 90}}),
]


# Docking fails on the first attempt, the mission aborts, then it retries and lands.
# The drone can't line up with the pad: Docking searches above it, gives up after 5 s
# (alignment_search_timeout) and the mission aborts. On the retry the alignment comes back and it lands.
# Run for 15000 ms.
# IDLE -> HOVERING -> TRACKING -> HOVERING -> DOCKING_INIT -> DOCKING_APPROACH -> ABORT
#      -> DOCKING_INIT -> DOCKING_APPROACH -> DOCKED -> IDLE
DOCKING_RETRY_SCENARIO = [
    (500, {"bird": BIRD_SEEN}),
    # The bird leaves (Docking asks to return 3 s later) and the pad alignment is lost
    (3000, {"bird": BIRD_GONE, "dock": {"alignment_valid": False}}),
    # The alignment comes back during the second attempt
    (12000, {"dock": {"alignment_valid": True}}),
]


# The battery drops to 8% while tracking: Safety raises an emergency (critical battery),
# so the drone lands in place, disarms, and stays in EMERGENCY_LAND.
# Run for 9000 ms.
# IDLE -> HOVERING -> TRACKING -> EMERGENCY_LAND
EMERGENCY_SCENARIO = [
    (500, {"bird": BIRD_SEEN}),
    (3000, {"battery": {"percent": 8}}),
]


# The cart keeps moving, so Safety never allows the approach: the drone holds in DOCKING_INIT.
# Run for 5500 ms.
# IDLE -> HOVERING -> TRACKING -> DOCKING_INIT
BLOCKED_DOCK_SCENARIO = [
    (500, {"bird": BIRD_SEEN}),
    # The moving cart blocks tracking, so Docking asks to return straight away (no 3 s wait for the bird)
    (3000, {"bird": BIRD_GONE, "environment": {"cart_moving": True}}),
]


# The drone reaches the dock before it is lined up with the pad. Docking hovers above the pad
# (search) until the alignment is valid at 8000 ms, then aligns and descends.
# Run for 11000 ms.
# IDLE -> HOVERING -> TRACKING -> HOVERING -> DOCKING_INIT -> DOCKING_APPROACH -> DOCKED -> IDLE
NOT_ALIGNED_SCENARIO = [
    (500, {"bird": BIRD_SEEN}),
    # The bird leaves (Docking asks to return 3 s later) and the pad alignment is lost
    (3000, {"bird": BIRD_GONE, "dock": {"alignment_valid": False}}),
    # Within Docking's 5 s search time, so it doesn't give up
    (8000, {"dock": {"alignment_valid": True}}),
]


# The cart starts moving while the drone is descending. Safety withdraws the approach, Docking aborts
# the attempt, and the mission waits in DOCKING_INIT until the cart stops. Docking then climbs back
# to the approach height and lands.
# Run for 12000 ms.
# IDLE -> HOVERING -> TRACKING -> HOVERING -> DOCKING_INIT -> DOCKING_APPROACH -> ABORT
#      -> DOCKING_INIT -> DOCKING_APPROACH -> DOCKED -> IDLE
CART_DURING_DESCENT_SCENARIO = [
    (500, {"bird": BIRD_SEEN}),
    # The bird leaves; Docking asks to return 3 s later
    (3000, {"bird": BIRD_GONE}),
    # The descent runs from 6400 ms
    (7000, {"environment": {"cart_moving": True}}),
    (8000, {"environment": {"cart_moving": False}}),
]


# Communication is lost while tracking: Safety raises an emergency (comm loss),
# so the drone lands in place, disarms, and stays in EMERGENCY_LAND.
# Run for 9000 ms.
# IDLE -> HOVERING -> TRACKING -> EMERGENCY_LAND
COMM_LOSS_SCENARIO = [
    (500, {"bird": BIRD_SEEN}),
    (3000, {"environment": {"comm_ok": False}}),
]


# Demo name -> (scenario, how long to run it in ms).
# Used by: python3 -m state_machine.run_loop <name>
DEMOS = {
    "full_cycle": (DEMO_SCENARIO, 10000),
    "docking_retry": (DOCKING_RETRY_SCENARIO, 15000),
    "emergency": (EMERGENCY_SCENARIO, 9000),
    "blocked_dock": (BLOCKED_DOCK_SCENARIO, 5500),
    "not_aligned": (NOT_ALIGNED_SCENARIO, 11000),
    "cart_during_descent": (CART_DURING_DESCENT_SCENARIO, 12000),
    "comm_loss": (COMM_LOSS_SCENARIO, 9000),
}
