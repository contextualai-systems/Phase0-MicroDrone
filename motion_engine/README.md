# Motion Engine Module — Phase‑0 MicroDrone

## Purpose
The Motion Engine module defines how the drone *would* move — without actually controlling hardware in Phase‑0.  
This semester focuses on **motor command stubs**, **PID structure**, and **test harnesses**.

## Responsibilities (Phase‑0)
- Provide motor command stubs (no real ESC output)
- Define PID controller structure
- Log motor commands for debugging
- Provide a test harness for simulated movement

---

## Comms Architecture — Dual Link (MAVLink + uXRCE-DDS)

The drone runs **two independent links** between the PX4 flight controller and the
companion computer. Each carries a different class of traffic.

| Link | Owns | Why this link |
|---|---|---|
| **MAVLink** | Golf cart ↔ drone communication, motion commands, operator/GCS traffic | Radio-native, multi-node, lossy-tolerant. The only protocol QGroundControl speaks, and the only one that reaches an off-board vehicle over a telemetry radio |
| **uXRCE-DDS** | Image processing: CV pose/detections in, state estimate out | A near-direct mirror of PX4's internal uORB bus into ROS 2 topics. No translation layer, minimum latency on the highest-rate stream in the system |

### Our split

**MAVLink — golf cart, motion, and operator**
- Golf cart ↔ drone link: the cart is the mobile ground station. Commands out, telemetry back
- Motion commands: waypoints, goto targets, takeoff/land/RTL
- Arm / disarm, flight-mode changes, kill — the safety path
- Parameter get/set, sensor calibration, airframe config
- Low-rate telemetry: attitude, GPS, battery, RC, status text
- ULog download (on the ground — it saturates the radio)

**uXRCE-DDS — image processing**
- `/fmu/in/vehicle_visual_odometry` — pose from the CV subteam's vision node, 30–50 Hz
- `/fmu/out/vehicle_odometry` — fused EKF2 state estimate, back to `navigation/` and the CV node
- `/fmu/out/vehicle_local_position` — local NED position for the control loop
- `/fmu/out/vehicle_status` — arming state, nav state, failsafe flags
- `/fmu/out/timesync_status` — FC↔companion clock offset; watch it when debugging vision latency

### Why both, instead of picking one

Dropping either forces a real compromise:

- **XRCE-only** loses QGroundControl and the golf cart link. XRCE is a
  point-to-point bridge to one companion computer, not a radio protocol — it
  has no answer for talking to a separate ground vehicle. You'd also lose
  parameter config, calibration, and field monitoring without building custom
  ROS 2 UI tools.
- **MAVLink-only** forces vision data through a MAVROS translation layer
  (`VISION_POSITION_ESTIMATE`). PX4 is built around an internal bus called uORB;
  uXRCE-DDS mirrors it almost directly, while MAVLink packs and unpacks a generic
  packet on both ends. That's fine for "go to waypoint X" and wrong for 50 Hz
  camera-derived pose.

### The dividing rule
> **Off-vehicle or human-initiated → MAVLink.**  
> **Vision pipeline on the companion computer → uXRCE-DDS.**

> ⚠️ **One caveat on putting motion on MAVLink.** Waypoint- and goto-style commands
> are a good fit. If Phase‑2+ moves to a true *offboard* control loop streaming
> setpoints at 20–50 Hz, that loop should move to
> `/fmu/in/trajectory_setpoint` + `/fmu/in/offboard_control_mode` over uXRCE-DDS —
> MAVLink adds parse latency at that rate. The golf cart link stays MAVLink either
> way. Revisit this when the control loop stops being stubs.

---

## Physical Wiring

```
        Golf cart (mobile GCS)
              ▲
              │ telemetry radio, MAVLink
              ▼
  Flight Controller (PX4, e.g. Pixhawk)
  ┌──────────────────────────────────┐
  │  TELEM1 ──── MAVLink 57600 ───────┼──► radio ──► golf cart / QGroundControl
  │  TELEM2 ──── uXRCE-DDS 921600 ────┼──┐
  └──────────────────────────────────┘  │
                                        ▼
  Companion Computer (4 GB RAM)
  ┌──────────────────────────────────────────────┐
  │  MicroXRCEAgent  ──► ROS 2 /fmu/* topics      │
  │       ▲                    ▲                 │
  │  cv/ vision node     navigation/ + motion    │
  └──────────────────────────────────────────────┘
```

If the airframe has Ethernet (Pixhawk 6X, CUAV X7+), put uXRCE-DDS on Ethernet
(UDP) and leave both UARTs free.

---

## Configuration

### PX4 — MAVLink on TELEM1
```
MAV_0_CONFIG  = 101        # TELEM 1
MAV_0_MODE    = 0          # Normal
MAV_0_RATE    = 1200       # B/s. Keep low — a 57.6 kbps radio saturates fast
SER_TEL1_BAUD = 57600
```

### PX4 — uXRCE-DDS on TELEM2
```
UXRCE_DDS_CFG    = 102     # TELEM 2   (or 1000 for Ethernet)
SER_TEL2_BAUD    = 921600
UXRCE_DDS_DOM_ID = 0       # must match ROS_DOMAIN_ID on the companion
UXRCE_DDS_KEY    = 1       # unique per vehicle
```

### PX4 — EKF2, to actually consume the vision pose
Setting the topic up is not enough; EKF2 ignores external vision until told to fuse it.
```
EKF2_EV_CTRL  = 15         # bitmask: horiz. pos + vert. pos + velocity + yaw
EKF2_HGT_REF  = 3          # Vision as primary height reference (indoor / no GPS)
EKF2_EV_DELAY = <measured> # ms, camera-to-FC latency. Measure it, don't guess
```
> `EKF2_EV_CTRL` replaced the old `EKF2_AID_MASK` in PX4 v1.14 — confirm against
> the version pinned in `external/PX4-Autopilot`. An unmeasured `EKF2_EV_DELAY`
> is the most common cause of a vision-fused estimate that oscillates or drifts.

### Companion — XRCE agent
```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600   # TELEM2
MicroXRCEAgent udp4 -p 8888                          # Ethernet / SITL
```
ROS 2 nodes need `px4_msgs` built from the **same PX4 version** as the firmware.
Mismatched definitions fail silently — topics appear but fields are garbage.

### Companion — MAVLink router (optional)
Only for GCS access over Wi-Fi on the bench. Keep a **direct** FC→radio path for
flight: routing the safety path through the companion computer reintroduces the
dependency the split exists to avoid.
```ini
# /etc/mavlink-router/main.conf
[UartEndpoint fc]
Device = /dev/ttyAMA0
Baud = 57600

[UdpEndpoint gcs]
Mode = Normal
Address = <golf cart IP>
Port = 14550
```

### Verify
```bash
ros2 topic list | grep fmu                # /fmu/out/* within a few seconds
ros2 topic hz /fmu/out/vehicle_odometry   # expect your configured rate
# QGroundControl: heartbeat within ~5 s, params load, no STATUSTEXT errors
```

---

## Gotchas

- **QoS mismatch silently drops everything.** PX4 publishes `/fmu/out/*` as
  `BEST_EFFORT` + `KEEP_LAST(5)`. A subscriber defaulting to `RELIABLE` connects
  to nothing and reports no error. Set the profile explicitly.
- **Domain ID must match.** `UXRCE_DDS_DOM_ID` on the FC vs `ROS_DOMAIN_ID` in the
  companion's shell. A mismatch looks exactly like a dead cable.
- **Baud matters.** At 115200 the odometry stream will not sustain 50 Hz. Use
  921600 or Ethernet.
- **`MAV_0_RATE` is bytes/second, not a percentage.** Too high on a 57.6 kbps
  radio drops heartbeats, which looks like a hardware fault.
- **TELEM2 is spoken for.** Don't assign a MAVLink instance to it.
- **Don't subscribe to `/fmu/out/sensor_combined` in flight code.** Highest-rate
  topic on the bus; it will eat the CPU budget for no benefit.

### RAM budget (4 GB companion)
| Process | Rough footprint |
|---|---|
| ROS 2 middleware + XRCE agent | ~300–400 MB |
| CV node (tracking) | 400 MB – 1.5 GB depending on model |
| motion_engine loop | < 100 MB |
| mavlink-router | < 20 MB |

Two protocols cost far less than the headroom a vision model needs — the split is
not where the memory goes. If it gets tight, cap the XRCE publication rate before
touching the MAVLink side.

---

## Inputs
- Desired motion command (from Navigation, and from the golf cart over MAVLink)
- State vector (from Navigation, via `/fmu/out/vehicle_odometry` over uXRCE-DDS)
- Safety constraints (from Safety Layer)

## Outputs
- Motor command stub (placeholder values, Phase‑0)
- Motion commands to PX4 over MAVLink (Phase‑1+)
- Logged motor activity

## File Structure
- `run_motion.py` — main entry point
- `pid/` — PID controller structure
- `commands/` — motor command stubs
- `utils/` — logging + test harness
- `sitl/` — PX4 SITL + Gazebo bridge: boots a real simulated drone in WSL and
  flies a hover test over MAVLink/MAVSDK. Opt-in only — see `sitl/README.md`
  for the architecture and `RUN_SITL_TESTS=1 pytest tests/test_sitl_hover.py`
  to run it. The stub-based commands above and this live-sim path are
  independent; nothing in `commands/`/`pid/` depends on `sitl/`.

## Phase‑0 Scope
Phase‑0 is **SITL only**. In `sitl/`, PX4's Onboard MAVLink instance streams to
UDP 14540 and MAVSDK connects to it; the XRCE side runs as
`MicroXRCEAgent udp4 -p 8888` against the same simulated vehicle. No physical
UARTs and no golf cart yet — the wiring above is the target architecture the
SITL work stands in for.

## Open Decisions
- **Companion computer** not yet selected (Raspberry Pi 5 / Jetson Orin Nano /
  other). Decides whether TELEM2 attaches via native UART or a USB-to-UART
  bridge, and whether Ethernet XRCE is available at all.
- **Golf cart side**: what runs there — full QGroundControl, or a custom
  MAVLink node? Determines whether we need a second MAVLink instance.
- CV output format: full VIO pose vs. bearing-only target track.

## Good First Issues
- Add PID parameter placeholders
- Create a motor command logging function
- Add a simple test harness for simulated movement
- Write a health check that reports both links up/down in one command
- Measure and document `EKF2_EV_DELAY` for our camera + FC pair

## Future Phases
- Phase‑1: Real ESC control; real UART bring-up of both links; golf cart radio link
- Phase‑2: Multi‑axis stabilization; decide whether the control loop moves to
  offboard setpoints over uXRCE-DDS (see caveat above)
- Phase‑3: Full motion control loop; link-loss failsafe (behavior when XRCE drops
  but MAVLink survives)
