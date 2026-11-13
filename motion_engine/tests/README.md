# Motion Engine tests

Every test here runs on the simulation clock against the kinematic
`MotionStub`. There is no PX4, no hardware, and no wall-clock time, so the
suite is deterministic and finishes in about a second.

```bash
py -3 -m pytest motion_engine/tests                      # everything
py -3 -m pytest motion_engine/tests/test_waypoints.py -v # one file, verbose
py -3 -m pytest motion_engine/tests -k comm_loss         # by name
```

## How the tests are built

Most tests drive Motion the way the coordinator will (INTEGRATION_README §3):
**one `submit()` per command, one `step(0.1 s, now_ns)` per tick, then
`feedback()`**. Commands that need renewing are re-sent every tick with a
fresh 0.2 s expiry.

Most flights are recorded and passed through `trace_checker.check_trace()`
at the end (`rig.check()`). So on top of its own asserts, a test fails if the
flight ever exceeds a speed or yaw limit, leaves the geofence, moves while a
command is expired, reports a landing while airborne, or lets a rejected
command change what Motion is flying.

### Shared helpers (not test files)

| File | What it gives you |
|---|---|
| `conftest.py` | Puts the repo root on `sys.path` (so a single file can be run by path), and provides two fixtures: `rig` (disarmed on the pad, clock at 0) and `airborne_rig` (armed, holding exactly 5 m above the pad). |
| `motion_rig.py` | `Rig`, a stand-in for the coordinator's tick loop around a recorded `MotionStub`. `rig.cmd()` builds a contract `Command`, `rig.renewed(c)` re-issues it now, `rig.tick()` steps once, `rig.fly(c, until)` renews `c` every tick until a condition holds, `rig.launch(z)` arms and climbs to `z`, `rig.goto(pose)` flies there, and `rig.check()` runs the trace checker. It also has the shared constants (`DT`, `DT_NS`, `S`, `VALIDITY_NS`, `PAD`, `LIM`, `REPO_ROOT`, `MOTION_ENGINE_ROOT`) and `assessment(blocked=..., emergency=...)` for building Safety verdicts. |

## The test files

### `test_motion_stubs.py`: the stub's direct method API

Calls `arm()`, `move_to()`, `step()` and the other methods directly, the way
scripts and the synthetic-data generator use the stub. Uses its own float
`FakeClock`. **The direct API clamps an out-of-limits target onto the fence,
while the contract path (`submit()`) rejects it.** That difference is
intentional, and it is why geofence tests exist in both files.

| Group | Tests |
|---|---|
| Basics | `test_pose_helpers`: distance helpers. `test_initial_state`: disarmed at the origin with no target. `test_logger_none_is_fine`: runs with no logger. |
| Kinematics | `test_takeoff_uses_vertical_cap` (2 m/s), `test_move_to_converges_in_expected_time` (5 m/s, and snaps exactly onto the target), `test_diagonal_is_straight_and_capped`, `test_vertical_cap_is_independent` (horizontal and vertical are capped separately), `test_speed_override_only_lowers`, `test_yaw_takes_shortest_path` (4 wrap-around cases, 90 deg/s), `test_at_target_uses_arrival_tolerance`, `test_step_zero_or_no_target_is_noop`. |
| Geofence | `test_geofence_clamps_ceiling_floor_and_radius` (clamps and logs `clamped_to_geofence`), `test_geofence_is_centered_on_home`, `test_altitude_floor_blocks_flight_not_landing`, `test_unclamped_command_has_no_clamp_decision`. |
| Arming | `test_motion_commands_rejected_when_disarmed` (all 11 flight methods), `test_disarm_refused_while_airborne` (also: a plain `land()` does not auto-disarm), `test_history_and_active_command`. |
| Mission commands | `test_track_position_flies_the_given_pose`, `test_motion_has_no_bird_geometry_api` (bird geometry belongs to Navigation), `test_dock_approach_then_descend_reaches_dock`, `test_hover_and_stop_hold_current_pose`, `test_set_orientation_only_yaw_moves` (roll and pitch are not simulated). |
| Emergency | `test_emergency_land_descends_in_place_ignores_commands_and_auto_disarms`: 1 m/s straight down, latched, idempotent, auto-disarms on touchdown, can be re-armed afterwards. |
| Logging | `test_arrival_logged_exactly_once`, `test_step_does_not_log_every_tick`, `test_every_accepted_command_logs_one_motion_record`, `test_log_context_is_attached`. |
| Phase-1 hooks | `test_every_public_command_has_phase1_hook`: finds every public method in the source, so a new command without a `# PHASE-1 HOOK:` comment fails. `test_position_hooks_mention_ned_conversion`: each hook names its real MAVLink/XRCE message, and the position and velocity hooks mention the ENU to NED conversion. |

### `test_motion_contract.py`: the `submit()` / ack / feedback contract

The Motion Engine deliverables from INTEGRATION_README §3-§5 and §8, driven
through the contract the coordinator uses. Expiry under command loss and the
emergency latch are covered in every flight phase by
`test_failure_injection.py`, so they are not repeated here.

| Group | Tests |
|---|---|
| Ack vs progress | `test_accepted_means_executing_not_done`: an accepted ack means in progress, not finished. `test_launch_reports_arrival_and_airborne_and_refuses_disarm`. |
| Approach and landing | `test_approach_and_descent_land_settled_then_disarm`: "arrived" while still in the air must not read as landed. Ordinary landings do not auto-disarm. `test_flight_commands_rejected_when_not_armed`. |
| Replacement | `test_new_command_replaces_target_before_next_step`: a HOVER takes effect before the next step. `test_rejected_command_leaves_previous_active_and_reports_it`. |
| Limits | `test_geofence_and_ceiling_edges_reject_not_clamp` (6 cases exactly on and just past the ceiling and fence), `test_speed_limits_in_command_only_lower`. |
| Renewal and IDs | `test_renewal_extends_without_restart_or_extra_log`, `test_reusing_an_id_for_a_different_request_is_rejected`: a reused ID is rejected, including an old ID and a verbatim replay. |
| Malformed input | `test_malformed_or_stale_commands_are_rejected_with_reason`: 13 cases (missing expiry or target, NaN or inf, wrong frame, unsupported kind or schema, empty ID, negative cap, bad, future or expired timestamp), each with its exact `Reason`. `test_non_command_input_does_not_raise`. `test_kind_may_be_given_as_its_string_value`. |
| Feedback | `test_feedback_is_read_only_and_json_serialisable`, `test_identical_inputs_reproduce_identical_traces_and_logs`: two runs of the same script give byte-identical traces and logs. |

### `test_velocity_command.py`: `SET_VELOCITY`

A velocity setpoint has no target, so it must always carry an expiry (a
dead-man switch). It is rejected, never scaled down, when it is too fast or
non-finite, and it stops *at* a boundary with an explicit `boundary_hold`.

| Group | Tests |
|---|---|
| Happy path | `test_velocity_happy_path_moves_at_commanded_rate`, `test_velocity_without_yaw_rate_keeps_heading`, `test_position_command_replaces_velocity`. |
| Rejections | `test_non_finite_velocity_rejected` (5 cases), `test_over_speed_velocity_rejected_not_scaled` (6 cases, including the command's own lower caps), `test_velocity_exactly_at_limits_is_accepted`, `test_velocity_requires_expiry_and_velocity_and_no_target`, `test_velocity_rejected_on_ground_and_when_disarmed`. |
| Boundary hold | `test_ceiling_boundary_hold`: stops exactly where the ceiling was crossed, logged once, and the finished ID cannot be renewed. `test_geofence_boundary_hold`. `test_floor_boundary_hold_is_not_a_landing`. `test_velocity_toward_boundary_from_the_boundary_holds_without_moving`: moving away from the boundary is still allowed. |
| Emergency | `test_emergency_latch_wins_over_velocity`. |
| Direct API | `test_direct_set_velocity_refuses_instead_of_clamping`. |

Expiry of a velocity command is the `set_velocity` case in
`test_failure_injection.py` §A, and its comm-loss landing is in §J.

### `test_drone_state.py`: the `DroneState` feedback contract

Checks the fields START_HERE requires (position, velocity, orientation,
frame, timestamp, flight mode, validity), and that the old `VehicleFeedback`
layout was only ever *added to*.

- `test_stub_state_has_start_here_fields`, `test_velocity_is_estimated_from_motion`,
  `test_flight_mode_follows_active_command` (7 modes, from DISARMED to EMERGENCY_LAND).
- `test_to_dict_keeps_legacy_keys_and_only_adds`, `test_legacy_alias_and_keyword_construction_still_work`:
  `docking/run_docking.py` still builds feedback without the new fields, and
  `validate()` reports what is missing instead of supplying a default.
- `test_validate_rejects`: 22 broken states (bad schema or timestamp or frame,
  missing fields, non-finite values, airborne *and* grounded, settled in the
  air, airborne but disarmed, ...), each with its `Reason`.
  `test_validate_accepts_invalid_state_with_reason_and_never_raises`.
- `test_readme_example_round_trips_through_json`, `test_readme_example_matches_what_the_stub_publishes`:
  the JSON example in `motion_engine/README.md` is parsed and checked against real
  output, so the README cannot drift.

### `test_motion_command_contract_doc.py`: keeps the README's contract section accurate

Reads the "MotionCommand contract" section of `motion_engine/README.md`.

- `test_command_examples_round_trip`, `test_ack_examples_have_exactly_the_ack_fields`:
  each tagged JSON example (`<!-- motion-command-example: NAME -->`) has exactly the
  dataclass fields and round-trips.
- `test_examples_replay_through_motion_stub`: replays the examples through a stub
  and checks the acks match the README exactly.
- `test_reason_strings_match_code`, `test_kind_lists_match_code`: every `Reason` and
  command kind is documented, and the README names no reason that does not exist.

If you change a `Reason` or a payload field, update the README in the same PR.

### `test_failure_injection.py`: the Phase-0 "definition of done"

Failures are injected mid-flight to check that missing inputs, stale data,
command loss, geofence violations and low battery each have a defined, tested
outcome (START_HERE), and that a rejected or expired command can never leave
an unsafe target active (INTEGRATION_README §11). Every scenario ends with the
trace checker.

| Section | Tests |
|---|---|
| **A. Command loss** | `test_command_loss_freezes_the_vehicle_within_one_validity_window`: 6 kinds, including `set_velocity` and `descend_to_dock`. At most one more tick of motion, then `expired_hold` on the spot, and the expired ID cannot be revived. Also: `test_one_dropped_renewal_is_tolerated`, `test_after_expiry_the_stale_id_cannot_be_revived_but_a_fresh_id_resumes`, `test_lifecycle_commands_complete_without_any_renewal`, `test_emergency_descent_never_expires_or_escalates`. |
| **B. Loop stalls and clock** | `test_loop_stall_expires_the_target_before_moving` (3 cases, including a 5 s `dt`), `test_clock_running_backwards_does_not_revive_an_expired_command`. |
| **C. Late commands** | `test_late_expired_command_is_rejected_and_the_current_target_keeps_flying`. |
| **D. Geofence while tracking** | `test_bird_walking_out_of_the_geofence_never_drags_the_drone_out`, `test_safety_replaces_an_out_of_limits_target_and_the_drone_stops_this_tick`. |
| **E. Emergency in every phase** | `test_emergency_in_any_phase_descends_in_place_and_latches` (climb, tracking, approach, descent, hover): latch refuses everything else, a repeat is idempotent, 1 m/s straight down, auto-disarm. `test_emergency_while_armed_on_the_pad_disarms_without_flying`, `test_emergency_while_disarmed_is_rejected`. |
| **F. Hazards via the real Safety layer** | `test_cart_motion_during_descent_stops_descent_before_that_ticks_movement`, `test_hazard_with_hold_forbidden_lands_in_place_away_from_the_pad`, `test_low_battery_stops_tracking_but_still_allows_the_return_to_the_pad`, `test_critical_battery_mid_approach_overrides_docking_with_emergency_landing`. |
| **G. Garbage input** | `test_garbage_submitted_every_tick_does_not_change_the_flight`: 19 kinds of junk, submitted every tick, and the flown path is identical to a clean run. |
| **H. Arrival is not landing** | `test_descent_lost_inside_arrival_tolerance_never_reports_landed`. |
| **I. Bad `dt`** | `test_zero_negative_or_bad_dt_does_not_move_the_vehicle` (9 values: 0, negative, NaN, inf, None, a string, bool), `test_any_real_number_is_a_valid_dt` (int, Fraction), `test_bad_dt_tick_leaves_a_clean_trace_and_flight_resumes`, `test_bad_dt_does_not_keep_an_expired_target_alive`. |
| **J. Comm-loss escalation** | An expired hold that lasts 10 s turns into a straight-down landing. `test_escalation_lands_in_place_after_the_timeout`, `test_expired_velocity_command_escalates_to_comm_loss_landing`, `test_a_fresh_command_resets_the_timer`, `test_renewals_inside_the_validity_window_never_start_the_timer`, `test_fresh_valid_command_cancels_a_comm_loss_landing_but_a_stale_one_cannot`, `test_emergency_latch_still_dominates_a_comm_loss_landing`, and the "does not escalate" cases: settled on the pad, armed on the ground, disarmed, no sim time (`test_step_without_simulation_time_never_escalates`). `test_comm_loss_timeout_is_configurable_and_uses_simulation_time`. |

### `test_safety_adapter.py`: Motion and Safety together

Runs the real `safety_layer.command_check` and the real `MotionStub` through
`motion_engine.safety_adapter.check_for_motion()`. Safety rules on each
command, and Motion must accept whatever Safety approves *or substitutes*.

- **Kind coverage**: `test_safety_rules_on_every_kind_motion_accepts` (every contract
  kind has a Safety permission).
- **Tracking permission**: `test_tracking_kinds_fly_when_allowed_and_become_a_hover_when_blocked`
  (TRACK_TARGET, MOVE_TO, SET_YAW, SET_VELOCITY). `test_targetless_flight_kinds_are_allowed`.
- **SET_VELOCITY replacements**: `test_unmapped_set_velocity_is_refused_and_held` (fails closed
  if the mapping is ever removed), `test_emergency_replacement_of_a_velocity_command_is_accepted`,
  `test_grounded_emergency_disarm_of_a_velocity_command_is_accepted`,
  `test_only_a_copied_velocity_is_cleared`.
- **Replacements Motion must accept**: `test_blocked_tracking_becomes_a_hover_motion_accepts_and_stops`,
  `test_emergency_or_no_hold_permission_becomes_a_lifecycle_landing_motion_accepts`,
  `test_repeated_hold_replacement_is_a_renewal`, `test_target_outside_geofence_is_replaced_before_motion_sees_it`,
  `test_speed_cap_above_the_limit_is_replaced`.
- **Expiry**: `test_renewed_command_is_not_flagged_expired_by_safety`, `test_unrenewed_command_is_expired_and_held`.
- **Guard**: `test_non_motion_replacement_is_refused` (the adapter raises if Safety ever returns something that is not a `Command`).

### `test_limits_config.py`: Motion's limits equal Safety's config

INTEGRATION_README §4: Safety and Motion use the same configured limits.

- `test_default_limits_match_safety_config`: 8 shared fields, each equal to its
  `safety_layer.config` constant. `test_defaults_equal_from_config`.
- `test_unshared_fields_are_only_the_documented_ones`: a new `MotionLimits` field
  must either be mapped to config or be listed here.
- `test_comm_loss_landing_fits_inside_the_return_deadline`, `test_comm_loss_timeout_follows_config_when_defined`,
  `test_from_config_follows_a_changed_config`, `test_from_config_accepts_any_source`.
- `test_run_loop_builds_motion_from_safety_config`: the coordinator passes Motion the
  config limits, not the defaults.

### `test_waypoints.py`: `WaypointExecutor`

Every tick goes executor → Safety → `MotionStub.submit` → step → feedback →
`executor.update()`, using its own `WaypointRig` (named so it is not confused
with `motion_rig.Rig`).

- **Nominal**: `test_flies_every_waypoint_in_order_and_completes`,
  `test_every_command_is_a_move_to_and_ids_renew_per_waypoint`,
  `test_progress_distance_shrinks_toward_each_waypoint`.
- **Validation at load**: `test_bad_routes_fail_at_load_and_never_fly` (7 cases: empty route,
  NaN, inf, not a Pose, above the ceiling, outside the fence, below the floor),
  `test_route_limits_come_from_safety_config`, `test_route_id_is_required`.
- **Interruptions**: `test_motion_rejection_fails_the_route_with_motions_reason`,
  `test_safety_hold_does_not_count_as_arrival_and_route_resumes`,
  `test_expired_command_is_reissued_under_a_new_id`, `test_cancel_stops_commands_and_load_replaces_route`,
  `test_emergency_latch_rejects_waypoints`.
- **Faults mid-route** (3 s into the patrol, short of waypoint 0):
  - `test_emergency_from_safety_mid_route_lands_in_place_and_never_counts_a_waypoint`:
    straight down, no arrival counted, and the route fails (`not_armed`) instead of relaunching.
  - `test_emergency_sent_mid_route_fails_the_route_and_lands_in_place`: the coordinator's own
    EMERGENCY_LAND fails the route, and the executor stops sending.
  - `test_low_battery_mid_route_holds_and_the_return_goes_through_docking`: waypoint MOVE_TOs
    need the tracking permission, so under low battery the route only holds. The way home is
    GOTO_DOCK_APPROACH / DESCEND_TO_DOCK.
  - `test_comm_loss_landing_mid_route_is_called_off_when_the_coordinator_returns`: 12 s of
    silence starts a comm-loss landing. The executor's fresh ID calls it off and the route completes.
- `test_progress_serializes`.

### `test_state_machine_alignment.py`: Motion as driven by `state_machine/run_loop.py`

Swaps the `MotionStub` inside `run_loop.run_world()` for a recorded one and
runs every demo in `state_machine/demos.py`.

- `test_motion_accepts_everything_and_the_flight_is_clean` (every demo): every command
  is accepted, the trace is clean, Motion never refused or clamped anything, and every
  command carries a real mission state.
- `test_docking_demos_land_exactly_on_the_pad_before_docked`, `test_emergency_demo_descends_in_place_and_disarms`,
  `test_blocked_dock_holds_still_at_takeoff_altitude`.
- `test_settled_landing_survives_its_command_expiring`, `test_off_pad_touchdown_is_not_settled_and_holds_when_it_expires`.

If the state machine changes how it drives Motion, this is the file that fails.

### `test_trace_checker.py`: `motion_engine/trace_checker.py`

Each rule gets a clean trace and a minimal trace that breaks exactly that
rule, so a rule that stops firing (or starts firing on good data) fails here
first.

- **Clean traces**: `test_clean_trace_passes_every_rule`, `test_moving_exactly_at_every_limit_is_allowed`,
  `test_yaw_rate_uses_the_short_way_round`, `test_pose_only_trace_skips_the_rules_that_need_flags`.
- **Every rule fires**: `test_every_rule_has_a_bad_trace` (the `BAD_TRACES` table must cover
  `RULES`), `test_rule_fires_on_its_bad_trace` (15 rules), plus
  `test_nan_sample_does_not_cascade_into_geometry_violations`,
  `test_emergency_release_flags_a_vehicle_still_armed_after_touchdown`,
  `test_accepted_ack_must_become_the_active_command`, `test_renewal_and_rejection_in_one_tick_are_fine`,
  `test_custom_limits_are_used`, `test_geofence_is_measured_from_home`.
- **Emergency and landing branches**: `test_emergency_motion_catches_each_way_of_breaking_the_descent`
  (too fast, climbing, sideways, each tripping only `emergency_motion`),
  `test_emergency_descent_exactly_at_the_emergency_rate_is_allowed`,
  `test_the_tick_an_emergency_starts_is_held_to_the_emergency_rate`,
  `test_status_landed_without_landing_settled_is_flagged`.
- **Metrics**: `test_metrics_of_a_straight_20_m_flight`, `test_landing_error_is_measured_from_the_pad`,
  `test_empty_trace_is_trivially_clean`.
- **Recorder and JSONL**: `test_recorder_attaches_acks_to_the_tick_and_stamps_end_of_tick_time`,
  `test_jsonl_round_trip_keeps_every_field`, `test_jsonl_with_an_unknown_schema_version_is_refused`,
  `test_demo_mission_is_a_clean_nominal_landing_checked_by_every_rule`, `test_demo_mission_is_deterministic`.
- **Synthetic CSVs**: `test_synthetic_generator_output_passes` (5 scenarios from
  `synthetic/generate_motion_data.py`), `test_csv_loader_reads_files_without_metadata`,
  `test_csv_with_a_teleport_is_caught`, `test_unknown_file_type_is_refused`.
- **CLI**: `test_cli_exit_codes` (0 clean, 1 violations, 2 missing file), `test_cli_expands_wildcards_itself`,
  `test_cli_demo_json_and_save`, `test_cli_needs_input`.

### `test_universal_logging.py`: Motion's records in the shared UniversalLog

- `test_universal_records_match_memory_sink_records`: nothing is dropped or reshaped
  on the way into UniversalLog.
- `test_real_universal_log_writes_one_json_line_per_record`, `test_motion_and_cv_share_one_run_file`.
- `test_state_enum_is_written_as_its_value`, `test_missing_state_defaults_to_idle_like_cv`,
  `test_unknown_state_is_kept_and_flagged_by_universal_log`,
  `test_non_json_values_do_not_break_the_logger` (NaN, inf, enums, tuples, objects).

## Adding a test

- Use the `rig` / `airborne_rig` fixtures and finish with `rig.check()`, so the trace
  checker also looks at your flight.
- Put a failure scenario in the matching section of `test_failure_injection.py`.
- A new `MotionStub` method needs a `# PHASE-1 HOOK:` comment, or
  `test_every_public_command_has_phase1_hook` fails.
- A new `Reason` or payload field must also go in `motion_engine/README.md`, or
  the doc tests fail.
- A new trace-checker rule needs an entry in `BAD_TRACES`.
