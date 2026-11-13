"""Tests for `motion_engine/trace_checker.py`.

Each rule gets a clean trace and a minimal trace that breaks exactly that
rule, so a rule that silently stops firing (or starts firing on good data)
fails here first. Also covers the metrics, the JSONL recorder round trip,
the synthetic-CSV loader, the demo mission and the CLI exit codes.
"""

import json
import sys

from motion_engine.tests.motion_rig import DT_NS as TICK_NS, MOTION_ENGINE_ROOT

_SYNTHETIC = MOTION_ENGINE_ROOT / "synthetic"
if str(_SYNTHETIC) not in sys.path:
    sys.path.insert(0, str(_SYNTHETIC))

import pytest

from motion_engine.contracts import Command, MotionCommandType as K, Pose, SimulationClock
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.trace_checker import (
    RULES,
    TRACE_SCHEMA_VERSION,
    TraceRecorder,
    TraceSample,
    check_trace,
    load_jsonl,
    load_synthetic_csv,
    load_trace,
    main,
    record_demo_mission,
)


def sample(tick, x=0.0, y=0.0, z=0.0, yaw=0.0, **kw):
    """A trace sample at `tick` * 0.1 s. Full flags by default (airborne from z)."""
    flags = dict(
        armed=True, airborne=z > 0.05, grounded=z <= 0.05, emergency_descent=False,
        landing_settled=False, status="in_progress", active_command_id="a", has_active_id=True,
    )
    flags.update(kw)
    return TraceSample(t_ns=tick * TICK_NS, x=x, y=y, z=z, yaw_deg=yaw, **flags)


def straight_line(n=10, speed=5.0, z=5.0):
    return [sample(i, x=i * speed * 0.1, z=z) for i in range(n)]


# -- clean traces -----------------------------------------------------------


def test_clean_trace_passes_every_rule():
    report = check_trace(straight_line())
    assert report.ok, report.format_text()
    assert report.violations == []


def test_moving_exactly_at_every_limit_is_allowed():
    trace = [sample(0, z=5.0), sample(1, x=0.5, z=5.2, yaw=9.0)]  # 5 m/s, 2 m/s, 90 deg/s
    assert check_trace(trace).ok


def test_yaw_rate_uses_the_short_way_round():
    trace = [sample(0, z=5.0, yaw=358.0), sample(1, z=5.0, yaw=4.0)]  # 6 deg, not 354
    assert check_trace(trace).ok


def test_pose_only_trace_skips_the_rules_that_need_flags():
    trace = [TraceSample(t_ns=i * TICK_NS, x=0.1 * i, y=0.0, z=1.0, yaw_deg=0.0) for i in range(5)]
    report = check_trace(trace)
    assert report.ok
    assert set(report.rules_checked) == {
        "finite_pose", "time_monotonic", "below_ground", "above_ceiling",
        "outside_geofence", "horizontal_speed", "vertical_speed", "yaw_rate",
    }


# -- every rule fires on a minimal bad trace ----------------------------------

BAD_TRACES = {
    "finite_pose": [sample(0, z=5.0), sample(1, x=float("nan"), z=5.0)],
    "time_monotonic": [sample(1, z=5.0), sample(1, z=5.0)],
    "below_ground": [sample(0, z=-0.2, airborne=False, grounded=True)],
    "above_ceiling": [sample(0, z=30.5)],
    "outside_geofence": [sample(0, x=150.5, z=5.0)],
    "horizontal_speed": [sample(0, z=5.0), sample(1, x=0.6, z=5.0)],
    "vertical_speed": [sample(0, z=5.0), sample(1, z=5.3)],
    "yaw_rate": [sample(0, z=5.0, yaw=0.0), sample(1, z=5.0, yaw=10.0)],
    "disarmed_motion": [sample(0, armed=False), sample(1, x=0.1, armed=False)],
    "ground_flags": [sample(0, z=0.02, airborne=True, grounded=False)],
    "landed_off_ground": [sample(0, z=0.2, landing_settled=True, status="landed")],
    "emergency_motion": [sample(0, z=5.0, emergency_descent=True),
                         sample(1, x=0.3, z=4.9, emergency_descent=True)],
    "emergency_release": [sample(0, z=3.0, emergency_descent=True),
                          sample(1, z=2.9, emergency_descent=False)],
    "expired_hold_moved": [sample(0, z=5.0), sample(1, x=0.2, z=5.0, status="expired_hold")],
    "ack_active_id": [
        sample(0, z=5.0, active_command_id="a"),
        sample(1, z=5.0, active_command_id="b",
               acks=[{"command_id": "b", "accepted": False, "reason": "not_armed",
                      "active_command_id": "b"}]),
    ],
}


def test_every_rule_has_a_bad_trace():
    assert set(BAD_TRACES) == set(RULES)


@pytest.mark.parametrize("rule", RULES)
def test_rule_fires_on_its_bad_trace(rule):
    report = check_trace(BAD_TRACES[rule])
    assert not report.ok
    assert rule in report.by_rule(), report.format_text()
    hit = report.rule_hits(rule)[0]
    assert hit.message and hit.t_ns == BAD_TRACES[rule][hit.index].t_ns


def test_nan_sample_does_not_cascade_into_geometry_violations():
    report = check_trace(BAD_TRACES["finite_pose"] + [sample(2, x=1.0, z=5.0)])
    assert report.by_rule() == {"finite_pose": 1}


def test_emergency_release_flags_a_vehicle_still_armed_after_touchdown():
    trace = [sample(0, z=0.1, emergency_descent=True),
             sample(1, z=0.0, emergency_descent=False, armed=True)]
    hits = check_trace(trace).rule_hits("emergency_release")
    assert [h.message for h in hits] == ["still armed after emergency touchdown"]


#: One tick of emergency descent (default limits: 1 m/s, so 0.1 m per tick).
EMERGENCY_STEPS = {
    "too_fast": (dict(z=4.85), "descended 1.50 m/s, emergency rate is 1.0 m/s"),
    "climbing": (dict(z=5.05), "climbed 0.050 m during emergency descent"),
    "sideways": (dict(x=0.05, z=4.9), "moved 0.050 m sideways during emergency descent"),
}


@pytest.mark.parametrize("case", sorted(EMERGENCY_STEPS))
def test_emergency_motion_catches_each_way_of_breaking_the_descent(case):
    """Straight down at no more than the emergency rate: anything else is
    flagged, and only by emergency_motion (1.5 m/s is still under the normal
    2 m/s vertical cap, so vertical_speed stays quiet)."""
    step, message = EMERGENCY_STEPS[case]
    trace = [sample(0, z=5.0, emergency_descent=True), sample(1, emergency_descent=True, **step)]
    report = check_trace(trace)
    assert report.by_rule() == {"emergency_motion": 1}, report.format_text()
    assert report.rule_hits("emergency_motion")[0].message == message


def test_emergency_descent_exactly_at_the_emergency_rate_is_allowed():
    trace = [sample(i, z=5.0 - 0.1 * i, emergency_descent=True) for i in range(5)]
    assert check_trace(trace).ok


def test_the_tick_an_emergency_starts_is_held_to_the_emergency_rate():
    """Motion accepts EMERGENCY_LAND before the step, so the first emergency
    tick must already be at 1 m/s, not the normal 2 m/s descent."""
    trace = [sample(0, z=5.0), sample(1, z=4.8, emergency_descent=True)]
    assert check_trace(trace).by_rule() == {"emergency_motion": 1}
    trace[1] = sample(1, z=4.9, emergency_descent=True)
    assert check_trace(trace).ok


def test_status_landed_without_landing_settled_is_flagged():
    """Feedback that says 'landed' while landing_settled is False contradicts
    itself even on the ground; Docking reads landing_settled, the state
    machine reads status, and they must agree."""
    trace = [sample(0, z=0.0, status="landed", landing_settled=False)]
    hits = check_trace(trace).rule_hits("landed_off_ground")
    assert [h.message for h in hits] == ["status 'landed' but landing_settled is False"]
    assert check_trace([sample(0, z=0.0, status="landed", landing_settled=True)]).ok


def test_accepted_ack_must_become_the_active_command():
    trace = [sample(0, z=5.0, active_command_id="a"),
             sample(1, z=5.0, active_command_id="a",
                    acks=[{"command_id": "b", "accepted": True, "reason": "accepted",
                           "active_command_id": "a"}])]
    assert "ack_active_id" in check_trace(trace).by_rule()


def test_renewal_and_rejection_in_one_tick_are_fine():
    trace = [sample(0, z=5.0, active_command_id="a"),
             sample(1, z=5.0, active_command_id="a", acks=[
                 {"command_id": "a", "accepted": True, "reason": "renewed", "active_command_id": "a"},
                 {"command_id": "x", "accepted": False, "reason": "invalid_command",
                  "active_command_id": "a"},
             ])]
    report = check_trace(trace)
    assert report.ok
    assert report.metrics["rejected_by_reason"] == {"invalid_command": 1}


def test_custom_limits_are_used():
    slow = MotionLimits(max_horizontal_speed_mps=1.0, max_altitude_m=3.0, geofence_radius_m=2.0)
    report = check_trace(straight_line(n=10, speed=5.0, z=5.0), limits=slow)
    assert {"horizontal_speed", "above_ceiling", "outside_geofence"} <= set(report.by_rule())


def test_geofence_is_measured_from_home():
    trace = [sample(0, x=290.0, z=5.0)]
    assert not check_trace(trace).ok
    assert check_trace(trace, home=Pose(200.0, 0.0, 0.0, 0.0)).ok


# -- metrics -------------------------------------------------------------------


def test_metrics_of_a_straight_20_m_flight():
    trace = straight_line(n=41, speed=5.0, z=5.0)      # 4 s at 5 m/s
    m = check_trace(trace).metrics
    assert m["duration_s"] == pytest.approx(4.0)
    assert m["path_length_m"] == pytest.approx(20.0)
    assert m["max_horizontal_speed_mps"] == pytest.approx(5.0)
    assert m["max_vertical_speed_mps"] == 0.0
    assert m["max_radius_m"] == pytest.approx(20.0)
    assert m["airborne_time_s"] == pytest.approx(4.0)
    assert m["landing_error_m"] is None                # never came down


def test_landing_error_is_measured_from_the_pad():
    trace = [sample(0, x=3.0, y=4.0, z=0.0, landing_settled=True, status="landed")]
    assert check_trace(trace).metrics["landing_error_m"] == pytest.approx(5.0)
    assert check_trace(trace, pad=Pose(3.0, 4.0, 0.0, 0.0)).metrics["landing_error_m"] == 0.0
    assert check_trace(trace).metrics["landed"] is True


def test_empty_trace_is_trivially_clean():
    report = check_trace([])
    assert report.ok and report.samples == 0 and report.metrics["final_pose"] is None


# -- recorder / JSONL ----------------------------------------------------------


def test_recorder_attaches_acks_to_the_tick_and_stamps_end_of_tick_time():
    clock = SimulationClock()
    rec = TraceRecorder(MotionStub(clock=clock))
    rec.snapshot(clock.timestamp_ns)
    arm = Command(command_id="arm", kind=K.ARM, mission_state="T", issued_ns=0,
                  expires_ns=None, source="t", reason="t")
    rec.submit(arm, clock.timestamp_ns)
    rec.step(0.1, clock.timestamp_ns)
    assert [s.t_ns for s in rec.samples] == [0, TICK_NS]
    assert rec.samples[0].acks == []
    assert [a["command_id"] for a in rec.samples[1].acks] == ["arm"]
    assert rec.samples[1].armed is True


def test_jsonl_round_trip_keeps_every_field(tmp_path):
    rec = record_demo_mission()
    path = rec.write_jsonl(tmp_path / "demo.jsonl")
    header = json.loads(path.read_text().splitlines()[0])
    assert header["trace_schema_version"] == TRACE_SCHEMA_VERSION
    loaded = load_jsonl(path)
    assert [s.to_dict() for s in loaded] == [s.to_dict() for s in rec.samples]
    assert check_trace(loaded).metrics == check_trace(rec.samples).metrics


def test_jsonl_with_an_unknown_schema_version_is_refused(tmp_path):
    path = tmp_path / "future.jsonl"
    path.write_text(json.dumps({"trace_schema_version": 99}) + "\n")
    with pytest.raises(ValueError, match="trace_schema_version"):
        load_jsonl(path)


def test_demo_mission_is_a_clean_nominal_landing_checked_by_every_rule():
    rec = record_demo_mission()
    report = check_trace(rec.samples, limits=rec.motion.limits)
    assert report.ok, report.format_text()
    assert report.rules_checked == list(RULES)  # full feedback: no rule skipped
    m = report.metrics
    assert m["landed"] is True and m["landing_error_m"] == 0.0
    assert m["rejected_by_reason"] == {} and m["expired_hold_ticks"] == 0
    assert m["max_horizontal_speed_mps"] == pytest.approx(5.0)


def test_demo_mission_is_deterministic():
    a = [s.to_dict() for s in record_demo_mission().samples]
    b = [s.to_dict() for s in record_demo_mission().samples]
    assert a == b


# -- synthetic CSVs --------------------------------------------------------------


@pytest.fixture(scope="module")
def generator():
    return pytest.importorskip("generate_motion_data")


@pytest.mark.parametrize("scenario", ["up_down", "square", "rotate", "drift", "random_walk"])
def test_synthetic_generator_output_passes(tmp_path, generator, scenario):
    moves = generator.scenario_moves(scenario, seed=3)
    drift = generator.DRIFT_MPS.get(scenario, (0.0, 0.0))
    rows = generator.generate(moves, drift_mps=drift)
    path = tmp_path / f"{scenario}.csv"
    generator.write_csv(rows, path, generator.metadata(scenario, moves, 10.0, 0.0, 3, drift))
    samples = load_synthetic_csv(path)
    assert len(samples) == len(rows)
    report = check_trace(samples, name=scenario)
    assert report.ok, report.format_text()


def test_csv_loader_reads_files_without_metadata(tmp_path):
    path = tmp_path / "bare.csv"
    path.write_text("t_s,timestamp_ns,step,direction,x_m,y_m,z_m,yaw_deg\n"
                    "0.0,0,0,up,0,0,0,0\n0.1,100000000,0,up,0,0,0.05,0\n")
    samples = load_trace(path)
    assert [s.z for s in samples] == [0.0, 0.05]


def test_csv_with_a_teleport_is_caught(tmp_path):
    path = tmp_path / "teleport.csv"
    path.write_text("# scenario=teleport\n"
                    "t_s,timestamp_ns,step,direction,x_m,y_m,z_m,yaw_deg\n"
                    "0.0,0,0,hover,0,0,1,0\n0.1,100000000,0,hover,8,0,1,0\n")
    assert check_trace(load_trace(path)).by_rule() == {"horizontal_speed": 1}


def test_unknown_file_type_is_refused(tmp_path):
    with pytest.raises(ValueError):
        load_trace(tmp_path / "trace.txt")


# -- CLI -------------------------------------------------------------------------


def _write_csv(path, rows):
    path.write_text("t_s,timestamp_ns,step,direction,x_m,y_m,z_m,yaw_deg\n"
                    + "".join(f"{i / 10},{i * TICK_NS},0,hover,{x},0,1,0\n" for i, x in enumerate(rows)))
    return path


def test_cli_exit_codes(tmp_path, capsys):
    good = _write_csv(tmp_path / "good.csv", [0.0, 0.1, 0.2])
    bad = _write_csv(tmp_path / "bad.csv", [0.0, 3.0])
    assert main([str(good)]) == 0
    assert main([str(good), str(bad)]) == 1
    assert main([str(tmp_path / "missing.csv")]) == 2
    out = capsys.readouterr().out
    assert "horizontal_speed" in out and "1/2 traces clean" in out


def test_cli_expands_wildcards_itself(tmp_path, capsys):
    _write_csv(tmp_path / "a.csv", [0.0, 0.1])
    _write_csv(tmp_path / "b.csv", [0.0, 0.2])
    assert main([str(tmp_path / "*.csv")]) == 0
    assert "2/2 traces clean" in capsys.readouterr().out


def test_cli_demo_json_and_save(tmp_path, capsys):
    saved = tmp_path / "demo.jsonl"
    assert main(["--demo", "--json", "--save", str(saved)]) == 0
    reports = json.loads(capsys.readouterr().out)
    assert reports[0]["name"] == "demo_mission" and reports[0]["ok"] is True
    assert main([str(saved)]) == 0


def test_cli_needs_input():
    with pytest.raises(SystemExit):
        main([])
