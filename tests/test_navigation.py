"""Navigation decides where to go; Motion only flies the pose it is given.

`standoff_target` is the tracking geometry that used to live in
`MotionStub.track_target()`; these are the same cases, now tested where the
decision belongs.
"""

import math

import pytest

from motion_engine.contracts import Pose
from motion_engine.motion_stubs import MotionStub
from navigation import build_route, standoff_target
from navigation.tracking import BACK_OFF_TO_STANDOFF, CLOSE_TO_STANDOFF, HOLD_STANDOFF

HERE = Pose(0.0, 0.0, 12.0, 0.0)


def test_standoff_east():
    t = standoff_target(HERE, bearing_deg=0.0, distance_m=40.0, standoff_m=15.0)
    assert t.pose.as_tuple() == pytest.approx((25.0, 0.0, 12.0, 0.0))
    assert t.decision == CLOSE_TO_STANDOFF and t.travel_m == pytest.approx(25.0)


def test_standoff_north_faces_target_and_sets_altitude():
    t = standoff_target(HERE, bearing_deg=90.0, distance_m=30.0, standoff_m=10.0, altitude_m=15.0)
    assert t.pose.as_tuple() == pytest.approx((0.0, 20.0, 15.0, 90.0), abs=1e-9)


def test_backs_off_when_too_close():
    t = standoff_target(HERE, bearing_deg=0.0, distance_m=5.0, standoff_m=15.0)
    assert t.pose.x == pytest.approx(-10.0) and t.decision == BACK_OFF_TO_STANDOFF


def test_holds_inside_tolerance():
    t = standoff_target(HERE, bearing_deg=45.0, distance_m=15.1, standoff_m=15.0)
    assert t.decision == HOLD_STANDOFF


def test_relative_to_vehicle_and_negative_bearing_wraps():
    t = standoff_target(Pose(10.0, 10.0, 5.0, 0.0), bearing_deg=-90.0, distance_m=25.0)
    assert t.pose.as_tuple() == pytest.approx((10.0, 0.0, 5.0, 270.0), abs=1e-9)


@pytest.mark.parametrize("kw", [
    {"bearing_deg": math.nan, "distance_m": 10.0},
    {"bearing_deg": 0.0, "distance_m": math.inf},
    {"bearing_deg": 0.0, "distance_m": -1.0},
    {"bearing_deg": 0.0, "distance_m": 10.0, "standoff_m": -1.0},
])
def test_bad_geometry_raises(kw):
    with pytest.raises(ValueError):
        standoff_target(HERE, **kw)


def test_motion_flies_navigations_pose():
    m = MotionStub()
    m.arm(); m.takeoff(12.0)
    while not m.at_target():
        m.step(0.1)
    t = standoff_target(m.pose, bearing_deg=0.0, distance_m=40.0)
    assert m.track_position(t.pose.x, t.pose.y, t.pose.z, yaw_deg=t.pose.yaw_deg)
    while not m.at_target():
        m.step(0.1)
    assert m.pose == t.pose


# -- routes ----------------------------------------------------------------------


def test_build_route_from_xy_faces_direction_of_travel():
    r = build_route([(10, 0), (10, 10), (0, 10)], altitude_m=5.0, start=Pose(0, 0, 5, 0))
    assert [p.as_tuple() for p in r] == [
        pytest.approx((10, 0, 5, 0)), pytest.approx((10, 10, 5, 90)), pytest.approx((0, 10, 5, 180)),
    ]


def test_build_route_keeps_z_and_pose_yaw_when_asked():
    r = build_route([(1, 2, 3), Pose(4, 5, 6, 77)], face_direction_of_travel=False)
    assert r == [Pose(1, 2, 3, 0), Pose(4, 5, 6, 77)]


def test_build_route_altitude_overrides_and_zero_leg_keeps_heading():
    r = build_route([(0, 10, 3), (0, 10, 9)], altitude_m=5.0, start=Pose(0, 0, 5, 0))
    assert [p.yaw_deg for p in r] == pytest.approx([90.0, 90.0])
    assert {p.z for p in r} == {5.0}


@pytest.mark.parametrize("points, kw", [([], {}), ([(1, 2)], {}), ([(1,)], {"altitude_m": 5})])
def test_build_route_rejects_bad_input(points, kw):
    with pytest.raises(ValueError):
        build_route(points, **kw)
