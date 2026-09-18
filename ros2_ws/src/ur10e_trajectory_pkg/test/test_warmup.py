#!/usr/bin/env python3
"""Warmup gate: home to the chosen start, rest to rest, or an honest refusal."""
import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from ur10e_trajectory_pkg import motion_limits, warmup
from ur10e_trajectory_pkg.validation_core import TrajectoryValidator

from test_geometry_invariants import _urdf_path


@pytest.fixture(scope='module')
def validator():
    return TrajectoryValidator(
        _urdf_path(), mesh_base_path=get_package_share_directory('ur_description'))


@pytest.fixture(scope='module')
def home():
    return np.concatenate(([1.5], np.deg2rad([0.0, -75.0, 100.0, -115.0, -80.0, 0.0])))


@pytest.fixture(scope='module')
def start(home):
    return home + np.concatenate(([0.4], np.deg2rad([20.0, -10.0, 15.0, 5.0, 30.0, 45.0])))


def test_the_warmup_ends_exactly_at_the_start_and_at_rest(validator, home, start):
    result = warmup.plan_warmup(validator, home, start)
    assert result['status'] == warmup.OK
    positions = np.asarray(result['positions'])
    np.testing.assert_allclose(positions[0], home, atol=1e-12)
    np.testing.assert_allclose(positions[-1], start, atol=1e-12)
    _, _, velocity, acceleration = warmup.sample_rest_to_rest(
        home, start, result['duration_s'])
    np.testing.assert_allclose(velocity[[0, -1]], 0.0, atol=1e-12)
    np.testing.assert_allclose(acceleration[[0, -1]], 0.0, atol=1e-12)


def test_the_closed_form_duration_touches_a_limit_and_never_exceeds_one(validator, home, start):
    """Sampled densely, the binding limit is reached and none is exceeded."""
    result = warmup.plan_warmup(validator, home, start, rate_hz=5000.0)
    peak = max(result['peak_velocity_ratio'], result['peak_acceleration_ratio'],
               result['peak_jerk_ratio'])
    assert peak <= 1.0 + 1e-9
    assert peak == pytest.approx(1.0, abs=1e-4)
    assert result['binding']['kind'] in ('velocity', 'acceleration', 'jerk')


def test_a_short_rail_move_is_sized_by_its_jerk_limit(home):
    """0.2 m of rail sized on velocity and acceleration took 0.48 s and peaked
    at 104 m/s^3 against the rail's 100; validation refused what planning
    accepted. The jerk bound gives cbrt(60 |D| / J)."""
    start = home.copy()
    start[0] += 0.2
    velocity, acceleration, jerk = np.full(7, 10.0), np.full(7, 5.0), np.full(7, 100.0)
    duration, joint, kind = warmup.rest_to_rest_duration(
        home, start, velocity, acceleration, jerk_limits=jerk)
    assert joint == 0 and kind == 'jerk'
    assert duration == pytest.approx(np.cbrt(60.0 * 0.2 / 100.0))
    assert 60.0 * 0.2 / duration ** 3 == pytest.approx(100.0)


def test_the_binding_joint_is_the_slowest_to_move(home):
    velocity = np.ones(7)
    acceleration = np.full(7, 100.0)
    start = home.copy()
    start[4] += 2.0
    duration, joint, kind = warmup.rest_to_rest_duration(home, start, velocity,
                                                         acceleration)
    assert joint == 4 and kind == 'velocity'
    assert duration == pytest.approx(15.0 * 2.0 / 8.0)


def test_no_motion_takes_the_minimum_duration(validator, home):
    result = warmup.plan_warmup(validator, home, home)
    assert result['status'] == warmup.OK
    assert result['duration_s'] == warmup.MINIMUM_DURATION_S
    assert result['binding']['kind'] == 'minimum_duration'


def test_a_blocked_straight_move_reports_no_direct_warmup(validator, home, start,
                                                          monkeypatch):
    monkeypatch.setattr(validator, 'check_all_collisions',
                        lambda q, verbose=False: True)
    result = warmup.plan_warmup(validator, home, start)
    assert result['status'] == warmup.NO_DIRECT_WARMUP
    assert 'positions' not in result
    assert result['collision_queries'] == 1


def test_a_straight_move_below_the_self_clearance_floor_is_not_ok(validator, home, start,
                                                                  monkeypatch):
    """Collision alone let a move passing 6.5 mm from itself plan as ok, and
    only validation, of the winner alone, then refused it."""
    monkeypatch.setattr(validator, 'self_clearance',
                        lambda q: {'distance_m': 0.0065,
                                   'links': ['forearm_link', 'wrist_2_link']})
    result = warmup.plan_warmup(validator, home, start)
    assert result['status'] == warmup.DIRECT_BELOW_SELF_CLEARANCE
    assert result['min_self_clearance_m'] == pytest.approx(0.0065)
    assert '6.5 mm' in result['reason'] and 'positions' not in result


def test_a_direct_move_below_the_floor_routes_through_a_via(monkeypatch):
    home, via, start = np.zeros(7), np.full(7, 0.3), np.full(7, 0.9)

    def fake_plan(validator, begin, end, velocity_limits=None,
                  acceleration_limits=None, step_bounds=None, rate_hz=100.0):
        if np.allclose(begin, home) and np.allclose(end, start):
            return {'status': warmup.DIRECT_BELOW_SELF_CLEARANCE, 'reason': '6.5 mm'}
        return _segment(begin, end, 1.5, rate_hz)

    from ur10e_trajectory_pkg import continuous_validator
    monkeypatch.setattr(warmup, 'plan_warmup', fake_plan)
    monkeypatch.setattr(continuous_validator, 'validate_warmup',
                        lambda validator, record: {
                            'passed': True, 'self_clearance': {'min_distance_m': 0.02}})
    route = warmup.plan_route(None, home, start, via_poses=[via], rate_hz=100.0)
    assert route['status'] == warmup.OK and route['route_kind'] == 'via'


def test_an_endpoint_outside_the_joint_limits_is_refused(validator, home):
    outside = home.copy()
    outside[3] = np.pi + 0.1
    result = warmup.plan_warmup(validator, home, outside)
    assert result['status'] == warmup.OUTSIDE_JOINT_LIMITS


def test_the_binding_status_comes_from_the_urdf_check(validator, home):
    start = home.copy()
    start[0] += 1.2                  # rail-dominated, inside the 0-3 m travel
    result = warmup.plan_warmup(validator, home, start)
    assert result['binding']['joint'] == 'linear_rail_joint'
    statuses = motion_limits.limit_statuses(validator)
    kind = result['binding']['kind']
    assert result['binding']['status'] == statuses[kind][0]


# --------------------------------------------------------------------------
# Two-segment warmup, for starts whose straight rest-to-rest move collides
# --------------------------------------------------------------------------

def _segment(start_q, end_q, duration, rate_hz=100.0):
    samples = int(duration * rate_hz) + 1
    times = np.arange(samples) / rate_hz
    ramp = np.linspace(0.0, 1.0, samples)[:, None]
    positions = np.asarray(start_q) + ramp * (np.asarray(end_q) - np.asarray(start_q))
    return {'status': warmup.OK, 'times': times.tolist(),
            'positions': positions.tolist(), 'duration_s': float(duration),
            'rate_hz': rate_hz}


def test_the_dwell_holds_the_via_pose_at_rest():
    """The seam between segments must be a genuine rest point: both segments
    end and begin at rest, so the dwell holds the via exactly."""
    home, via, start = np.zeros(7), np.full(7, 0.3), np.full(7, 0.6)
    first, second = _segment(home, via, 1.0), _segment(via, start, 1.0)

    joined = warmup.concatenate_segments(first, second, dwell_s=0.5, rate_hz=100.0)
    times = np.asarray(joined['times'])
    positions = np.asarray(joined['positions'])

    assert np.all(np.diff(times) > 0)
    np.testing.assert_allclose(np.diff(times), 0.01, atol=1e-9)
    np.testing.assert_allclose(positions[0], home)
    np.testing.assert_allclose(positions[-1], start)
    dwell_samples = int(round(0.5 * 100.0))
    held = positions[len(first['times']):len(first['times']) + dwell_samples]
    np.testing.assert_allclose(held, np.repeat(via[None, :], dwell_samples, axis=0))
    assert joined['dwell_s'] == pytest.approx(0.5)


def _patch_search(monkeypatch, blocked_second=(), segment_fails=(), joined_fails=False,
                  durations=None):
    from ur10e_trajectory_pkg import continuous_validator

    durations = durations or {}

    def fake_plan(validator, begin, end, velocity_limits=None,
                  acceleration_limits=None, step_bounds=None, rate_hz=100.0):
        key = round(float(np.asarray(end)[0]), 6)
        if key in blocked_second:
            return {'status': warmup.NO_DIRECT_WARMUP, 'reason': 'collides'}
        return _segment(begin, end, durations.get(key, 1.0), rate_hz)

    def fake_validate(validator, record):
        via = round(float(np.asarray(record['positions'])[-1][0]), 6)
        seam = round(float(np.asarray(record['positions'])[0][0]), 6)
        if via in segment_fails or seam in segment_fails:
            return {'passed': False, 'self_clearance': {'min_distance_m': 0.004}}
        if joined_fails and 'dwell_s' in record:
            return {'passed': False, 'self_clearance': {'min_distance_m': 0.02}}
        return {'passed': True, 'self_clearance': {'min_distance_m': 0.02}}

    monkeypatch.setattr(warmup, 'plan_warmup', fake_plan)
    monkeypatch.setattr(continuous_validator, 'validate_warmup', fake_validate)


def test_the_shortest_validated_via_pose_is_chosen(monkeypatch):
    home, start = np.zeros(7), np.full(7, 0.9)
    vias = [np.full(7, 0.1), np.full(7, 0.2), np.full(7, 0.3)]
    # via 0.1 cannot reach the start; 0.2 and 0.3 can, 0.3 sooner.
    _patch_search(monkeypatch, durations={0.1: 1.0, 0.2: 2.0, 0.3: 0.5, 0.9: 1.0})

    def fake_plan_blocking_first(validator, begin, end, velocity_limits=None,
                                 acceleration_limits=None, step_bounds=None, rate_hz=100.0):
        if np.allclose(begin, vias[0]):
            return {'status': warmup.NO_DIRECT_WARMUP, 'reason': 'collides'}
        return _segment(begin, end, {0.1: 1.0, 0.2: 2.0, 0.3: 0.5, 0.9: 1.0}[
            round(float(np.asarray(end)[0]), 6)], rate_hz)

    monkeypatch.setattr(warmup, 'plan_warmup', fake_plan_blocking_first)
    result = warmup.plan_two_segment_warmup(None, home, start, vias, rate_hz=100.0)

    assert result['status'] == warmup.OK
    np.testing.assert_allclose(result['via'], vias[2])
    assert result['total_duration_s'] == pytest.approx(0.5 + 0.5 + 1.0)
    rejected = [c['rejected'] for c in result['considered']]
    assert rejected[0] == 'second_segment_no_direct_warmup'
    assert result['min_self_clearance_m'] == pytest.approx(0.02)


def test_first_match_stops_at_the_first_route(monkeypatch):
    home, start = np.zeros(7), np.full(7, 0.9)
    vias = [np.full(7, 0.2), np.full(7, 0.3)]
    _patch_search(monkeypatch, durations={0.2: 2.0, 0.3: 0.5, 0.9: 1.0})
    result = warmup.plan_two_segment_warmup(None, home, start, vias, rate_hz=100.0,
                                            first_match=True)
    np.testing.assert_allclose(result['via'], vias[0])
    assert len(result['considered']) == 1


def test_no_via_pose_reports_why_each_was_rejected(monkeypatch):
    home, start = np.zeros(7), np.full(7, 0.9)
    vias = [np.full(7, 0.2), np.full(7, 0.3)]
    _patch_search(monkeypatch, segment_fails=(0.2, 0.3), durations={0.2: 1.0, 0.3: 1.0, 0.9: 1.0})
    result = warmup.plan_two_segment_warmup(None, home, start, vias, rate_hz=100.0)
    assert result['status'] == warmup.NO_TWO_SEGMENT_WARMUP
    assert [c['rejected'] for c in result['considered']] == ['segment_validation'] * 2


def test_a_failing_concatenation_is_not_accepted(monkeypatch):
    """Both segments passing separately is not the same as the command that
    will actually be sent passing."""
    home, start = np.zeros(7), np.full(7, 0.9)
    vias = [np.full(7, 0.3)]
    _patch_search(monkeypatch, joined_fails=True, durations={0.3: 1.0, 0.9: 1.0})
    result = warmup.plan_two_segment_warmup(None, home, start, vias, rate_hz=100.0)
    assert result['status'] == warmup.NO_TWO_SEGMENT_WARMUP
    assert result['considered'][0]['rejected'] == 'joined_validation'


def test_a_direct_route_is_one_segment_with_no_dwell(validator, home, start):
    """One segment is the common case and must not pay for the route format:
    no via, no dwell in the total."""
    route = warmup.plan_route(validator, home, start, rate_hz=200.0)
    assert route['status'] == warmup.OK
    assert route['route_kind'] == 'direct'
    assert len(route['segments']) == 1
    assert route['total_duration_s'] == pytest.approx(route['segments'][0]['duration_s'])
    np.testing.assert_allclose(route['rest_points'][0], home)
    np.testing.assert_allclose(route['rest_points'][-1], start)
    np.testing.assert_allclose(route['segments'][0]['begins_at'], home)
    np.testing.assert_allclose(route['segments'][0]['ends_at'], start)


def test_a_blocked_start_routes_through_a_via_with_the_dwell_counted(monkeypatch):
    """Two legs and the rest between them, in the same shape a direct route
    has, so nothing downstream needs to know which case it got."""
    home, via, start = np.zeros(7), np.full(7, 0.3), np.full(7, 0.9)

    def fake_plan(validator, begin, end, velocity_limits=None,
                  acceleration_limits=None, step_bounds=None, rate_hz=100.0):
        if np.allclose(begin, home) and np.allclose(end, start):
            return {'status': warmup.NO_DIRECT_WARMUP, 'reason': 'collides'}
        return _segment(begin, end, 1.5, rate_hz)

    def fake_validate(validator, record):
        return {'passed': True, 'self_clearance': {'min_distance_m': 0.02}}

    from ur10e_trajectory_pkg import continuous_validator
    monkeypatch.setattr(warmup, 'plan_warmup', fake_plan)
    monkeypatch.setattr(continuous_validator, 'validate_warmup', fake_validate)

    route = warmup.plan_route(None, home, start, via_poses=[via], rate_hz=100.0,
                              dwell_s=0.5)
    assert route['status'] == warmup.OK and route['route_kind'] == 'via'
    assert len(route['segments']) == 2
    assert [s['index'] for s in route['segments']] == [0, 1]
    np.testing.assert_allclose(route['rest_points'][1], via)
    assert route['total_duration_s'] == pytest.approx(1.5 + 0.5 + 1.5)


def test_a_route_names_the_leg_that_cannot_be_planned(monkeypatch):
    home, via, start = np.zeros(7), np.full(7, 0.3), np.full(7, 0.9)

    def fake_plan(validator, begin, end, velocity_limits=None,
                  acceleration_limits=None, step_bounds=None, rate_hz=100.0):
        if np.allclose(begin, via):
            return {'status': warmup.NO_DIRECT_WARMUP, 'reason': 'collides'}
        return _segment(begin, end, 1.0, rate_hz)

    monkeypatch.setattr(warmup, 'plan_warmup', fake_plan)
    route = warmup.route_from_rest_points(None, [home, via, start], rate_hz=100.0)
    assert route['status'] == warmup.NO_DIRECT_WARMUP
    assert route['failed_segment'] == 1

    too_few = warmup.route_from_rest_points(None, [home], rate_hz=100.0)
    assert too_few['status'] == warmup.OUTSIDE_JOINT_LIMITS
