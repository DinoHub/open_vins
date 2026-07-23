import math

import pytest
from vo_eval_tools.trajectory import Figure8Trajectory, Figure8WeaveTrajectory


def make_traj(half_length=1.5, half_width=1.0, loop_period=25.0,
              center=(2.0, -3.0, 4.0), start_yaw=0.3, yaw_offset=0.0):
    return Figure8Trajectory(
        half_length_m=half_length,
        half_width_m=half_width,
        loop_period_s=loop_period,
        center_xyz=center,
        start_yaw_rad=start_yaw,
        pattern_yaw_offset_rad=yaw_offset,
    )


def make_weave_traj(
        half_length=1.5, half_width=1.0, loop_period=25.0,
        center=(2.0, -3.0, 4.0), start_yaw=0.3, yaw_offset=0.0,
        weave_height=1.0, weave_cycles=2):
    return Figure8WeaveTrajectory(
        half_length_m=half_length,
        half_width_m=half_width,
        loop_period_s=loop_period,
        center_xyz=center,
        start_yaw_rad=start_yaw,
        weave_height_m=weave_height,
        weave_cycles_per_loop=weave_cycles,
        pattern_yaw_offset_rad=yaw_offset,
    )


def test_starts_at_center():
    # Position must start exactly at the center regardless of yaw rotation.
    center = (2.0, -3.0, 4.0)
    traj = make_traj(center=center, start_yaw=0.3)
    x, y, z, _vx, _vy, vz, _yaw = traj.sample(0.0)
    assert x == pytest.approx(center[0], abs=1e-9)
    assert y == pytest.approx(center[1], abs=1e-9)
    assert z == pytest.approx(center[2], abs=1e-9)
    assert vz == pytest.approx(0.0, abs=1e-9)


def test_local_velocity_at_start_with_no_rotation():
    # With yaw=0 the world frame equals the local frame, so vx_local(0) = 0
    # is directly observable (vy_local(0) = 2*half_width*omega is legitimately
    # nonzero - the curve does not start at rest, only at the center point;
    # figure8_controller's ramp_duration_s blend handles this at flight entry).
    traj = make_traj(start_yaw=0.0, yaw_offset=0.0)
    _x, _y, _z, vx, vy, vz, _yaw = traj.sample(0.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    expected_vy = 2.0 * traj.half_width * traj.omega
    assert vy == pytest.approx(expected_vy, rel=1e-9)
    assert vz == pytest.approx(0.0, abs=1e-9)


def test_ends_at_center_after_one_loop():
    center = (2.0, -3.0, 4.0)
    loop_period = 25.0
    traj = make_traj(center=center, loop_period=loop_period)
    x, y, z, _vx, _vy, _vz, _yaw = traj.sample(loop_period)
    assert x == pytest.approx(center[0], abs=1e-6)
    assert y == pytest.approx(center[1], abs=1e-6)
    assert z == pytest.approx(center[2], abs=1e-9)


def test_periodicity():
    loop_period = 25.0
    traj = make_traj(loop_period=loop_period)
    for t in (0.0, 3.7, 12.1, 20.0):
        a = traj.sample(t)
        b = traj.sample(t + loop_period)
        for va, vb in zip(a, b):
            assert va == pytest.approx(vb, abs=1e-6)


def test_max_horizontal_speed_matches_numeric_search():
    traj = make_traj()
    analytic_max = traj.max_horizontal_speed()

    numeric_max = 0.0
    n = 200000
    for i in range(n):
        t = traj.loop_period * i / n
        _x, _y, _z, vx, vy, _vz, _yaw = traj.sample(t)
        numeric_max = max(numeric_max, math.hypot(vx, vy))

    assert numeric_max <= analytic_max + 1e-3
    assert numeric_max == pytest.approx(analytic_max, rel=1e-3)


def test_default_scale_speed_is_conservative():
    # Regression guard: the room-scale defaults used by figure8_controller's
    # PARAM_DEFAULTS should stay comfortably under both the recommended
    # 1-2 m/s guidance and this vehicle's MPC_XY_VEL_MAX=5 m/s PX4 param.
    traj = make_traj(half_length=2.5, half_width=1.5, loop_period=20.0)
    assert traj.max_horizontal_speed() < 2.0


def test_yaw_offset_rotates_pattern():
    traj_a = make_traj(start_yaw=0.0, yaw_offset=0.0)
    traj_b = make_traj(start_yaw=0.0, yaw_offset=math.pi / 2)
    t = 5.0
    xa, ya, _za, _vxa, _vya, _vza, _yawa = traj_a.sample(t)
    xb, yb, _zb, _vxb, _vyb, _vzb, _yawb = traj_b.sample(t)
    # A 90 degree rotation should not leave the sampled point unchanged
    # (unless it happens to land exactly on the axis of symmetry).
    assert (xa, ya) != pytest.approx((xb, yb))


def test_invalid_parameters_raise():
    with pytest.raises(ValueError):
        make_traj(half_length=0.0)
    with pytest.raises(ValueError):
        make_traj(half_width=-1.0)
    with pytest.raises(ValueError):
        make_traj(loop_period=0.0)


def test_weave_starts_and_ends_at_center_altitude():
    center = (2.0, -3.0, 4.0)
    loop_period = 25.0
    traj = make_weave_traj(
        center=center, loop_period=loop_period, weave_height=1.0, weave_cycles=2)
    _x0, _y0, z0, _vx0, _vy0, vz0, _yaw0 = traj.sample(0.0)
    assert z0 == pytest.approx(center[2], abs=1e-9)
    assert vz0 == pytest.approx(traj.max_vertical_speed(), abs=1e-9)

    _x1, _y1, z1, *_rest = traj.sample(loop_period)
    assert z1 == pytest.approx(center[2], abs=1e-6)


def test_weave_reaches_full_amplitude():
    center = (0.0, 0.0, 4.0)
    loop_period = 25.0
    weave_height = 1.2
    traj = make_weave_traj(
        center=center, loop_period=loop_period, weave_height=weave_height, weave_cycles=2)
    quarter_cycle_t = (2.0 * math.pi / traj.weave_omega) / 4.0
    _x, _y, z, *_rest = traj.sample(quarter_cycle_t)
    assert z == pytest.approx(center[2] + weave_height / 2.0, abs=1e-6)


def test_weave_matches_base_trajectory_horizontally():
    kwargs = {
        'half_length': 1.5, 'half_width': 1.0, 'loop_period': 25.0,
        'center': (2.0, -3.0, 4.0), 'start_yaw': 0.3, 'yaw_offset': 0.1,
    }
    flat = make_traj(**kwargs)
    weave = make_weave_traj(**kwargs, weave_height=2.0, weave_cycles=3)
    for t in (0.0, 3.7, 12.1, 20.0):
        xf, yf, _zf, vxf, vyf, _vzf, yawf = flat.sample(t)
        xw, yw, _zw, vxw, vyw, _vzw, yaww = weave.sample(t)
        assert (xf, yf, vxf, vyf, yawf) == pytest.approx((xw, yw, vxw, vyw, yaww))


def test_weave_max_vertical_speed_matches_numeric_search():
    traj = make_weave_traj(weave_height=1.5, weave_cycles=3)
    analytic_max = traj.max_vertical_speed()

    numeric_max = 0.0
    n = 200000
    for i in range(n):
        t = traj.loop_period * i / n
        _x, _y, _z, _vx, _vy, vz, _yaw = traj.sample(t)
        numeric_max = max(numeric_max, abs(vz))

    assert numeric_max <= analytic_max + 1e-3
    assert numeric_max == pytest.approx(analytic_max, rel=1e-3)


def test_weave_invalid_parameters_raise():
    with pytest.raises(ValueError):
        make_weave_traj(weave_height=-1.0)
    with pytest.raises(ValueError):
        make_weave_traj(weave_cycles=0)
