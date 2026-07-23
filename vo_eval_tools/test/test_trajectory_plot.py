import numpy as np
import pytest
from vo_eval_tools.trajectory_plot import _axis_ranges, align_se3, associate


def _rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def test_align_se3_recovers_known_transform():
    rng = np.random.default_rng(42)
    source = rng.uniform(-5, 5, size=(50, 3))

    r_true = _rotation_matrix(0.1, -0.2, 1.3)
    t_true = np.array([2.0, -3.5, 0.7])
    target = (r_true @ source.T).T + t_true

    r_est, t_est = align_se3(source, target)

    assert r_est == pytest.approx(r_true, abs=1e-9)
    assert t_est == pytest.approx(t_true, abs=1e-9)

    recovered = (r_est @ source.T).T + t_est
    assert recovered == pytest.approx(target, abs=1e-9)


def test_align_se3_identity_for_identical_sets():
    rng = np.random.default_rng(1)
    points = rng.uniform(-1, 1, size=(20, 3))
    r, t = align_se3(points, points)
    assert r == pytest.approx(np.eye(3), abs=1e-9)
    assert t == pytest.approx(np.zeros(3), abs=1e-9)


def test_align_se3_too_few_points_raises():
    points = np.zeros((2, 3))
    with pytest.raises(ValueError):
        align_se3(points, points)


def test_associate_matches_close_timestamps():
    t_est = np.array([0.0, 1.0, 2.0, 3.0])
    t_gt = np.array([0.005, 0.99, 2.5, 3.01, 3.5])
    est_idx, gt_idx = associate(t_est, t_gt, max_diff=0.02)
    # t_est=2.0 has no gt within 0.02s (nearest is 2.5, off by 0.5) -> dropped
    assert list(est_idx) == [0, 1, 3]
    assert list(gt_idx) == [0, 1, 3]


def test_associate_empty_when_no_overlap():
    t_est = np.array([0.0, 1.0])
    t_gt = np.array([100.0, 101.0])
    est_idx, gt_idx = associate(t_est, t_gt, max_diff=0.02)
    assert len(est_idx) == 0
    assert len(gt_idx) == 0


def test_axis_ranges_matches_known_extents():
    points = np.array([
        [-2.0, 1.0, 10.0],
        [3.0, -1.0, 10.5],
        [0.0, 0.5, 9.8],
    ])
    ranges = _axis_ranges(points)
    assert ranges == pytest.approx([5.0, 2.0, 0.7])


def test_axis_ranges_combines_multiple_arrays():
    a = np.array([[0.0, 0.0, 0.0]])
    b = np.array([[1.0, -2.0, 0.5]])
    ranges = _axis_ranges(a, b)
    assert ranges == pytest.approx([1.0, 2.0, 0.5])


def test_axis_ranges_floors_degenerate_axis():
    # Constant Z (e.g. weave disabled) must not produce a zero range.
    points = np.array([[0.0, 0.0, 4.0], [1.0, 1.0, 4.0], [2.0, -1.0, 4.0]])
    ranges = _axis_ranges(points)
    assert ranges[2] > 0.0
    assert ranges[2] < 1e-3
