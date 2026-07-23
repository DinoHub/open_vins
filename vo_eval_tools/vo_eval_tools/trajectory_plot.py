"""
Plot the OpenVINS estimate against ground truth from a TUM-format pair.

Standalone usage:
    ros2 run vo_eval_tools plot_evaluation --bag <bag_dir> [--out <path>]

Loads gt.txt/est.txt (writing them via bag_to_tum.convert() first if they
don't already exist), time-associates the two trajectories, aligns the
estimate onto ground truth with the same rigid SE3 fit ov_eval's
error_singlerun uses (Umeyama, no scale - see align_se3() docstring), and
saves a 4-panel figure: 3D trajectory, top-down XY trajectory, altitude
over time, and position error over time.
"""

import argparse
import os

import matplotlib

# Must precede the pyplot import: no display needed, we only ever save PNGs.
matplotlib.use('Agg')  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load_tum(path):
    """Load a TUM-format trajectory file: returns (times, positions Nx3, quats Nx4 xyzw)."""
    times, positions, quats = [], [], []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            times.append(float(parts[0]))
            positions.append([float(v) for v in parts[1:4]])
            quats.append([float(v) for v in parts[4:8]])
    return np.array(times), np.array(positions), np.array(quats)


def associate(t_est, t_gt, max_diff=0.02):
    """
    Nearest-neighbor time-match two sorted, monotonically increasing timestamp arrays.

    Mirrors ov_eval's AlignUtils::perform_association 20ms window so the
    plotted comparison uses the same pairing as the reported ATE/RPE numbers.
    Returns (est_indices, gt_indices) of matched pairs.
    """
    est_idx, gt_idx = [], []
    j = 0
    n_gt = len(t_gt)
    for i, te in enumerate(t_est):
        while j < n_gt - 1 and abs(t_gt[j + 1] - te) <= abs(t_gt[j] - te):
            j += 1
        if abs(t_gt[j] - te) <= max_diff:
            est_idx.append(i)
            gt_idx.append(j)
    return np.array(est_idx, dtype=int), np.array(gt_idx, dtype=int)


def align_se3(source_points, target_points):
    """
    Umeyama rigid alignment (rotation + translation, no scale).

    Finds R, t minimizing sum ||target_i - (R @ source_i + t)||^2, i.e. the
    same SE3 fit ov_eval's error_singlerun applies (Umeyama 1991, no scale
    solved since stereo VIO has observable metric scale - see run_evaluation
    default align mode discussion). source_points/target_points are Nx3.
    """
    if source_points.shape[0] < 3:
        raise ValueError('Need at least 3 matched points to align')

    mu_src = source_points.mean(axis=0)
    mu_tgt = target_points.mean(axis=0)
    src_c = source_points - mu_src
    tgt_c = target_points - mu_tgt

    n = source_points.shape[0]
    sigma = (tgt_c.T @ src_c) / n
    u, _d, vt = np.linalg.svd(sigma)
    sign = np.sign(np.linalg.det(u) * np.linalg.det(vt))
    s = np.diag([1.0, 1.0, sign])
    r = u @ s @ vt
    t = mu_tgt - r @ mu_src
    return r, t


def _axis_ranges(*point_arrays):
    """
    Per-axis (x,y,z) extents across one or more Nx3 point arrays.

    Floored to a small positive value so a perfectly flat axis (e.g. weave
    disabled, Z constant) doesn't produce a degenerate zero-size box aspect.
    """
    all_points = np.concatenate(point_arrays, axis=0)
    ranges = all_points.max(axis=0) - all_points.min(axis=0)
    return np.where(ranges < 1e-6, 1e-6, ranges)


def _set_3d_equal_aspect(ax, *point_arrays):
    """
    Undistorted 3D aspect ratio: box proportions match the real data extents per axis.

    Without this, matplotlib's default independent-per-axis autoscaling would
    visually exaggerate a small vertical weave relative to the much larger
    horizontal extent, making it look far more dramatic than it actually is.
    """
    ax.set_box_aspect(_axis_ranges(*point_arrays))


def plot(gt_path, est_path, out_path, align_mode='se3', title=None, max_time_diff=0.02):
    t_gt, p_gt, _q_gt = load_tum(gt_path)
    t_est, p_est, _q_est = load_tum(est_path)

    est_idx, gt_idx = associate(t_est, t_gt, max_diff=max_time_diff)
    if len(est_idx) < 3:
        raise RuntimeError(
            f'Only {len(est_idx)} matched poses within {max_time_diff}s - too few to align/plot')

    matched_est = p_est[est_idx]
    matched_gt = p_gt[gt_idx]

    if align_mode == 'se3':
        r, t = align_se3(matched_est, matched_gt)
    elif align_mode == 'none':
        r, t = np.eye(3), np.zeros(3)
    else:
        raise ValueError(f"align_mode must be 'se3' or 'none' for plotting, got {align_mode!r}")

    est_aligned_full = (r @ p_est.T).T + t
    est_aligned_matched = (r @ matched_est.T).T + t
    errors = np.linalg.norm(est_aligned_matched - matched_gt, axis=1)
    err_times = t_est[est_idx] - t_est[0]
    rmse = float(np.sqrt(np.mean(errors**2)))

    fig = plt.figure(figsize=(14, 11))

    ax = fig.add_subplot(2, 2, 1, projection='3d')
    ax.plot(p_gt[:, 0], p_gt[:, 1], p_gt[:, 2], label='Ground Truth', linewidth=2, color='black')
    ax.plot(est_aligned_full[:, 0], est_aligned_full[:, 1], est_aligned_full[:, 2],
            label='OpenVINS (aligned)', linewidth=1, alpha=0.8, color='tab:orange')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D trajectory')
    ax.legend()
    _set_3d_equal_aspect(ax, p_gt, est_aligned_full)

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(p_gt[:, 0], p_gt[:, 1], label='Ground Truth', linewidth=2, color='black')
    ax.plot(est_aligned_full[:, 0], est_aligned_full[:, 1],
            label='OpenVINS (aligned)', linewidth=1, alpha=0.8, color='tab:orange')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Top-down trajectory (XY)')
    ax.axis('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t_gt - t_gt[0], p_gt[:, 2], label='Ground Truth', linewidth=2, color='black')
    ax.plot(t_est - t_est[0], est_aligned_full[:, 2],
            label='OpenVINS (aligned)', linewidth=1, alpha=0.8, color='tab:orange')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude / Z (m)')
    ax.set_title('Altitude over time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(err_times, errors, color='crimson', linewidth=1)
    ax.axhline(rmse, color='gray', linestyle='--', label=f'RMSE = {rmse:.3f} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position error (m)')
    ax.set_title('Position error over time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    if title:
        fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Path to the recorded rosbag directory')
    parser.add_argument(
        '--align-mode', default='se3', choices=('se3', 'none'),
        help='Alignment applied before plotting (default: se3)')
    parser.add_argument(
        '--out', default=None,
        help='Output PNG path (default: <bag_dir>/trajectory_plot.png)')
    parsed = parser.parse_args(args=args)

    gt_path = os.path.join(parsed.bag, 'gt.txt')
    est_path = os.path.join(parsed.bag, 'est.txt')
    if not (os.path.isfile(gt_path) and os.path.isfile(est_path)):
        from vo_eval_tools.bag_to_tum import convert
        est_path, gt_path = convert(parsed.bag, parsed.bag)

    out_path = parsed.out or os.path.join(parsed.bag, 'trajectory_plot.png')
    plot(gt_path, est_path, out_path, align_mode=parsed.align_mode)
    print(f'Saved trajectory plot to: {out_path}')


if __name__ == '__main__':
    main()
