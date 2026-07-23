"""
Orchestrate: bag -> TUM conversion -> ov_eval error_singlerun -> summary.

Usage:
    ros2 run vo_eval_tools run_evaluation --bag <bag_dir> [--align-mode se3]

Wraps vo_eval_tools.bag_to_tum.convert() and then invokes OpenVINS' own
`ov_eval error_singlerun` tool (already built in this workspace) rather than
reimplementing ATE/RPE alignment. Argument order to error_singlerun is
<align_mode> <groundtruth_file> <estimate_file> - verified directly against
error_singlerun.cpp's usage string and ResultTrajectory constructor call,
since the package's own doc example has estimate/groundtruth mislabeled.
"""

import argparse
import os
import re
import subprocess
import sys

from vo_eval_tools.bag_to_tum import convert
from vo_eval_tools.trajectory_plot import plot as plot_trajectories

VALID_ALIGN_MODES = ('posyaw', 'posyawsingle', 'se3', 'se3single', 'sim3', 'none')

# Exact format strings confirmed from ov_eval/src/error_singlerun.cpp.
_RE_ATE = re.compile(r'rmse_ori\s*=\s*([\d.]+)\s*\|\s*rmse_pos\s*=\s*([\d.]+)')
_RE_RPE_SEG = re.compile(
    r'seg\s+(\d+)\s*-\s*median_ori\s*=\s*([\d.]+)\s*\|\s*median_pos\s*=\s*([\d.]+)'
    r'\s*\((\d+)\s*samples\)')


def _parse_summary(stdout_text):
    summary_lines = []

    ate_match = _RE_ATE.search(stdout_text)
    if ate_match:
        rmse_ori, rmse_pos = ate_match.groups()
        summary_lines.append(f'ATE: rmse_ori={rmse_ori} deg, rmse_pos={rmse_pos} m')
    else:
        summary_lines.append('ATE: could not parse rmse line from error_singlerun output')

    rpe_matches = _RE_RPE_SEG.findall(stdout_text)
    if rpe_matches:
        for seg, median_ori, median_pos, samples in rpe_matches:
            summary_lines.append(
                f'RPE @ {seg}m: median_ori={median_ori} deg, '
                f'median_pos={median_pos} m ({samples} samples)')
        zero_sample_segs = [seg for seg, _, _, samples in rpe_matches if int(samples) == 0]
        if zero_sample_segs:
            summary_lines.append(
                f'  NOTE: 0 samples at segment length(s) {zero_sample_segs} m - '
                f'trajectory may not accumulate enough path length at that scale')
    else:
        summary_lines.append('RPE: could not parse segment lines from error_singlerun output')

    return '\n'.join(summary_lines)


def run_evaluation(bag_dir, align_mode='se3', out_dir=None, save_results=True, make_plot=True):
    if align_mode not in VALID_ALIGN_MODES:
        raise ValueError(f'align_mode must be one of {VALID_ALIGN_MODES}, got {align_mode!r}')

    out_dir = out_dir or bag_dir
    est_path, gt_path = convert(bag_dir, out_dir)

    cmd = ['ros2', 'run', 'ov_eval', 'error_singlerun', align_mode, gt_path, est_path]
    print(f"\nRunning: {' '.join(cmd)}\n")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "Could not run 'ros2 run ov_eval error_singlerun' - is the workspace sourced "
            '(source install/setup.bash) and was ov_eval built?') from exc

    print(proc.stdout)
    if proc.stderr:
        print(proc.stderr, file=sys.stderr)

    if proc.returncode != 0:
        raise RuntimeError(
            f'error_singlerun exited with code {proc.returncode} - see output above '
            f'(a common cause is too few time-associated pose pairs between est/gt)')

    summary = _parse_summary(proc.stdout)
    print('\n--- Summary ---')
    print(summary)

    if save_results:
        results_path = os.path.join(out_dir, 'results.txt')
        with open(results_path, 'w') as f:
            f.write(f'bag_dir: {bag_dir}\n')
            f.write(f'align_mode: {align_mode}\n\n')
            f.write(proc.stdout)
            f.write('\n--- Summary ---\n')
            f.write(summary)
            f.write('\n')
        print(f'\nFull results saved to: {results_path}')

    if make_plot:
        if align_mode in ('se3', 'none'):
            plot_path = os.path.join(out_dir, 'trajectory_plot.png')
            try:
                plot_trajectories(gt_path, est_path, plot_path, align_mode=align_mode)
                print(f'Saved trajectory plot to: {plot_path}')
            except RuntimeError as exc:
                print(f'WARNING: could not generate trajectory plot: {exc}')
        else:
            print(
                f"NOTE: skipping plot - align_mode={align_mode!r} isn't supported by the "
                f"plotter (only 'se3'/'none'); the ATE/RPE numbers above still used your "
                f'requested align_mode correctly.')

    return summary


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Path to the recorded rosbag directory')
    parser.add_argument(
        '--align-mode', default='se3', choices=VALID_ALIGN_MODES,
        help='Trajectory alignment method passed to ov_eval error_singlerun (default: se3)')
    parser.add_argument(
        '--out-dir', default=None,
        help='Output directory for gt.txt/est.txt/results.txt/trajectory_plot.png '
             '(default: same as --bag)')
    parser.add_argument(
        '--no-plot', action='store_true',
        help='Skip generating trajectory_plot.png')
    parsed = parser.parse_args(args=args)

    try:
        run_evaluation(
            parsed.bag, align_mode=parsed.align_mode, out_dir=parsed.out_dir,
            make_plot=not parsed.no_plot)
    except (RuntimeError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
