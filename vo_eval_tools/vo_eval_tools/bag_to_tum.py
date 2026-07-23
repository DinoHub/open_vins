"""
Convert a recorded rosbag's OpenVINS and ground-truth topics to TUM-format text files.

No ROS2 equivalent of this conversion exists in the vendored ov_eval package
(its ROS1 pose_to_file tool is explicitly disabled in ov_eval's ROS2 cmake),
so this is new code, written from scratch for this purpose.

Output format, one pose per line (verified against ov_eval's Loader.cpp):
    # timestamp(s) tx ty tz qx qy qz qw
    <sec+nanosec*1e-9> <pos.x> <pos.y> <pos.z> <quat.x> <quat.y> <quat.z> <quat.w>
Quaternion is x,y,z,w order (not w-first). Timestamps in seconds as floats,
taken from msg.header.stamp - never from the bag's own receive-time - so the
recorded trajectory is timestamped by when the estimator/simulator produced
the measurement, not when the recorder happened to receive it.
"""

import argparse
import os
import sys

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
import rosbag2_py
import yaml

EST_TOPIC = '/odomimu'
GT_TOPIC = '/px4vision_custom_0/odometry'

EST_FILENAME = 'est.txt'
GT_FILENAME = 'gt.txt'

# Nominal publish rates, used only for a sanity-check warning, not for any
# correctness-critical computation.
EST_NOMINAL_HZ = 15.0
GT_NOMINAL_HZ = 50.0


def _detect_storage_id(bag_dir):
    metadata_path = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.isfile(metadata_path):
        with open(metadata_path, 'r') as f:
            metadata = yaml.safe_load(f)
        try:
            return metadata['rosbag2_bagfile_information']['storage_identifier']
        except (KeyError, TypeError):
            pass
    return 'mcap'


def _stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _read_topic_lines(bag_dir, storage_id):
    """Read the bag once, return {topic: [(t_seconds, line_str), ...]}."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    available_topics = {t.name for t in reader.get_all_topics_and_types()}
    for topic in (EST_TOPIC, GT_TOPIC):
        if topic not in available_topics:
            raise RuntimeError(
                f"Topic '{topic}' not found in bag '{bag_dir}'. "
                f'Available topics: {sorted(available_topics)}')

    reader.set_filter(rosbag2_py.StorageFilter(topics=[EST_TOPIC, GT_TOPIC]))

    results = {EST_TOPIC: [], GT_TOPIC: []}
    while reader.has_next():
        topic, data, _recv_time_ns = reader.read_next()
        msg = deserialize_message(data, Odometry)
        t = _stamp_to_seconds(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        line = f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n'
        results[topic].append((t, line))

    return results


def convert(bag_dir, out_dir, storage_id=None):
    """
    Convert bag_dir's /odomimu and ground-truth topics to TUM txt files.

    Returns (est_path, gt_path).
    """
    if storage_id is None:
        storage_id = _detect_storage_id(bag_dir)

    results = _read_topic_lines(bag_dir, storage_id)

    os.makedirs(out_dir, exist_ok=True)
    est_path = os.path.join(out_dir, EST_FILENAME)
    gt_path = os.path.join(out_dir, GT_FILENAME)

    for topic, path, nominal_hz in (
        (EST_TOPIC, est_path, EST_NOMINAL_HZ),
        (GT_TOPIC, gt_path, GT_NOMINAL_HZ),
    ):
        entries = sorted(results[topic], key=lambda e: e[0])
        with open(path, 'w') as f:
            f.write('# timestamp(s) tx ty tz qx qy qz qw\n')
            f.writelines(line for _, line in entries)
        _report_topic(topic, entries, nominal_hz)

    _report_overlap(results[EST_TOPIC], results[GT_TOPIC])

    return est_path, gt_path


def _report_topic(topic, entries, nominal_hz):
    n = len(entries)
    if n == 0:
        print(f'WARNING: topic {topic} has 0 messages in the bag')
        return
    t0, t1 = entries[0][0], entries[-1][0]
    duration = t1 - t0
    rate = (n - 1) / duration if duration > 0 else 0.0
    print(f'{topic}: {n} messages, {duration:.2f}s span, '
          f'~{rate:.1f} Hz (nominal ~{nominal_hz:.0f} Hz)')
    if duration > 0 and abs(rate - nominal_hz) / nominal_hz > 0.25:
        print('  WARNING: observed rate deviates >25% from nominal - possible dropped messages')


def _report_overlap(est_entries, gt_entries):
    if not est_entries or not gt_entries:
        return
    est_t0, est_t1 = est_entries[0][0], est_entries[-1][0]
    gt_t0, gt_t1 = gt_entries[0][0], gt_entries[-1][0]
    overlap_start = max(est_t0, gt_t0)
    overlap_end = min(est_t1, gt_t1)
    overlap = max(0.0, overlap_end - overlap_start)
    print(f'Time overlap between {EST_TOPIC} and {GT_TOPIC}: {overlap:.2f}s '
          f'(est span {est_t1 - est_t0:.2f}s, gt span {gt_t1 - gt_t0:.2f}s)')
    if overlap < 0.5 * min(est_t1 - est_t0, gt_t1 - gt_t0):
        print('  WARNING: small time overlap relative to trajectory length - '
              'check both topics were recorded on the same (sim) clock source, '
              "this will starve ov_eval's 20ms association window")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, help='Path to the recorded rosbag directory')
    parser.add_argument(
        '--out-dir', default=None,
        help='Output directory for gt.txt/est.txt (default: same as --bag)')
    parser.add_argument(
        '--storage', default=None,
        help='rosbag2 storage id (default: auto-detect from metadata.yaml, else mcap)')
    parsed = parser.parse_args(args=args)

    out_dir = parsed.out_dir or parsed.bag
    try:
        est_path, gt_path = convert(parsed.bag, out_dir, storage_id=parsed.storage)
    except RuntimeError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)

    print(f'Wrote estimate -> {est_path}')
    print(f'Wrote groundtruth -> {gt_path}')


if __name__ == '__main__':
    main()
