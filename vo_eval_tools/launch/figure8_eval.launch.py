"""
Launch the figure-8 offboard controller plus a bag recording of it vs ground truth.

Assumes the Gazebo/PX4 sim stack, mavros, and OpenVINS (cf_sim_launch.py)
are already running - this launch file only adds the flight controller and
the recorder on top of that existing stack.
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_runs_root():
    """Default eval-run output directory: <ros2_ws>/vo_eval_runs.

    <ros2_ws> is the workspace actually running this launch file, derived from
    COLCON_PREFIX_PATH (set by `source install/setup.bash`) rather than
    hardcoded - so this keeps working no matter where the workspace lives.
    Override with `runs_root:=/some/path` to place runs anywhere else.
    """
    colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if not colcon_prefix:
        raise RuntimeError(
            'COLCON_PREFIX_PATH is not set - did you `source install/setup.bash`?')
    ws_root = os.path.dirname(colcon_prefix.rstrip('/'))
    return os.path.join(ws_root, 'vo_eval_runs')


def _launch_setup(context, *args, **kwargs):
    runs_root = LaunchConfiguration('runs_root').perform(context)
    config = LaunchConfiguration('config').perform(context)

    os.makedirs(runs_root, exist_ok=True)
    bag_dir = os.path.join(runs_root, datetime.now().strftime('%Y%m%d_%H%M%S'))

    controller_node = Node(
        package='vo_eval_tools',
        executable='figure8_controller',
        name='figure8_offboard_controller',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
    )

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir, '-s', 'mcap',
             '/odomimu', '/px4vision_custom_0/odometry'],
        output='screen',
    )

    return [
        LogInfo(msg=['Recording evaluation bag to: ', bag_dir]),
        controller_node,
        bag_record,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('vo_eval_tools')
    default_config = os.path.join(pkg_share, 'config', 'figure8_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'runs_root', default_value=_default_runs_root(),
            description='Parent directory under which each run gets a timestamped folder'),
        DeclareLaunchArgument('config', default_value=default_config),
        OpaqueFunction(function=_launch_setup),
    ])
