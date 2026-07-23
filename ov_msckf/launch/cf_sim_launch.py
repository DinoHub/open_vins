import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ov_msckf')
    ros_params = os.path.join(pkg_share, 'config', 'cf', 'cf_stereo_ros.yaml')
    stereo_config = os.path.join(pkg_share, 'config', 'cf', 'cf_stereo.yaml')

    return LaunchDescription([
        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            name='run_subscribe_msckf',
            output='screen',
            arguments=[stereo_config],   # <-- passes as argv[1]
            parameters=[
                ros_params,
                {
                    'config_path': stereo_config,
                    'verbosity': 'DEBUG',
                    'use_sim_time': True,
                    'camera_topic_0': '/px4vision_custom_0/zedx_left/image',
                    'camera_topic_1': '/px4vision_custom_0/zedx_right/image',
                    'imu_topic': '/px4vision_custom_0/mavros/imu/data',
                }
            ]
        )
    ])