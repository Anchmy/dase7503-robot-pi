"""Online SLAM with slam_toolbox (needs odom -> base_link from the base).

Usage:
    ros2 launch robot_navigation slam.launch.py
    ros2 launch robot_navigation slam.launch.py \\
        params_file:=/path/to/slam_toolbox.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('robot_navigation'), 'config',
        'slam_toolbox.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
        ),
    ])
