"""Map the field with the lidar only: no wheel odometry required.

Cartographer tracks its own motion from the laser scans. Push the robot (or
carry the Pi + lidar) slowly around the field, then save the map with:

    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
        "{filename: '/home/rsi-bd-11/field_map.pbstream', \
          include_unfinished_submaps: true}"

and convert it to pgm/yaml with cartographer_pbstream_to_ros_map:

    ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
        -pbstream_filename=$HOME/field_map.pbstream \
        -map_filestem=$HOME/field_map -resolution=0.05

That writes field_map.pgm and field_map.yaml, which can be passed to
navigation.launch.py as map:=... once the poses have been checked.

Usage:
    ros2 launch robot_navigation cartographer_mapping.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    config_dir = os.path.join(pkg_share, 'config')

    resolution = LaunchConfiguration('resolution')
    rate = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='0.3'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_field.lua',
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', rate,
            ],
        ),
    ])
