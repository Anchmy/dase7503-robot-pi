"""Nav2 on the saved field map, with AMCL localisation.

Usage:
    ros2 launch robot_navigation navigation.launch.py
    ros2 launch robot_navigation navigation.launch.py map:=/path/to/field.yaml

    # On a desk, with the real lidar looking at a room that is not the field:
    ros2 launch robot_navigation navigation.launch.py desk_mode:=true

    # With the RViz window (needs a display: run it on the Pi's desktop, or
    # over SSH with WAYLAND_DISPLAY set):
    ros2 launch robot_navigation navigation.launch.py rviz:=true

The robot base must publish /odom and TF odom -> base_link (micro-ROS bridge),
or run fake_base.launch.py as a stand-in.

desk_mode writes a copy of nav2_params.yaml with the live lidar obstacle
layers and the collision monitor's scan source disabled, and with AMCL's
scan updates frozen, so the room around the desk neither blocks paths nor
drags the pose estimate away from the field map.  The production
configuration in config/nav2_params.yaml is left untouched.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_params_file(params_file, desk_mode):
    """Return the params file to use, writing a desk variant when asked."""
    if not desk_mode:
        return params_file

    with open(params_file, 'r', encoding='utf-8') as handle:
        params = yaml.safe_load(handle)

    for namespace in ('global_costmap', 'local_costmap'):
        params[namespace][namespace]['ros__parameters']['obstacle_layer'][
            'enabled'] = False
    # observation_sources must stay non-empty (an empty list makes the node
    # throw "No parameter value set"), so disable the source instead.
    params['collision_monitor']['ros__parameters']['scan']['enabled'] = False

    # Freeze AMCL: on a desk the real lidar sees a room, not the field, so
    # scan matching would drag the pose estimate away from the initial pose
    # and the robot would follow a path in the wrong direction.  With the
    # update thresholds unreachable, map -> odom stays at the start pose.
    amcl = params['amcl']['ros__parameters']
    amcl['update_min_d'] = 1.0e9
    amcl['update_min_a'] = 1.0e9

    desk_file = os.path.join(tempfile.gettempdir(), 'nav2_params_desk.yaml')
    with open(desk_file, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(params, handle, sort_keys=False)
    return desk_file


def _launch_setup(context):
    pkg_share = get_package_share_directory('robot_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'field.yaml')

    map_yaml = LaunchConfiguration('map').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    desk_mode = LaunchConfiguration('desk_mode').perform(context).lower() in (
        'true', '1', 'yes')
    rviz = LaunchConfiguration('rviz').perform(context).lower() in (
        'true', '1', 'yes')

    params_file = _make_params_file(params_file, desk_mode)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': 'False',
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': 'False',
            }.items(),
        ),
    ]

    if rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'field.rviz')],
            output='screen',
        ))

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    default_map = os.path.join(pkg_share, 'maps', 'field.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'desk_mode', default_value='false',
            description='ignore live lidar obstacles (desk testing)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='also start RViz with rviz/field.rviz'),
        OpaqueFunction(function=_launch_setup),
    ])
