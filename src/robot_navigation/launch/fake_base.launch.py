"""Start the desk-test fake base.

    ros2 launch robot_navigation fake_base.launch.py
    ros2 launch robot_navigation fake_base.launch.py \
        initial_x:=2.72 initial_y:=1.00 initial_yaw:=3.1416

Publishes /odom and TF odom -> base_link from integrated /cmd_vel, so Nav2 can
be tested without the chassis.  Do not run this on the real robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('initial_x', default_value='2.72'),
        DeclareLaunchArgument('initial_y', default_value='1.00'),
        DeclareLaunchArgument('initial_yaw', default_value='3.1416'),
        Node(
            package='robot_navigation',
            executable='fake_base.py',
            name='fake_base',
            output='screen',
            parameters=[{
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_yaw': initial_yaw,
            }],
        ),
    ])
