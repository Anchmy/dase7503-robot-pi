"""Start the game state machine.

Usage:
    ros2 launch game_manager game_manager.launch.py
    ros2 launch game_manager game_manager.launch.py use_nav2:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_nav2 = LaunchConfiguration('use_nav2')
    params_file = PathJoinSubstitution([
        FindPackageShare('game_manager'), 'config', 'game_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        Node(
            package='game_manager',
            executable='game_manager',
            name='game_manager',
            output='screen',
            parameters=[params_file, {'use_nav2': use_nav2}],
        ),
    ])
