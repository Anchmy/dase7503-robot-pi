"""Bring up the Camera Module 3 and the QR scanner in one shot.

Usage:
    ros2 launch qr_scanner camera_qr.launch.py
    ros2 launch qr_scanner camera_qr.launch.py width:=800 height:=600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    decode_rate = LaunchConfiguration('decode_rate')

    return LaunchDescription([
        # 640x480 at 15 fps is plenty to read a QR code 0.35 m away and
        # leaves CPU for Nav2.  On the Pi 5 the camera at 720p/30 fps used
        # ~40% of a core on its own; 640x480/15 fps is roughly a quarter of
        # that.  FrameDurationLimits is [min_us, max_us] in microseconds.
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('decode_rate', default_value='3.0',
                              description='QR decode attempts per second'),

        # camera_ros names the node "camera" so the raw topic ends up as
        # /camera/image_raw, which qr_node subscribes to by default.
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[{
                'width': ParameterValue(width, value_type=int),
                'height': ParameterValue(height, value_type=int),
                'format': 'XRGB8888',
                'FrameDurationLimits': [66666, 66666],
            }],
            output='screen',
        ),
        Node(
            package='qr_scanner',
            executable='qr_node',
            name='qr_scanner',
            arguments=['--rate', decode_rate],
            output='screen',
        ),
    ])
