"""Start every sensor node on the Raspberry Pi with one command.

    ros2 launch robot_bringup robot_bringup.launch.py

Brings up:
  * Camera Module 3 through camera_ros        -> /camera/image_raw
  * QR scanner                                -> /qr_detected
  * MS200 lidar through oradar_lidar          -> /scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_qr_launch = os.path.join(
        get_package_share_directory('qr_scanner'), 'launch',
        'camera_qr.launch.py')
    lidar_launch = os.path.join(
        get_package_share_directory('oradar_lidar'), 'launch',
        'ms200_scan.launch.py')

    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    decode_rate = LaunchConfiguration('decode_rate')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_yaw = LaunchConfiguration('camera_yaw')
    camera_pitch = LaunchConfiguration('camera_pitch')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('decode_rate', default_value='3.0'),

        # Camera mounting on base_link: metres and radians.
        # The QR plate on each rack sits 232.5 mm above the field floor and the
        # robot may be at most 180 mm tall, so the camera mounts around
        # 170 mm and tilts up a few degrees.  Update to the real mount.
        DeclareLaunchArgument('camera_x', default_value='0.12'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.17'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='-0.10'),

        # The chassis firmware publishes /odom but not TF.  Enable this on the
        # real robot; keep it off when fake_base is running, because both
        # nodes would broadcast odom -> base_link.
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[camera_x, camera_y, camera_z,
                       camera_yaw, camera_pitch, '0.0',
                       'base_link', 'camera'],
        ),

        Node(
            package='robot_bringup',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
            condition=IfCondition(publish_odom_tf),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_qr_launch),
            launch_arguments={
                'width': width,
                'height': height,
                'decode_rate': decode_rate,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
        ),

        # The vendor driver never sends SET_RUN_MODE when it starts without
        # data, so an MS200 that powered up stopped stays silent forever.
        # The watchdog re-sends the run command whenever /scan dries up.
        Node(
            package='robot_bringup',
            executable='lidar_watchdog.py',
            name='lidar_watchdog',
            output='screen',
        ),
    ])
