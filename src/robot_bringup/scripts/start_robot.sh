#!/usr/bin/env bash
# One-shot launcher for the Raspberry Pi: sources the three workspaces and
# starts the camera, QR scanner and lidar.
#
#   ~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_robot.sh
#   ros2 launch robot_bringup robot_bringup.launch.py   # equivalent

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /home/rsi-bd-11/lidar_ros_ws/install/setup.bash
source /home/rsi-bd-11/camera_ws/install3/setup.bash
set -u

exec ros2 launch robot_bringup robot_bringup.launch.py "$@"
