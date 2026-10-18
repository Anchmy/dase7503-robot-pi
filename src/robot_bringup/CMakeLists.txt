cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/start_robot.sh scripts/start_match.sh scripts/odom_to_tf.py
        scripts/lidar_watchdog.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
