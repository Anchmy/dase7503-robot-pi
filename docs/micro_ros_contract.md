# micro-ROS base integration contract

What the ESP32 firmware must provide for the Raspberry Pi stack (Nav2 +
`game_manager`) to drive the robot.  Everything here is testable without
the lift or the camera.

## Topics

| Direction | Topic | Type | Rate | Notes |
| --- | --- | --- | --- | --- |
| Pi -> ESP32 | `/cmd_vel` | `geometry_msgs/Twist` | up to 20 Hz | `linear.x` m/s, `angular.z` rad/s |
| ESP32 -> Pi | `/odom` | `nav_msgs/Odometry` | >= 30 Hz | wheel-integrated pose |
| Pi -> ESP32 | `/lift_cmd` | `std_msgs/String` | on change | `"up"` / `"down"` |
| ESP32 -> Pi | `/lift_state` | `std_msgs/String` | ~5 Hz | optional: `"up"`, `"down"`, `"moving"` |

`/cmd_vel` is published by Nav2 through `velocity_smoother` and
`collision_monitor`; the firmware does not need to know about those.

## Frames and TF

The Pi stack uses `map -> odom -> base_link -> {lidar, camera}`.

* `/odom` must have `header.frame_id: odom` and
  `child_frame_id: base_link`.
* The ESP32 does **not** have to broadcast TF.  On the real robot start
  `robot_bringup` with `publish_odom_tf:=true`, which turns `/odom` into
  `odom -> base_link` (`robot_bringup/odom_to_tf.py`).
* Do **not** run `fake_base` and `odom_to_tf` at the same time: both would
  broadcast the same transform.
* If the firmware does broadcast `odom -> base_link` itself, leave
  `publish_odom_tf:=false`.

## Time

`tf2` rejects stamps that are far from the Pi clock.  Symptoms are
`Lookup would require extrapolation into the future` and Nav2 goals that
abort immediately.

* Call `rmw_uros_sync_session(1000)` after the agent connection is up.
* Stamp every `/odom` message with `rmw_uros_epoch_nanos()`.
* If the stamp is left at zero, `odom_to_tf` falls back to the Pi clock,
  but the odometry then has no timing information and must not be trusted
  for navigation.

## `/cmd_vel` behaviour

* Units: metres per second (positive forward) and radians per second
  (positive counter-clockwise).  `linear.y` is always 0.
* Our `velocity_smoother` caps commands at 0.25 m/s and 1.0 rad/s.
* Treat `|linear.x| < 0.01` and `|angular.z| < 0.01` as stop.
* **Safety timeout:** if no `/cmd_vel` arrives for 0.5 s, stop both motors.
  Nav2 publishes zeros when it stops, but the firmware must not depend on
  the Pi being alive.
* Comfortable limits for the chassis are about 0.5 m/s^2 linear and
  1.5 rad/s^2 angular; anything faster makes DWB overshoot the goal.

## Odometry quality

* Integrate both wheel encoders; publish `x`, `y`, `yaw` in `odom`.
* Fill the pose covariance diagonal with small values
  (e.g. 0.002, 0.002, 0.0, 0.0, 0.0, 0.01); AMCL uses it.
* A 5 % scale error is fine: AMCL corrects it against the lidar.  A wrong
  sign or a yaw drift of several degrees per metre is not.
* Publish at 30 Hz or more.  Below ~15 Hz the costmap TF lookups start
  failing on turns.

## Lift

* `/lift_cmd` carries `"up"` and `"down"` strings.
* `game_manager` waits `lift_time` seconds after each command (1.5 s in
  `game_params.yaml`).  If the real lift takes longer, raise that parameter;
  otherwise the robot drives away while the rack is still moving.
* The rack must be clear of the floor before the robot starts driving to the
  destination.

## Startup order on the robot

1. Power the ESP32; start the micro-ROS agent on the Pi
   (`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200`).
2. `ros2 launch robot_bringup robot_bringup.launch.py publish_odom_tf:=true`
3. `ros2 launch robot_navigation navigation.launch.py`
4. `ros2 launch game_manager game_manager.launch.py`

## Acceptance tests

```bash
# 1. odometry exists and is fast enough
ros2 topic hz /odom                         # expect >= 30 Hz

# 2. TF chain is complete
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link     # after Nav2 is up

# 3. signs: +x must move the robot forward, +z must turn it left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
# odom x should grow by ~0.10 m per second

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.30}}"
# odom yaw should grow by ~0.30 rad per second

# 4. safety timeout: stop publishing, the robot must stop within 0.5 s
```

Then drive one Nav2 goal in an open area and compare `/odom` with a tape
measure: the final error should be a few centimetres, not tens of
centimetres.

## Known pitfalls

* The course's reference firmware publishes `odom` with
  `frame_id: odom_frame` and `child_frame_id: base_footprint`.  Either
  rename them in the firmware or rely on `odom_to_tf` (which ignores the
  message frame names).
* A micro-ROS session that drops (WiFi or USB) must stop the motors; the
  agent reconnects automatically, but the firmware keeps its last command
  unless it implements the 0.5 s timeout.
* Keep the `/cmd_vel` callback short.  Doing PID work inside the executor
  callback starves the micro-ROS spin loop and adds latency to every other
  topic.
