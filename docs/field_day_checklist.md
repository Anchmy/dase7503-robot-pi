# Field-day checklist

What to do on the real field, in order, with the acceptance criteria that
say whether each step actually worked.  Everything below runs on the Pi
(`rsi-bd-11`); commands assume both workspaces are sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install3/setup.bash
source ~/lidar_ros_ws/install/setup.bash
```

## 0. Before leaving the desk

* The desk rehearsal passes: 4 shipments, score 20, 97 s
  (`docs/rehearsal_2026-10-18.md`).
* Bring: tape measure (5 m), masking tape, a printed QR plate mounted at
  232.5 mm, the charger, a phone hotspot.
* Charge the Pi and check the SD card is not full (`df -h /`).

## 1. Start the sensors

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

With the chassis, use `publish_odom_tf:=true`; on a desk with `fake_base`,
leave it false (see `docs/micro_ros_contract.md`).

Acceptance:

```bash
ros2 topic hz /scan                # ~10 Hz
ros2 topic hz /camera/image_raw    # ~10 Hz (>= 5 Hz with Foxglove streaming)
ros2 node list | grep lidar_watchdog
ros2 run tf2_ros tf2_echo base_link camera
# translation (0.120, 0.000, 0.170), pitch -5.73 deg
```

`lidar_watchdog` re-sends the MS200 run command when `/scan` stops for 5 s
and logs `lidar stream recovered` when data returns. It cannot help when the
USB re-enumerates: `/dev/oradar` then points at a new `ttyACM*` while the
driver still holds the deleted one. Before blaming the lidar, compare
`readlink /dev/oradar` with `ls -l /proc/$(pgrep -f oradar_scan)/fd`; if they
differ, restart the stack. Repeated `over-current change` lines in `dmesg`
mean a cable or connector short, not software; a `USB disconnect` with no
over-current line is still a re-enumeration, so check the tty pair first
instead of trusting the absence of over-current.

## 2. Map the field with the lidar (before the chassis exists)

AMCL must match the real field against a map.  Cartographer needs no wheel
odometry, so it can run while the robot is pushed by hand.

```bash
ros2 launch robot_navigation cartographer_mapping.launch.py
```

Push the robot slowly (about 0.2 m/s) around the field: along all four
walls, then a loop past the racks, then back to the start zone.  Keep the
lidar between 0.3 and 1.5 m from the walls; do not spin the robot in place
faster than ~30 deg/s.

Save and convert:

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/rsi-bd-11/field_map.pbstream', include_unfinished_submaps: true}"

ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=$HOME/field_map.pbstream \
  -map_filestem=$HOME/field_map -resolution=0.05
```

Acceptance:

* `field_map.pgm` shows four straight walls and the rack footprints, with
  no doubled walls (doubled walls mean the scan matching slipped — redo the
  loop more slowly).
* Measured wall-to-wall size is 3020 x 2000 mm (60 x 40 cells at 0.05 m,
  plus a small margin).
* The map loads in Nav2 and the robot pose at the start zone
  (2.72, 1.00) is inside the free space of the map, facing the field.

**Stop Cartographer before starting Nav2.**  Both publish `/map`; a leftover
`cartographer_occupancy_grid_node` replaces the static field map and the
planner then fails from every pose:

```bash
pgrep -af cartographer               # must print nothing
ros2 topic info /map                 # Publisher count: 1 after Nav2 starts
```

## 3. Localisation on the field map

```bash
ros2 launch robot_navigation navigation.launch.py map:=$HOME/field_map.yaml
```

AMCL starts at the start-zone pose configured in `nav2_params.yaml`
(`initial_pose: 2.72, 1.00, yaw 3.1416`).  Check convergence in RViz
(`/amcl_pose` and the particle cloud) with the robot physically parked in
the start zone facing the field.

If the pose is wrong, either fix the map origin or set the initial pose by
hand; do not let `game_manager` start until `map -> base_link` looks right.

Acceptance:

```bash
ros2 run tf2_ros tf2_echo map base_link     # finite, within the field
```

## 4. Camera / QR plate geometry

The plate centre is 232.5 mm above the field floor; the camera optical
centre is at 170 mm with a 5.73 deg upward tilt.  At the 0.35 m approach
distance the plate is about 10 deg above horizontal, i.e. roughly 4 deg
above the optical axis — comfortably inside the frame.  Check it rather
than trust it:

1. Put a printed QR plate (`RACKA_…`) on a rack, centre exactly 232.5 mm up.
2. Park the robot at the approach pose (0.35 m from the rack face).
3. `ros2 topic echo /qr_detected` must print the payload within 1 s.
4. Repeat with the plate 50 mm higher and 50 mm lower: the payload must
   still decode, because the real rack and floor vary.

If the plate is cut off at the top of the frame, raise the camera or
increase the pitch in `robot_bringup.launch.py`; if it is cut off at the
bottom, lower the camera.  Re-check the TF afterwards.

Measured decode distance (fill in): ______ m.  Minimum reliable: ______ m.

## 5. Calibrate the rack poses

The poses in `game_manager/config/game_params.yaml` come from the drawing
(`tools/make_field_map.py`).  Measured rack centres (tape from the
bottom-left field corner):

| Rack | Drawing (x, y) | Measured (x, y) | Approach pose used |
| --- | --- | --- | --- |
| A | 1.275, 1.580 | | 1.275, 1.230, +1.5708 |
| B | 1.825, 0.510 | | 1.825, 0.860, -1.5708 |
| C | 1.925, 1.720 | | 1.925, 1.370, +1.5708 |
| D | 1.325, 0.200 | | 1.325, 0.550, -1.5708 |

Approach poses are 0.35 m in front of the rack face, facing it.  Move them
closer only after measuring the lift's reach; the QR is cached for 30 s
(`rack_qr_cache_time`), so the plate does not have to stay in view at the
final pose.

Acceptance: `ros2 run tf2_ros tf2_echo map base_link` at each rack approach
agrees with the tape measure within 50 mm.

## 6. Run the match stack

One command (sensors are skipped if they are already up):

```bash
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --map ~/field_map.yaml
```

With the chassis, add `--odom-tf` so `/odom` is broadcast as TF.  Add
`--rviz` to watch on the Pi's monitor, or `--foxglove` and connect the
Foxglove desktop app 3.x to `ws://<pi-ip>:8765` from a laptop (older Studio
builds speak only `foxglove.websocket.v1`, which bridge 3.x rejects).
Ctrl-C stops the whole stack.
Without the script:

```bash
ros2 launch game_manager game_manager.launch.py
```

Watch `/game_state`, `/game_score` and `/lift_cmd`.  On the field the START
QR is read from the plate; the supervisor is not used.

Acceptance for a full run:

* all four racks lifted and delivered, score 20;
* no planner or controller failure in the Nav2 log;
* elapsed time under 150 s (desk rehearsal: 97 s), leaving margin inside
  the 180 s match limit;
* robot stops on its own at the destination after END.

## 7. Time budget

| Phase | Desk | Field allowance |
| --- | --- | --- |
| sensors up | 15 s | 30 s |
| START -> rack A | 9.7 s | 15 s |
| rack -> destination | 9.1 s | 15 s |
| full mission | 97 s | 150 s |

If the field run exceeds 150 s, raise the speed limits in
`nav2_params.yaml` (`max_vel_x`, `max_speed_xy`) before touching the rack
order: the field is 3 m wide, so most of the time is driving.
