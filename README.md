# DASE7503 project — Raspberry Pi software stack

ROS 2 Jazzy software that runs on the robot's Raspberry Pi 5. It covers the
perception and mission layers: camera, QR decoding, lidar, and the game state
machine. The motor control / micro-ROS bridge lives in the chassis firmware,
not in this workspace.

## Hardware

| Item | Model | Notes |
| --- | --- | --- |
| Computer | Raspberry Pi 5, 8 GB | Ubuntu 24.04 (noble), aarch64 |
| Camera | Camera Module 3 (IMX708) | 640x480 @ 15 fps, XRGB8888 -> `bgra8` |
| Lidar | MS200 (OrADAR) | 360 deg, 10 Hz, `/dev/oradar`, 230400 baud |

Host name: `rsi-bd-11` (192.168.3.18).

## Packages

```
src/
  qr_scanner/        camera + QR decoding node, and a combined launch file
  robot_bringup/     one-command sensor bringup (camera + QR + lidar)
  game_manager/      match state machine (START -> racks -> destination -> END)
  robot_navigation/  SLAM and Nav2 configuration, maps
tools/
  make_field_map.py       generates the synthetic field map from the course drawing
  mission_supervisor.py   desk rehearsal: injects the QR payloads a field would give
docs/
  rehearsal_2026-10-18.md  first end-to-end run, timeline and limits
  field_day_checklist.md   mapping, camera/QR geometry, rack calibration, acceptance tests
  micro_ros_contract.md    what the ESP32 firmware must provide
```

`robot_bringup` is the entry point for a run: it starts everything from
`qr_scanner` and `oradar_lidar`, plus the static `base_link -> lidar` TF that
comes with the lidar launch file.

### qr_scanner

* `qr_node` subscribes `/camera/image_raw`, decodes QR codes and publishes the
  payload on `/qr_detected` (`std_msgs/String`).
* Decoding runs at most 3 Hz (`--rate`) and on frames downscaled to 640 px
  wide (`--max-width`). Together with 640x480 camera frames this cut camera
  CPU from ~40% to ~14% and QR CPU from ~52% to ~36%, which left enough
  headroom for Nav2 on the Pi 5.
* A payload that stays in view is re-published every 2 s
  (`--repeat-interval`). Without this the first sighting is published once
  and lost if the mission state machine is not listening yet — which is
  exactly what happens when the rack plate appears during the approach.
* OpenCV `QRCodeDetector` is tried first, pyzbar is the fallback.
* Launch: `ros2 launch qr_scanner camera_qr.launch.py`

### robot_bringup

One command starts camera, QR scanner and lidar:

```bash
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_robot.sh
# or, after sourcing the workspaces:
ros2 launch robot_bringup robot_bringup.launch.py
```

Topics: `/camera/image_raw`, `/qr_detected`, `/scan`, TF `base_link -> lidar`.

`start_match.sh` starts a whole run in one command (it sources all three
workspaces itself, skips sensors that are already running, and logs each
component to `~/dase7503/logs/`):

```bash
# desk rehearsal: sensors + fake_base + Nav2 desk_mode + game_manager
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh --desk

# hands-free desk rehearsal: also injects START / RACKx / END
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --desk --supervisor

# field: sensors + Nav2 on the mapped field + game_manager
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --map ~/field_map.yaml

# field with the chassis: also broadcast /odom as TF
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --map ~/field_map.yaml --odom-tf
```

Ctrl-C stops every component it started.

The bringup also starts `lidar_watchdog`. The vendor driver only sends
`SET_RUN_MODE` from `Activate()`, and `Activate()` is only reached after the
driver has already received scan data, so an MS200 that powers up stopped
stays silent forever. The watchdog watches `/scan`: after 5 s without scans
it sends the speed + run frames the SDK would send, repeats the burst every
20 s, and logs `lidar stream recovered` when data returns. It writes nothing
while scans are flowing.

The watchdog cannot recover a USB re-enumeration. If the adapter drops off
the bus — sometimes with `over-current change` lines for every root hub in
`dmesg`, sometimes with only a `USB disconnect` and no over-current at all —
`/dev/oradar` moves to a new `ttyACM*` while the running driver keeps the
deleted one open, so it never sees the new tty. Check:

```bash
readlink /dev/oradar
ls -l /proc/$(pgrep -f oradar_scan)/fd | grep ttyACM   # must match
```

If they differ, restart the stack; only a restart reopens the port.

### Visualization

The stack is headless by default. There are two ways to see what it is doing:

* **RViz on the Pi's own display**: add `--rviz` to `start_match.sh` (or
  `rviz:=true` to `navigation.launch.py`). It loads `rviz/field.rviz`:
  field map, global/local costmaps, laser scan, global/local plans, AMCL
  pose and the camera image. Started from SSH, the script attaches to the
  desktop session's Xwayland automatically, because RViz renders through
  Ogre/GLX and needs an X display even on a Wayland desktop.
* **Foxglove on another machine**: install the bridge once
  (`sudo apt install ros-jazzy-foxglove-bridge`), add `--foxglove` to
  `start_match.sh`, then connect the Foxglove desktop app (3.x) to
  `ws://<pi-ip>:8765` (the script prints the exact URL). The 3.6 bridge
  only accepts the `foxglove.sdk.v1` subprotocol; Foxglove Studio 2.9 and
  older (and Lichtblick) offer `foxglove.websocket.v1` and are rejected
  with "does not support protocol version foxglove.websocket.v1".

### game_manager

State machine that plays one match. It consumes QR payloads and Nav2 action
results only; it does not read sensors directly.

```
WAIT_START --START--> DRIVE_TO_RACK --arrived--> WAIT_RACK_QR
    --RACKx_xxxx--> LIFT_UP --lifted--> DRIVE_TO_DEST --arrived-->
    LIFT_DOWN --lowered--> next rack ... -> WAIT_END_QR --END--> DONE
```

Topics/actions:

* subscribes `/qr_detected`
* publishes `/game_state`, `/game_score`, `/lift_cmd` (`up` / `down`)
* action client on `navigate_to_pose` (Nav2)

Run it without Nav2, on a desk:

```bash
ros2 run game_manager game_manager --ros-args \
  -p use_nav2:=false -p dry_run_travel_time:=3.0
```

Inject QR events by hand:

```bash
ros2 topic pub --once /qr_detected std_msgs/msg/String "{data: 'START'}"
ros2 topic pub --once /qr_detected std_msgs/msg/String "{data: 'RACKA_4X6M'}"
```

All field poses are parameters in `game_manager/config/game_params.yaml`
(map frame, metres, radians). They were derived from the dimension chains in
the course drawing (rack centres plus 0.35 m approach offsets), so they only
need small corrections against the real field.

Field behaviour worth knowing:

* `destination_points` gives one drop point per delivery, 0.35-0.40 m apart.
  Dropping all racks on the same pose would put the next goal on top of the
  rack just delivered.
* A rack QR first seen on the approach is cached for 30 s
  (`rack_qr_cache_time`) and reused on arrival, so the robot does not need
  the plate in view once it is parked at the rack.
* Only the QR of the rack that was driven to is accepted
  (`strict_rack_match`); a neighbouring rack's plate can enter the frame.
* END is the destination's QR and is in view on every delivery, so a
  mid-mission END is ignored. `end_qr_stops_mission:=true` restores the old
  emergency-stop behaviour for desk tests.
* A failed navigation to a rack is retried once before the rack is skipped.

### robot_navigation

* `maps/field.pgm` + `maps/field.yaml` — synthetic occupancy map generated by
  `tools/make_field_map.py` from the drawing. Origin is the bottom-left corner
  of the field; no mapping run is required before using Nav2.
* `launch/cartographer_mapping.launch.py` — lidar-only mapping that needs no
  wheel odometry; use it while the chassis is unavailable or to verify the
  synthetic map.
* `launch/slam.launch.py` — slam_toolbox online mapping once `/odom` exists.
* `launch/navigation.launch.py` — Nav2 + AMCL on `maps/field.yaml`.
* `scripts/fake_base.py` + `launch/fake_base.launch.py` — desk-test stand-in
  for the chassis. It integrates `/cmd_vel` into `/odom` and broadcasts
  `odom -> base_link`, so Nav2 and `game_manager` can be exercised before the
  micro-ROS firmware exists. Never start it on the real robot.
* **Never run Cartographer and Nav2 together.** Both publish `/map`; a
  surviving `cartographer_occupancy_grid_node` replaces the synthetic field
  map and the planner fails from every pose. Check
  `ros2 topic info /map` shows exactly one publisher before starting Nav2.

Desk test without a chassis:

```bash
ros2 launch robot_navigation fake_base.launch.py   # /odom + TF
ros2 launch robot_navigation navigation.launch.py  # Nav2 on field.pgm
ros2 launch game_manager game_manager.launch.py    # mission, use_nav2=true
```

Always start `game_manager` through the launch file: `ros2 run` skips
`game_params.yaml` and falls back to the code defaults (start pose 2.6/0.4,
rack poses 0.8/1.6), which are not the field layout.

### qr_scanner crash guard

OpenCV 4.6 raises `cv2.error` from `QRCodeDetector.detectAndDecode` on some
low-texture frames. `decoder.py` isolates every decode attempt and `qr_node`
catches anything left, so one bad frame cannot kill the QR node.

## Game rules (from the course PDFs)

* Field 3020 x 2000 mm, walls 500 mm high; start zone 600 x 800 mm
  bottom-right; destination zone 800 x 1400 mm along the left wall.
* Each rack carries its QR plate 232.5 mm above the field floor; the rack
  footprint is 150 x 140 mm.  The robot is capped at 180 mm tall, so the
  camera sits near the top of that envelope and tilts slightly up.
* QR payloads: `START`, `RACKA_XXXX` .. `RACKD_XXXX`, `END` (the `XXXX` part
  is a random unique string).
* Racks must not touch the floor; collisions deduct 2 points; each delivered
  rack scores 5 points.
* Match length: 3 minutes. The robot must be at most 30 x 30 x 18 cm.
* Deadline: physical robot + materials to HW 103A by 2026-11-26 17:00;
  programs and CAD zip to Moodle by 23:59.

## Building on the Pi

The lidar driver and our packages live in `~/lidar_ros_ws`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/lidar_ros_ws
colcon build --symlink-install --packages-select qr_scanner robot_bringup game_manager robot_navigation
source install/setup.bash
```

The camera driver is a separate workspace because it needs a patched
`camera_ros` against the Raspberry Pi libcamera fork:

* libcamera: `v0.7.2+rpt20261009` (Raspberry Pi fork)
* camera_ros: 0.7.0, sources patched in `~/camera_ws/src/camera_ros`
  (`libcamera::Span` -> `std::span`, `libcamera::dynamic_extent` ->
  `std::dynamic_extent`)
* built with `~/dase7503/build_camera_ws.sh` into `~/camera_ws/install3`
  (a fresh build base was needed because meson caches libpisp flags)
* `~/.bashrc` sources `~/camera_ws/install3/setup.bash`

## Current status

| Item | State |
| --- | --- |
| Camera Module 3 | working, `/camera/image_raw` at 640x480, 15 fps |
| QR recognition | working live (`START`, `RACKA_4X6M`), 3 Hz decode, crash-guarded |
| MS200 lidar | working, `/scan` at 10 Hz |
| Unified sensor bringup | working, one command starts all three nodes |
| Game state machine | tested in dry-run and with Nav2 (4 racks, score 20) |
| Nav2 on the field map | working: `desk_mode` rehearsal, 4 racks in 97 s, score 20 (final code) |
| Visualization | RViz config `rviz/field.rviz` (`--rviz`); Foxglove bridge on port 8765 (`--foxglove`) |
| Cartographer (lidar only) | installed; mapping + pbstream -> pgm validated on the desk |
| slam_toolbox | installed; needs wheel odometry, not usable until the chassis exists |
| Synthetic field map | generated from the drawing (75 x 54 px at 0.05 m) |
| micro-ROS base bridge | blocked: no control board / chassis yet |
| Field map from lidar | not built yet (Cartographer launch ready) |

## Next steps

1. Map the real field with Cartographer (lidar only, push the robot by hand)
   and compare rack poses with the synthetic map.
2. Integrate the micro-ROS base: `/cmd_vel` in, `/odom` + TF out, then tune
   Nav2 speed limits and the camera mount TF (`docs/micro_ros_contract.md`).
3. Measure the lift's reach and set the rack approach distance; check that the
   camera sees the plate at 232.5 mm from that distance.
4. Full 3-minute rehearsal on the field, then the PowerPoint and submission.
