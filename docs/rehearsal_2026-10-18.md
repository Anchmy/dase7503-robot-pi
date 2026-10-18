# Desk rehearsal — 2026-10-18

First end-to-end run of the mission stack: sensors, Nav2, the game state
machine and a fake chassis, all on the Raspberry Pi 5 (`rsi-bd-11`).

## What was running

| Node | Configuration |
| --- | --- |
| Camera Module 3 | 640 x 480, 15 fps (`FrameDurationLimits` 66666 us) |
| QR scanner | 3 Hz decode on 640 px-wide frames, 2 s re-publish |
| MS200 lidar | `/scan` at 10 Hz, `base_link -> lidar` static TF |
| `fake_base` | integrates `/cmd_vel` into `/odom`, publishes `odom -> base_link` |
| Nav2 | `navigation.launch.py desk_mode:=true` |
| `game_manager` | `game_manager.launch.py`, `use_nav2:=true` |
| `mission_supervisor` | injects START / RACKx / END when the state machine asks |

`desk_mode` disables the live lidar obstacle layers, the collision monitor's
scan source and AMCL's scan updates, because the desk is not the game field.
The production configuration (`config/nav2_params.yaml`) is unchanged.

## Timeline

Numbers below are from the final-code run (strict rack matching, QR cache,
`destination_points`, END ignored mid-mission, 1000 ms server timeout, one
rack retry). The first run, before those changes, ended at the same 97.0 s.

| t (s) | State | Note |
| --- | --- | --- |
| 3.0 | WAIT_START -> DRIVE_TO_RACK | START injected |
| 12.7 | WAIT_RACK_QR | rack A reached (9.7 s drive) |
| 12.8 | LIFT_UP | `RACKA_4X6M` injected, accepted at once |
| 14.4 | DRIVE_TO_DEST | lift time 1.6 s |
| 23.5 | LIFT_DOWN | 9.1 s to drop point 1 |
| 25.0 | DRIVE_TO_RACK | score 5 |
| 37.4 | WAIT_RACK_QR | rack B reached (12.4 s) |
| 39.0 | DRIVE_TO_DEST | back to drop point 2 |
| 49.6 | DRIVE_TO_RACK | score 10 |
| 61.2 | WAIT_RACK_QR | rack C reached (11.5 s) |
| 62.8 | DRIVE_TO_DEST | to drop point 3 |
| 73.6 | DRIVE_TO_RACK | score 15 |
| 83.4 | WAIT_RACK_QR | rack D reached (9.7 s) |
| 85.0 | DRIVE_TO_DEST | to drop point 4 |
| 96.9 | WAIT_END_QR | score 20 |
| 97.0 | DONE | END injected |

**Result: 4 shipments, score 20, 97.0 s of the 180 s limit.**

The two logs behind the table are kept under `docs/rehearsal_logs/`:
`2026-10-18_supervisor.log` (state transitions with timestamps) and
`2026-10-18_game_manager.log` (node decisions and rack QR confirmations).

The four drop points are 0.35-0.40 m apart inside the destination zone
(`destination_points` in `game_params.yaml`); the first version dropped every
rack on the same pose, which would have driven the robot into the rack it had
just delivered.

## Field guards verified separately

| Guard | Test | Result |
| --- | --- | --- |
| END mid-mission | publish `END` during DRIVE_TO_RACK | ignored, state unchanged |
| QR seen on approach | publish `RACKA_4X6M` 1 s after START | reused on arrival, LIFT_UP at once |
| Wrong rack QR | publish `RACKB_7K2M` at rack A, then A | B ignored, A accepted |

## What this proves

* The state machine and Nav2 action interface work together: every transition
  after `DRIVE_TO_RACK` is caused by a real `navigate_to_pose` result.
* Nav2 plans and drives the field map: start -> four racks -> four drop
  points, eight legs, no planner or controller failures.
* The QR crash guard holds: `qr_node` stayed alive throughout (it previously
  died on an OpenCV `convexHull` assertion).
* A `bt_navigator` goal ack timeout on a loaded Pi aborts a whole navigation.
  The upstream `default_server_timeout` of 20 ms is too tight here; it is now
  1000 ms, and a failed rack goal is retried once before the rack is skipped.

## Operational pitfall: two publishers on `/map`

Cartographer's `occupancy_grid_node` and Nav2's `map_server` both publish
`/map`. If a mapping launch is stopped but its `occupancy_grid_node` survives
(it is a separate process from `cartographer_node`), Nav2's static layer
receives whichever map arrives last — the desk room instead of the synthetic
field — and the planner then fails from every pose:

```
Failed to create plan with tolerance of: 0.200000
```

Rule: never run Cartographer and Nav2 at the same time, and after stopping a
mapping run check that `/map` has exactly one publisher before starting Nav2:

```bash
ros2 topic info /map     # Publisher count: 1
```

`pkill` on the launch script is not enough; verify with `pgrep -af cartographer`
and kill surviving `cartographer_occupancy_grid_node` processes by PID.

## What it does not prove

* **QR detection is simulated.** The supervisor publishes the payload the
  moment the robot reaches a rack. On the field the camera must read the
  plate at 232.5 mm height within the 15 s rack timeout.
* **No chassis.** `fake_base` is a kinematic integrator; there is no wheel
  slip, no real odometry, no micro-ROS.
* **No lift.** `/lift_cmd` is published but nothing consumes it.
* **No AMCL.** Localisation was frozen at the start pose. On the field AMCL
  must match the synthetic map against the real walls and racks.
* **Poses are from the drawing, not measured.** The rack approach poses
  (0.35 m in front of each rack) need checking on the real field, and the
  approach distance depends on the lift's reach.

## Reproducing it

```bash
# one command, hands-free (sensors + fake_base + Nav2 + game_manager +
# supervisor, stopped automatically when the mission ends):
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --desk --supervisor
```

Or step by step, with sensors already up via `robot_bringup`:

```bash
ros2 run robot_navigation fake_base.py --ros-args \
  -p initial_x:=2.72 -p initial_y:=1.0 -p initial_yaw:=3.1416
ros2 launch robot_navigation navigation.launch.py desk_mode:=true
ros2 launch game_manager game_manager.launch.py
python3 ~/dase7503/mission_supervisor.py --timeout 200
```
