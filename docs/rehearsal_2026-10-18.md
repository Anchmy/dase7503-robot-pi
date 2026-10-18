# 桌面排练 — 2026-10-18

任务栈的第一次端到端运行：传感器、Nav2、比赛状态机和假底盘，全部跑在树莓派 5（`rsi-bd-11`）上。

## 运行了什么

| 节点 | 配置 |
| --- | --- |
| 相机模块 3 | 640 x 480，15 fps（`FrameDurationLimits` 66666 us） |
| QR 识别 | 3 Hz 解码，画面宽 640 px，每 2 s 重发 |
| MS200 雷达 | `/scan` 10 Hz，静态 TF `base_link -> lidar` |
| `fake_base` | 把 `/cmd_vel` 积分成 `/odom`，发布 `odom -> base_link` |
| Nav2 | `navigation.launch.py desk_mode:=true` |
| `game_manager` | `game_manager.launch.py`，`use_nav2:=true` |
| `mission_supervisor` | 状态机请求时注入 START / RACKx / END |

`desk_mode` 关闭了实时雷达障碍层、碰撞监测的扫描源和 AMCL 的扫描更新，因为桌面不是比赛场地。生产配置（`config/nav2_params.yaml`）没有改动。

## 时间线

下面的数字来自最终代码的运行（严格货架匹配、二维码缓存、`destination_points`、任务中途忽略 END、1000 ms 服务器超时、一次货架重试）。在这些改动之前的第一次运行也在同样 97.0 秒结束。

| t (s) | 状态 | 说明 |
| --- | --- | --- |
| 3.0 | WAIT_START -> DRIVE_TO_RACK | 注入 START |
| 12.7 | WAIT_RACK_QR | 到达货架 A（行驶 9.7 s） |
| 12.8 | LIFT_UP | 注入 `RACKA_4X6M`，立即接受 |
| 14.4 | DRIVE_TO_DEST | 升降时间 1.6 s |
| 23.5 | LIFT_DOWN | 9.1 s 到投放点 1 |
| 25.0 | DRIVE_TO_RACK | 得分 5 |
| 37.4 | WAIT_RACK_QR | 到达货架 B（12.4 s） |
| 39.0 | DRIVE_TO_DEST | 回到投放点 2 |
| 49.6 | DRIVE_TO_RACK | 得分 10 |
| 61.2 | WAIT_RACK_QR | 到达货架 C（11.5 s） |
| 62.8 | DRIVE_TO_DEST | 到投放点 3 |
| 73.6 | DRIVE_TO_RACK | 得分 15 |
| 83.4 | WAIT_RACK_QR | 到达货架 D（9.7 s） |
| 85.0 | DRIVE_TO_DEST | 到投放点 4 |
| 96.9 | WAIT_END_QR | 得分 20 |
| 97.0 | DONE | 注入 END |

**结果：4 次投放，得分 20，180 秒限制内用时 97.0 秒。**

表格背后的两份日志保存在 `docs/rehearsal_logs/`：`2026-10-18_supervisor.log`（带时间戳的状态转换）和 `2026-10-18_game_manager.log`（节点决策和货架 QR 确认）。

四个投放点在放置区内相隔 0.35–0.40 m（`game_params.yaml` 中的 `destination_points`）；第一版把每个货架都放在同一个位姿，会让机器人撞上它刚放下的货架。

## 单独验证过的现场保护

| 保护 | 测试 | 结果 |
| --- | --- | --- |
| 任务中途的 END | 在 DRIVE_TO_RACK 期间发布 `END` | 忽略，状态不变 |
| 接近时看到 QR | START 后 1 秒发布 `RACKA_4X6M` | 到达后复用，立即 LIFT_UP |
| 错误的货架 QR | 在货架 A 处发布 `RACKB_7K2M`，再发布 A | B 被忽略，A 被接受 |

## 这证明了什么

* 状态机和 Nav2 action 接口能配合：`DRIVE_TO_RACK` 之后的每次转换都由真实的 `navigate_to_pose` 结果触发。
* Nav2 能在场地图上规划和行驶：起点 -> 四个货架 -> 四个投放点，八段路程，没有规划器或控制器失败。
* QR 崩溃保护有效：`qr_node` 全程存活（之前它会因 OpenCV `convexHull` 断言而死亡）。
* 在负载较高的树莓派上，`bt_navigator` 的目标确认超时会中止整个导航。上游默认的 `default_server_timeout` 20 ms 在这里太紧；现在改为 1000 ms，而且失败的货架目标会重试一次再跳过。

## 操作上的坑：`/map` 有两个发布者

Cartographer 的 `occupancy_grid_node` 和 Nav2 的 `map_server` 都发布 `/map`。如果建图 launch 被停掉但它的 `occupancy_grid_node` 还活着（它和 `cartographer_node` 是两个进程），Nav2 的静态层会收到最后到达的那张地图——桌面房间而不是合成场地——规划器随后从所有位姿都失败：

```
Failed to create plan with tolerance of: 0.200000
```

规则：永远不要同时运行 Cartographer 和 Nav2；停止建图后，启动 Nav2 前确认 `/map` 只有一个发布者：

```bash
ros2 topic info /map     # Publisher count: 1
```

只对 launch 脚本 `pkill` 不够；用 `pgrep -af cartographer` 检查，并按 PID 杀掉残留的 `cartographer_occupancy_grid_node` 进程。

## 这没有证明什么

* **QR 识别是模拟的。** supervisor 在机器人到达货架的那一刻发布内容。现场相机必须在 15 秒货架超时内读出 232.5 mm 高的标签。
* **没有底盘。** `fake_base` 是运动学积分器；没有轮子打滑、没有真实里程计、没有 micro-ROS。
* **没有升降。** `/lift_cmd` 会发布，但没有人消费。
* **没有 AMCL。** 定位被冻结在起点位姿。现场 AMCL 必须把合成地图与真实的墙和货架匹配起来。
* **位姿来自图纸，不是实测。** 货架接近位姿（每个货架前 0.35 m）需要在真实场地上核对，接近距离取决于升降机构的 reach。

## 如何复现

```bash
# 一条命令，无人值守（传感器 + fake_base + Nav2 + game_manager +
# supervisor，任务结束时自动停止）：
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --desk --supervisor
```

或者分步执行，传感器已通过 `robot_bringup` 启动：

```bash
ros2 run robot_navigation fake_base.py --ros-args \
  -p initial_x:=2.72 -p initial_y:=1.0 -p initial_yaw:=3.1416
ros2 launch robot_navigation navigation.launch.py desk_mode:=true
ros2 launch game_manager game_manager.launch.py
python3 ~/dase7503/mission_supervisor.py --timeout 200
```
