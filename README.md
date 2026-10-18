# DASE7503 项目 — 树莓派软件栈

运行在机器人树莓派 5 上的 ROS 2 Jazzy 软件，覆盖感知与任务层：相机、二维码识别、雷达和比赛状态机。电机控制 / micro-ROS 桥接在底盘固件里，不在本工作区。

## 硬件

| 项目 | 型号 | 说明 |
| --- | --- | --- |
| 计算机 | Raspberry Pi 5, 8 GB | Ubuntu 24.04 (noble), aarch64 |
| 相机 | Camera Module 3 (IMX708) | 640x480 @ 15 fps, XRGB8888 -> `bgra8` |
| 雷达 | MS200 (OrADAR) | 360 度, 10 Hz, `/dev/oradar`, 230400 baud |

主机名：`rsi-bd-11`（192.168.3.18）。

## 软件包

```
src/
  qr_scanner/        相机 + 二维码识别节点，以及组合 launch 文件
  robot_bringup/     一条命令启动全部传感器（相机 + QR + 雷达）
  game_manager/      比赛状态机（START -> 货架 -> 放置区 -> END）
  robot_navigation/  SLAM 与 Nav2 配置、地图
tools/
  make_field_map.py       根据课程图纸生成合成场地图
  mission_supervisor.py   桌面排练：注入比赛现场会给出的 QR 内容
docs/
  rehearsal_2026-10-18.md  第一次端到端运行、时间线和限制
  field_day_checklist.md   建图、相机/QR 几何、货架标定、验收测试
  micro_ros_contract.md    ESP32 固件需要提供什么
```

`robot_bringup` 是一次运行的入口：它启动 `qr_scanner` 和 `oradar_lidar` 的全部内容，以及雷达 launch 文件自带的静态 `base_link -> lidar` TF。

### qr_scanner

* `qr_node` 订阅 `/camera/image_raw`，解码二维码并把内容发布到 `/qr_detected`（`std_msgs/String`）。
* 解码频率最高 3 Hz（`--rate`），并且先把画面缩到宽 640 px（`--max-width`）。配合 640x480 的相机帧，相机 CPU 从约 40% 降到 14%，QR CPU 从约 52% 降到 36%，为树莓派 5 上的 Nav2 留出了足够余量。
* 持续出现在画面里的内容每 2 秒重发一次（`--repeat-interval`）。否则第一次看到时只发布一次，如果状态机还没开始监听就会丢失——货架标签在接近过程中出现时正是这种情况。
* 优先使用 OpenCV `QRCodeDetector`，失败时回退到 pyzbar。
* 启动：`ros2 launch qr_scanner camera_qr.launch.py`

### robot_bringup

一条命令启动相机、QR 识别和雷达：

```bash
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_robot.sh
# 或者先 source 各工作区后：
ros2 launch robot_bringup robot_bringup.launch.py
```

话题：`/camera/image_raw`、`/qr_detected`、`/scan`，TF `base_link -> lidar`。

`start_match.sh` 用一条命令启动整场比赛（它会自己 source 三个工作区，跳过已在运行的传感器，并把每个组件的日志写到 `~/dase7503/logs/`）：

```bash
# 桌面排练：传感器 + fake_base + Nav2 desk_mode + game_manager
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh --desk

# 无人值守的桌面排练：还会注入 START / RACKx / END
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --desk --supervisor

# 现场：传感器 + 在已建图的场地上运行 Nav2 + game_manager
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --map ~/field_map.yaml

# 现场带底盘：同时把 /odom 广播为 TF
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --map ~/field_map.yaml --odom-tf
```

Ctrl-C 会停止它启动的所有组件。

bringup 还会启动 `lidar_watchdog`。厂商驱动只在 `Activate()` 里发送 `SET_RUN_MODE`，而 `Activate()` 只有在驱动已经收到扫描数据后才会执行；所以上电时处于停止状态的 MS200 会永远保持沉默。看门狗监视 `/scan`：5 秒没有扫描就发送 SDK 会发的转速 + 启动帧，每 20 秒重试一次，数据恢复时记录 `lidar stream recovered`。扫描正常时它不写任何数据。

看门狗无法恢复 USB 重枚举。如果适配器从总线上掉下来——有时 `dmesg` 会对每个根集线器报 `over-current change`，有时只有一条 `USB disconnect`、完全没有过流——`/dev/oradar` 会指向新的 `ttyACM*`，而正在运行的驱动仍抱着已删除的旧 tty，永远看不到新设备。检查：

```bash
readlink /dev/oradar
ls -l /proc/$(pgrep -f oradar_scan)/fd | grep ttyACM   # 必须一致
```

不一致就重启整栈；只有重启才会重新打开端口。

### 可视化

本栈默认无界面。有两种方式查看运行情况：

* **在树莓派自己的屏幕上用 RViz**：给 `start_match.sh` 加 `--rviz`（或给 `navigation.launch.py` 加 `rviz:=true`）。它加载 `rviz/field.rviz`：场地图、全局/局部代价地图、激光扫描、全局/局部路径、AMCL 位姿和相机画面。从 SSH 启动时，脚本会自动接入桌面会话的 Xwayland，因为 RViz 通过 Ogre/GLX 渲染，即使在 Wayland 桌面上也需要 X display。
* **在另一台电脑上用 Foxglove**：先安装一次桥接（`sudo apt install ros-jazzy-foxglove-bridge`），给 `start_match.sh` 加 `--foxglove`，然后用 Foxglove 桌面版（3.x）连接 `ws://<pi-ip>:8765`（脚本会打印确切的 URL）。3.6 版桥接只接受 `foxglove.sdk.v1` 子协议；Foxglove Studio 2.9 及更早版本（以及 Lichtblick）只提供 `foxglove.websocket.v1`，会被拒绝并提示 "does not support protocol version foxglove.websocket.v1"。

### game_manager

打一场比赛的状态机。它只消费二维码内容和 Nav2 action 结果，不直接读取传感器。

```
WAIT_START --START--> DRIVE_TO_RACK --arrived--> WAIT_RACK_QR
    --RACKx_xxxx--> LIFT_UP --lifted--> DRIVE_TO_DEST --arrived-->
    LIFT_DOWN --lowered--> next rack ... -> WAIT_END_QR --END--> DONE
```

话题/action：

* 订阅 `/qr_detected`
* 发布 `/game_state`、`/game_score`、`/lift_cmd`（`up` / `down`）
* `navigate_to_pose`（Nav2）的 action client

在桌面上不启动 Nav2 单独运行：

```bash
ros2 run game_manager game_manager --ros-args \
  -p use_nav2:=false -p dry_run_travel_time:=3.0
```

手动注入二维码事件：

```bash
ros2 topic pub --once /qr_detected std_msgs/msg/String "{data: 'START'}"
ros2 topic pub --once /qr_detected std_msgs/msg/String "{data: 'RACKA_4X6M'}"
```

所有场地位姿都是 `game_manager/config/game_params.yaml` 里的参数（map 坐标系、米、弧度）。它们由课程图纸里的尺寸链推导而来（货架中心加 0.35 m 接近偏移），所以只需要根据真实场地做小幅修正。

值得了解的现场行为：

* `destination_points` 每次投放一个点，间隔 0.35–0.40 m。如果所有货架都放在同一个位姿，下一个目标就会落在刚放下的货架上。
* 接近时第一次看到的货架二维码会缓存 30 秒（`rack_qr_cache_time`），到达后复用，所以机器人停在货架前时不需要一直看得到标签。
* 只接受所前往货架的二维码（`strict_rack_match`）；相邻货架的标签可能进入画面。
* END 是放置区的二维码，每次投放时都在画面里，所以任务中途看到的 END 会被忽略。`end_qr_stops_mission:=true` 会恢复旧的急停行为，用于桌面测试。
* 前往货架的导航失败会重试一次，然后跳过该货架。

### robot_navigation

* `maps/field.pgm` + `maps/field.yaml` — 由 `tools/make_field_map.py` 根据图纸生成的合成占据栅格地图。原点是场地左下角；使用 Nav2 前不需要先建图。
* `launch/cartographer_mapping.launch.py` — 仅用雷达的建图，不需要轮式里程计；底盘不可用或要验证合成地图时使用。
* `launch/slam.launch.py` — 有 `/odom` 后用 slam_toolbox 在线建图。
* `launch/navigation.launch.py` — 在 `maps/field.yaml` 上运行 Nav2 + AMCL。
* `scripts/fake_base.py` + `launch/fake_base.launch.py` — 底盘的桌面替身。它把 `/cmd_vel` 积分成 `/odom` 并广播 `odom -> base_link`，所以在 micro-ROS 固件还不存在时也能测试 Nav2 和 `game_manager`。绝不要在真车上启动它。
* **永远不要同时运行 Cartographer 和 Nav2。** 两者都发布 `/map`；残留的 `cartographer_occupancy_grid_node` 会替换合成场地图，规划器就会从所有位姿都失败。启动 Nav2 前用 `ros2 topic info /map` 确认只有一个发布者。

无底盘的桌面测试：

```bash
ros2 launch robot_navigation fake_base.launch.py   # /odom + TF
ros2 launch robot_navigation navigation.launch.py  # Nav2 加载 field.pgm
ros2 launch game_manager game_manager.launch.py    # 任务，use_nav2=true
```

始终通过 launch 文件启动 `game_manager`：`ros2 run` 会跳过 `game_params.yaml`，回退到代码默认值（起点 2.6/0.4，货架位姿 0.8/1.6），那不是场地的布局。

### qr_scanner 崩溃保护

OpenCV 4.6 在某些低纹理画面上会让 `QRCodeDetector.detectAndDecode` 抛出 `cv2.error`。`decoder.py` 隔离每次解码尝试，`qr_node` 再捕获所有漏掉的异常，所以一帧坏画面不会杀死 QR 节点。

## 比赛规则（来自课程 PDF）

* 场地 3020 x 2000 mm，围栏高 500 mm；起点区 600 x 800 mm 在右下角；放置区 800 x 1400 mm 沿左墙。
* 每个货架的二维码标签中心距场地地面 232.5 mm；货架占地 150 x 140 mm。机器人高度上限 180 mm，所以相机位于这个包络的顶部附近并略微上仰。
* 二维码内容：`START`、`RACKA_XXXX` .. `RACKD_XXXX`、`END`（`XXXX` 是随机的唯一字符串）。
* 货架不能碰到地面；碰撞扣 2 分；每送达一个货架得 5 分。
* 比赛时长：3 分钟。机器人尺寸不得超过 30 x 30 x 18 cm。
* 截止时间：实体机器人 + 材料 2026-11-26 17:00 前送到 HW 103A；程序和 CAD 压缩包 23:59 前上传 Moodle。

## 在树莓派上构建

雷达驱动和我们的包在 `~/lidar_ros_ws`：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/lidar_ros_ws
colcon build --symlink-install --packages-select qr_scanner robot_bringup game_manager robot_navigation
source install/setup.bash
```

相机驱动单独一个工作区，因为它需要针对树莓派 libcamera 分支打补丁的 `camera_ros`：

* libcamera：`v0.7.2+rpt20261009`（Raspberry Pi fork）
* camera_ros：0.7.0，源码在 `~/camera_ws/src/camera_ros` 打过补丁（`libcamera::Span` -> `std::span`，`libcamera::dynamic_extent` -> `std::dynamic_extent`）
* 用 `~/dase7503/build_camera_ws.sh` 构建到 `~/camera_ws/install3`（需要全新的构建目录，因为 meson 会缓存 libpisp 标志）
* `~/.bashrc` 会 source `~/camera_ws/install3/setup.bash`

## 当前状态

| 项目 | 状态 |
| --- | --- |
| 相机模块 3 | 正常，`/camera/image_raw` 640x480、15 fps |
| 二维码识别 | 实时可用（`START`、`RACKA_4X6M`），3 Hz 解码，有崩溃保护 |
| MS200 雷达 | 正常，`/scan` 10 Hz |
| 统一传感器 bringup | 正常，一条命令启动三个节点 |
| 比赛状态机 | 已在 dry-run 和 Nav2 下测试（4 个货架，得分 20） |
| 场地图上的 Nav2 | 正常：`desk_mode` 排练 4 个货架 97 秒，得分 20（最终代码） |
| 可视化 | RViz 配置 `rviz/field.rviz`（`--rviz`）；Foxglove 桥接端口 8765（`--foxglove`） |
| Cartographer（仅雷达） | 已安装；建图和 pbstream -> pgm 已在桌面上验证 |
| slam_toolbox | 已安装；需要轮式里程计，底盘到位前不可用 |
| 合成场地图 | 已根据图纸生成（75 x 54 px，0.05 m） |
| micro-ROS 底盘桥接 | 阻塞：还没有控制板 / 底盘 |
| 雷达实测场地图 | 尚未构建（Cartographer launch 已就绪） |

## 下一步

1. 用 Cartographer 对真实场地建图（仅雷达，手动推车），并把货架位姿与合成地图对比。
2. 接入 micro-ROS 底盘：`/cmd_vel` 输入，`/odom` + TF 输出，然后调整 Nav2 速度限制和相机安装 TF（`docs/micro_ros_contract.md`）。
3. 测量升降机构的 reach（可触及范围），设定货架接近距离；检查相机在该距离能看到 232.5 mm 高的标签。
4. 在场地做完整的 3 分钟排练，然后做 PPT 和提交。
