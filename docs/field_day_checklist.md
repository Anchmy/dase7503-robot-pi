# 场地日清单

在真实场地上按顺序要做的事，以及判断每一步是否真正成功的验收标准。以下所有内容都在树莓派（`rsi-bd-11`）上运行；命令假设两个工作区都已 source：

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install3/setup.bash
source ~/lidar_ros_ws/install/setup.bash
```

## 0. 离开桌面之前

* 桌面排练通过：4 次投放，得分 20，97 秒（`docs/rehearsal_2026-10-18.md`）。
* 带上：卷尺（5 m）、美纹纸、一张中心安装在 232.5 mm 高的打印二维码标签、充电器、手机热点。
* 给树莓派充电，并检查 SD 卡没有满（`df -h /`）。

## 1. 启动传感器

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

带底盘时用 `publish_odom_tf:=true`；在桌面上用 `fake_base` 时保持 false（见 `docs/micro_ros_contract.md`）。

验收：

```bash
ros2 topic hz /scan                # ~10 Hz
ros2 topic hz /camera/image_raw    # ~10 Hz（Foxglove 推流时 >= 5 Hz）
ros2 node list | grep lidar_watchdog
ros2 run tf2_ros tf2_echo base_link camera
# translation (0.120, 0.000, 0.170), pitch -5.73 deg
```

`lidar_watchdog` 在 `/scan` 停止 5 秒后重发 MS200 启动命令，数据恢复时记录 `lidar stream recovered`。USB 重枚举时它救不了：`/dev/oradar` 会指向新的 `ttyACM*`，而驱动仍抱着已删除的旧 tty。怀疑雷达之前，先对比 `readlink /dev/oradar` 与 `ls -l /proc/$(pgrep -f oradar_scan)/fd`；不一致就重启整栈。`dmesg` 里反复出现 `over-current change` 说明是线缆或接头短路，不是软件；没有过流日志、只有一条 `USB disconnect` 也仍然是重枚举，所以先查 tty 是否一致，别把“没有过流”当成正常。

## 2. 用雷达给场地建图（底盘还不存在时）

AMCL 必须把真实场地与地图匹配起来。Cartographer 不需要轮式里程计，所以可以在手动推车时运行。

```bash
ros2 launch robot_navigation cartographer_mapping.launch.py
```

推着机器人绕场地慢慢走（约 0.2 m/s）：沿四面墙各走一遍，然后绕货架一圈，最后回到起点区。让雷达与墙保持 0.3–1.5 m；不要原地旋转超过约 30 度/秒。

保存并转换：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/rsi-bd-11/field_map.pbstream', include_unfinished_submaps: true}"

ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=$HOME/field_map.pbstream \
  -map_filestem=$HOME/field_map -resolution=0.05
```

验收：

* `field_map.pgm` 显示四面直墙和货架轮廓，没有重影墙（有重影墙说明扫描匹配滑了——用更慢的速度重走一圈）。
* 量出的墙到墙尺寸是 3020 x 2000 mm（0.05 m 下 60 x 40 格，再加一点边距）。
* 地图能在 Nav2 中加载，机器人在起点区的位姿 (2.72, 1.00) 位于地图的自由空间内，朝向场地。

**启动 Nav2 之前先停掉 Cartographer。** 两者都发布 `/map`；残留的 `cartographer_occupancy_grid_node` 会替换静态场地图，规划器随后从所有位姿都失败：

```bash
pgrep -af cartographer               # 必须没有任何输出
ros2 topic info /map                 # Nav2 启动后 Publisher count: 1
```

## 3. 在场地图上定位

```bash
ros2 launch robot_navigation navigation.launch.py map:=$HOME/field_map.yaml
```

AMCL 从 `nav2_params.yaml` 里配置的起点区位姿开始（`initial_pose: 2.72, 1.00, yaw 3.1416`）。把机器人实际停在起点区、朝向场地，在 RViz 里检查收敛情况（`/amcl_pose` 和粒子云）。

如果位姿不对，要么修正地图原点，要么手动设置初始位姿；在 `map -> base_link` 看起来正确之前，不要让 `game_manager` 启动。

验收：

```bash
ros2 run tf2_ros tf2_echo map base_link     # 有限值，且在场地范围内
```

## 4. 相机 / 二维码标签几何

标签中心距场地地面 232.5 mm；相机光心在 170 mm，向上倾斜 5.73 度。在 0.35 m 接近距离处，标签大约在水平线上方 10 度，即光轴上方约 4 度——完全在画面内。要实测，不要只相信计算：

1. 把打印好的二维码标签（`RACKA_…`）放到货架上，中心正好离地 232.5 mm。
2. 把机器人停在接近位姿（距货架正面 0.35 m）。
3. `ros2 topic echo /qr_detected` 必须在 1 秒内打印出内容。
4. 把标签上移 50 mm 和下移 50 mm 各重复一次：内容仍必须能解出，因为真实货架和地面有差异。

如果标签在画面顶部被切掉，抬高相机或增大 `robot_bringup.launch.py` 里的俯仰角；如果在底部被切掉，降低相机。之后重新检查 TF。

实测解码距离（填写）：______ m。最小可靠距离：______ m。

## 5. 标定货架位姿

`game_manager/config/game_params.yaml` 里的位姿来自图纸（`tools/make_field_map.py`）。实测货架中心（从场地左下角用卷尺量）：

| 货架 | 图纸 (x, y) | 实测 (x, y) | 使用的接近位姿 |
| --- | --- | --- | --- |
| A | 1.275, 1.580 | | 1.275, 1.230, +1.5708 |
| B | 1.825, 0.510 | | 1.825, 0.860, -1.5708 |
| C | 1.925, 1.720 | | 1.925, 1.370, +1.5708 |
| D | 1.325, 0.200 | | 1.325, 0.550, -1.5708 |

接近位姿在货架正面之前 0.35 m，正对货架。只有在测量升降机构的 reach 之后才把它移得更近；二维码会缓存 30 秒（`rack_qr_cache_time`），所以最终位姿不需要一直看得到标签。

验收：在每个货架接近位姿处运行 `ros2 run tf2_ros tf2_echo map base_link`，与卷尺测量结果相差不超过 50 mm。

## 6. 运行比赛栈

一条命令（传感器已经启动时会跳过）：

```bash
~/lidar_ros_ws/install/robot_bringup/lib/robot_bringup/start_match.sh \
    --map ~/field_map.yaml
```

带底盘时加 `--odom-tf`，让 `/odom` 广播为 TF。加 `--rviz` 可在树莓派显示器上观看；或者加 `--foxglove`，从笔记本用 Foxglove 桌面版 3.x 连接 `ws://<pi-ip>:8765`（旧版 Studio 只支持 `foxglove.websocket.v1`，3.x 桥接会拒绝）。Ctrl-C 停止整个栈。不用脚本时：

```bash
ros2 launch game_manager game_manager.launch.py
```

观察 `/game_state`、`/game_score` 和 `/lift_cmd`。现场 START 二维码从标签读取；不使用 supervisor。

完整运行的验收：

* 四个货架全部抬起并送达，得分 20；
* Nav2 日志中没有规划器或控制器失败；
* 用时低于 150 秒（桌面排练：97 秒），在 180 秒比赛限制内留出余量；
* END 之后机器人自己停在投放区。

## 7. 时间预算

| 阶段 | 桌面 | 现场允许 |
| --- | --- | --- |
| 传感器启动 | 15 s | 30 s |
| START -> 货架 A | 9.7 s | 15 s |
| 货架 -> 投放区 | 9.1 s | 15 s |
| 完整任务 | 97 s | 150 s |

如果现场运行超过 150 秒，先提高 `nav2_params.yaml` 里的速度限制（`max_vel_x`、`max_speed_xy`），再考虑改动货架顺序：场地只有 3 m 宽，大部分时间都花在行驶上。
