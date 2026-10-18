# micro-ROS 底盘接入约定

ESP32 固件必须提供什么，树莓派软件栈（Nav2 + `game_manager`）才能驱动机器人。以下所有内容都可以在没有升降机构和相机的情况下测试。

## 话题

| 方向 | 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- | --- |
| Pi -> ESP32 | `/cmd_vel` | `geometry_msgs/Twist` | 最高 20 Hz | `linear.x` m/s，`angular.z` rad/s |
| ESP32 -> Pi | `/odom` | `nav_msgs/Odometry` | >= 30 Hz | 轮式积分位姿 |
| Pi -> ESP32 | `/lift_cmd` | `std_msgs/String` | 变化时 | `"up"` / `"down"` |
| ESP32 -> Pi | `/lift_state` | `std_msgs/String` | ~5 Hz | 可选：`"up"`、`"down"`、`"moving"` |

`/cmd_vel` 由 Nav2 经 `velocity_smoother` 和 `collision_monitor` 发布；固件不需要知道这些。

## 坐标系与 TF

树莓派栈使用 `map -> odom -> base_link -> {lidar, camera}`。

* `/odom` 必须带 `header.frame_id: odom` 和 `child_frame_id: base_link`。
* ESP32 **不必**广播 TF。真车上启动 `robot_bringup` 时加 `publish_odom_tf:=true`，它会把 `/odom` 转成 `odom -> base_link`（`robot_bringup/odom_to_tf.py`）。
* **不要**同时运行 `fake_base` 和 `odom_to_tf`：两者会广播同一个变换。
* 如果固件自己广播 `odom -> base_link`，就保持 `publish_odom_tf:=false`。

## 时间

`tf2` 会拒绝与树莓派时钟相差太远的时间戳。症状是 `Lookup would require extrapolation into the future`，以及 Nav2 目标立即中止。

* agent 连接建立后调用 `rmw_uros_sync_session(1000)`。
* 每条 `/odom` 消息都用 `rmw_uros_epoch_nanos()` 打时间戳。
* 如果时间戳留为 0，`odom_to_tf` 会回退到树莓派时钟，但里程计就没有时间信息，不能用于导航。

## `/cmd_vel` 行为

* 单位：米/秒（正为前进）和弧度/秒（正为逆时针）。`linear.y` 始终为 0。
* 我们的 `velocity_smoother` 把指令限制在 0.25 m/s 和 1.0 rad/s。
* 把 `|linear.x| < 0.01` 和 `|angular.z| < 0.01` 视为停车。
* **安全超时：** 如果 0.5 秒没有收到 `/cmd_vel`，两个电机都停止。Nav2 停止时会发布零速度，但固件不能依赖树莓派还活着。
* 底盘比较舒适的极限约为 0.5 m/s^2 线加速度和 1.5 rad/s^2 角加速度；更快会让 DWB 冲过目标。

## 里程计质量

* 对两个轮编码器积分；在 `odom` 中发布 `x`、`y`、`yaw`。
* 位姿协方差对角线填小值（例如 0.002, 0.002, 0.0, 0.0, 0.0, 0.01）；AMCL 会用到。
* 5% 的尺度误差没关系：AMCL 会用雷达修正。符号错误或每米几度的偏航漂移则不行。
* 至少 30 Hz 发布。低于约 15 Hz 时，转弯时代价地图的 TF 查询会开始失败。

## 升降

* `/lift_cmd` 携带 `"up"` 和 `"down"` 字符串。
* `game_manager` 在每条命令后等待 `lift_time` 秒（`game_params.yaml` 中为 1.5 s）。如果真实升降更慢，调大这个参数；否则货架还在动，机器人就开走了。
* 机器人开始驶向投放区之前，货架必须已经离开地面。

## 真车上的启动顺序

1. 给 ESP32 上电；在树莓派上启动 micro-ROS agent（`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200`）。
2. `ros2 launch robot_bringup robot_bringup.launch.py publish_odom_tf:=true`
3. `ros2 launch robot_navigation navigation.launch.py`
4. `ros2 launch game_manager game_manager.launch.py`

## 验收测试

```bash
# 1. 里程计存在且足够快
ros2 topic hz /odom                         # 期望 >= 30 Hz

# 2. TF 链完整
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link     # Nav2 启动后

# 3. 方向符号：+x 必须让机器人前进，+z 必须让它左转
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
# odom 的 x 应每秒增加约 0.10 m

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.30}}"
# odom 的 yaw 应每秒增加约 0.30 rad

# 4. 安全超时：停止发布后，机器人必须在 0.5 秒内停下
```

然后在开阔区域让 Nav2 走一个目标，并用卷尺对比 `/odom`：最终误差应该是几厘米，而不是几十厘米。

## 已知的坑

* 课程参考固件发布的 `odom` 使用 `frame_id: odom_frame` 和 `child_frame_id: base_footprint`。要么在固件里改名，要么依赖 `odom_to_tf`（它忽略消息里的 frame 名称）。
* micro-ROS 会话断开（WiFi 或 USB）时必须停车；agent 会自动重连，但固件会保留最后一条指令，除非实现了 0.5 秒超时。
* 保持 `/cmd_vel` 回调简短。在 executor 回调里做 PID 运算会饿死 micro-ROS 的 spin 循环，并给其他所有话题增加延迟。
