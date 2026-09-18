# myworld_bringup

This package owns the `myworld2` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 中文翻译

# myworld_bringup

该包负责 myworld2 场景的 Launch 流程。场景专用 Launch 文件放在 launch/，地图、模型、RViz 和 URDF 资源放在本包中；公开入口是 entry.launch.py。

## 启动模式

默认自主模式和固定路径模式均由 `nav2_regulated_modules` 的统一入口负责规划、控制、平滑和底盘输出：

```bash
ros2 launch myworld_bringup entry.launch.py operation_mode:=autonomous
```

自主模式由 `regulated_navigator` 接收标准导航 Action，调用统一的 Planner、Smoother 和
Controller 接口；固定路径模式由同一个 `regulated_navigator` 接收
`byd_custom_msgs/action/NavigationService`，把 `NaviSegment[]` 生成的 Path 交给
Controller Server。两种模式都不再在本包内重复实现算法节点：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path \
  use_rviz:=true
```

无图形环境中可以关闭 RViz 和 Ignition GUI：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path \
  use_rviz:=false \
  headless:=true
```

遥控仿真使用独立的 `remote` 模式。该模式只启动 Gazebo、传感器桥接和
`chassis_control_to_twist`，不启动 AMCL、Planner、Controller、Velocity Smoother、
`regulated_navigator` 或 `controlpub`，保证 `/cmd_vel` 只有遥控网关一个控制源：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=remote \
  use_rviz:=false \
  headless:=false
```

遥控网关以 Reliable／Volatile QoS 订阅
`/downstream/chassis_control` 的 `byd_custom_msgs/msg/ChassisControl`，把前进、后退、
左转和右转分别映射为 `/cmd_vel` 的正／负线速度和正／负角速度。遥控端应以
`20-50 Hz` 连续发布；超过 `0.5 s` 没有新消息时，网关发布一次零速度并清除当前命令。
换向或直行／旋转切换时会先按加速度约束减速到零，再执行新命令。

例如以 `20 Hz` 持续前进：

```bash
ros2 topic pub -r 20 \
  /downstream/chassis_control \
  byd_custom_msgs/msg/ChassisControl \
  "{op: 0, linear_velocity: 0.2, angular_velocity: 0.0, acceleration: 0.5}"
```

原地左转：

```bash
ros2 topic pub -r 20 \
  /downstream/chassis_control \
  byd_custom_msgs/msg/ChassisControl \
  "{op: 2, linear_velocity: 0.0, angular_velocity: 0.4, acceleration: 1.0}"
```

显式停车：

```bash
ros2 topic pub --once \
  /downstream/chassis_control \
  byd_custom_msgs/msg/ChassisControl \
  "{op: 0, linear_velocity: 0.0, angular_velocity: 0.0, acceleration: 0.5}"
```

`ChassisControl` 当前一次只能表示直行或原地旋转，不能同时表达非零线速度和角速度。
网关默认将线速度限制在 `0.52 m/s`、角速度限制在 `2.0 rad/s`；非法数值、未知
`op` 或多个 `/cmd_vel` Publisher 会触发停车并等待新的有效遥控命令。

自主与固定路径模式的统一算法链为：

```text
/navigate_to_pose 或 /navigation_service -> regulated_navigator
  -> Planner/Smoother（自主模式） -> FollowPath -> /cmd_vel_nav
  -> velocity_smoother（/motion_state 闭环） -> /cmd_vel -> Gazebo DiffDrive
  -> controlpub -> /control_to_uart
```

固定路径的终点判定、速度平滑、MotionState 闭环、ChassisControl 监听和
`/control_to_uart` 输出均由 `nav2_regulated_modules` 封装；本包只提供仿真环境及其
输入适配。固定路径 `stopped_goal_checker` 的平面距离容差为 `10 mm`、航向容差为 `5°`，`FixedPathController` 固定路径速度默认由启动参数限制为 `0.8 m/s`；接近终点时允许速度降至
零，进入位置容差后先停车，再按终点航向完成对齐。

可调启动参数如下：

```text
fixed_path_max_linear_velocity            默认 0.80 m/s
fixed_path_approach_velocity_scaling_dist 默认 0.80 m
fixed_path_stack_start_delay              默认 3.00 s
rviz_start_delay                          默认 5.00 s
ign_partition                             默认按 ROS_DOMAIN_ID 和启动进程 PID 唯一生成
```

例如限制固定路径专用控制器的巡航速度和终点前减速距离：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path \
  fixed_path_max_linear_velocity:=0.8 \
  fixed_path_approach_velocity_scaling_dist:=0.8
```

`odom_to_motion_state` 把 Gazebo 的 `/odom` 转换为 `/motion_state`，供
Velocity Smoother 和底盘闭环使用；Controller Server 与 `regulated_navigator` 保持
读取仿真原生的 `nav_msgs/msg/Odometry` `/odom`。

固定路径与标准导航现在都使用原 `world_only.sdf` 仓库场景和
`localization_launch.py` 的 AMCL。TF 所有权保持为：AMCL 发布 `map -> odom`，
Gazebo 发布 `odom -> base_footprint`，Robot State Publisher 发布机器人内部静态 TF；
固定路径模式不再启动 `fixed_path_localization_launch.py` 的静态 `map -> odom`。
Gazebo 出生位姿与 AMCL 参数中的初始位姿保持一致，并设置
`always_reset_initial_pose: true`，避免 Lifecycle 重启时沿用旧定位状态。

为避免 RViz 在 AMCL 和传感器 TF 尚未就绪时显示首帧，固定路径控制节点默认延迟
`3 s` 启动，RViz 默认延迟 `5 s` 启动。Gazebo 原生 `gpu_lidar` 以 `base_scan`
为坐标系发布 `/scan`，再由 `ros_gz_bridge` 桥接给 AMCL、代价地图和 RViz；Gazebo
`VisualizeLidar` 可订阅同一雷达话题显示扫描射线。独立 Gazebo Server 使用
`--headless-rendering` 生成 GPU 雷达数据，图形客户端只负责显示。RViz 对 `/scan` 及
全局、局部体素点云只保留最新一帧，避免图形界面初始化期间积压旧时间戳数据并在 TF
就绪后集中重放。这两个延迟只影响启动显示和初始化顺序，不改变固定路径、速度或终端
控制参数。

每次启动还会按 `ROS_DOMAIN_ID` 和当前 Launch PID 自动生成唯一 `IGN_PARTITION`。
这样即使另一套 Gazebo 没有正常退出，新 Launch 的桥接也不会接入旧世界的 `/odom`、
`/clock` 和 `/odom/tf`。联调外部 Ignition 工具时可显式传入
`ign_partition:=myworld_debug`，并在工具终端设置同名 `IGN_PARTITION`。

自主与固定路径仿真模式启动 `controlpub`，仅把统一规控链的 `/cmd_vel` 转换为
`/control_to_uart`。`regulated_navigator` 仍监听 `/downstream/chassis_control`；没有
收到该话题时，不会由 ChassisControl 分支向 `/control_to_uart` 发布控制消息。

## 固定路径示例

固定模式 Gazebo 机器人出生 `x/y` 为 `(-2.8, -1.7)`。Action 到达后，
`regulated_navigator` 根据路径首段切线和 `motion_direction` 自动计算车辆车头：
前进使用 `node1 -> node2`，倒车使用其反向航向。两种 Action 都不调用 Gazebo
`set_pose`、不发布 `/initialpose`，车辆初始 `x/y/yaw` 均保持不变；`FixedPathController`
先检查 `0.70 m` 起点总距离安全门；首次横向误差不超过 `0.20 m` 且运动方向航向误差
严格小于 `15°` 时直接跟踪，其他情况先原地对齐到 `7°` 内并满足停稳与稳定周期要求。
前进以车头、后退以车辆反向与路径切线计算航向误差，随后分别输出正／负线速度。固定路径
旋转上限为 `0.4 rad/s`、角加速度上限为 `0.8 rad/s²`。原仓库世界中的斜向视觉通道
中心线左右边界各为 `0.40 m`。

固定路径要求 Action `max_speed` 为有限正数，`SpeedLimit` 只传递正的绝对速度上限；
`motion_direction=1`／`2` 由全部路径点姿态编码为前进／倒车车体朝向，控制器据此输出
正／负线速度。终点车头航向同样按末段路径切线和运动方向计算后交给
`stopped_goal_checker` 比较。固定路径分支由统一的 `regulated_navigator` 处理。启动
固定路径模式并等待 Lifecycle 节点进入 active 后发送：

仿真中的 Velocity Smoother 直接从标准 `/odom` 读取闭环速度，避免额外接口转换导致
反馈缺失后输出长期停在单周期增量；实车 `/motion_state` 仍由 `OdomSmoother` 原生支持，
保持 `v_car -> linear.x`、`w_car -> angular.z` 的映射，不切换为 OPEN_LOOP。

```bash
ros2 action send_goal --feedback \
  /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: 'myworld_fixed_path_001', navi_segment: [{segment_type: 1, segment_name: 'diagonal_corridor', segment_id: 'segment_001', node1: {x: -2.8, y: -1.7, z: 0.0}, node2: {x: 1.91, y: -4.83, z: 0.0}, control_pos1: {x: -0.933333, y: -2.933333, z: 0.0}, control_pos2: {x: 0.933333, y: -4.166667, z: 0.0}, max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0}]}"
```

检查 Action、Lifecycle 和速度链：

```bash
ros2 action info /navigation_service
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
ros2 param get /regulated_navigator operation_mode
ros2 topic echo /fixed_path_plan --once
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

要验证同一条路径前进，只需把上面 Action 中的 `task_id` 改成新的任务号，并将
`motion_direction: 2` 改为 `motion_direction: 1`；Action 不改写车辆初始位姿，控制器
先原地旋转到 `node1 -> node2` 的起点航向，再以正速度前进。倒车保持
`motion_direction: 2`，控制器先原地旋转到起点切线反方向，再以负速度跟踪，并在终点
切线反方向停车。

最终验收应以 `map -> base_footprint` 的误差为准，不要用存在 `map -> odom` 偏置的
原始 `/odom` 位置直接替代地图坐标。需要检查底盘闭环时，另行查看
`/downstream/chassis_control`、`/motion_state` 和 `/control_to_uart`。

一个 Action 可以包含多段连续路径，但当前要求所有分段使用相同 `motion_direction`；
整条路径限速取各段正 `max_speed` 的最小值。前进／倒退混合路径会被明确拒绝。
