# myworld_bringup

This package owns the `myworld2` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 当前入口与前提

[entry.launch.py](launch/entry.launch.py) 提供 Ignition Gazebo 仿真、机器人内部 TF、传感器桥接与三模式规控包装；当前不启动 AMCL，不发布 Gazebo 底盘定位 TF。外部定位须提供新鲜 `map→base_link`，本包不替代定位。

先在 `nav2_ws` 构建并加载环境，再运行一个入口：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path headless:=true use_rviz:=true \
  enable_collision_state:=true use_collision_monitor:=false
```

`operation_mode` 默认 autonomous，也可选 fixed_path 或 remote。自主／固定路径委托 nav2_regulated_modules；remote 使用 ChassisControl→Twist 仿真网关，不启动导航控制链。已有同场景运行时不重复启动。

## 默认资源与参数覆盖

| 项目 | entry 当前生效选择 |
|---|---|
| 地图 | fixed_path 使用 `/userdata/map/pbstream/out.yaml`；其他模式使用 models/myworld2/myworld2.yaml，可用 `map` 覆盖 |
| 世界 | models/myworld2/world_only.sdf，可用 `world_file` 覆盖 |
| 参数 | params/myworld2.yaml，可用 `params_file` 覆盖，再由入口生成外部定位适配参数 |
| 时间 | `use_sim_time=false`，使用系统时间；设 true 恢复仿真时钟桥接 |
| 界面 | `headless=true`，`use_rviz=true` |
| collision | fixed_path 默认启用外部状态门控；Collision Monitor 默认 false、区域可视化默认 true |
| 启动延迟 | fixed_path 栈 3 s、RViz 5 s；延迟不保证外部定位已就绪 |

地图参数声明的描述字符串仍提到 myworld3，但实际默认表达式为上表的 `/userdata/map/pbstream/out.yaml`，本文以执行代码为准。`ign_partition` 默认按 ROS_DOMAIN_ID 与启动 PID 生成；外部 Ignition 工具需连接相同 partition。

## 反馈与速度链

```text
Gazebo /odom → ROS 桥接重映射 → /odometry (Odometry)
  → odom_to_motion_state.py → /motion_state (MotionState，继承 Header)
外部定位 → map→base_link；Robot State Publisher → 机器人内部静态 TF
NavigationService → regulated_navigator → FollowPath → /cmd_vel_nav
  → velocity_smoother → /cmd_vel → Gazebo DiffDrive
  → controlpub → /control_to_uart
```

控制器和导航器读取 `/odometry`；平滑器闭环读取 `/motion_state`。入口适配将旧 `/odom` 参数统一到新话题，不再以旧 README 的 `/odom` 作为 ROS 侧反馈。默认控制／平滑频率及 Gazebo 里程计配置为 100 Hz，实际墙钟接收频率需要单独测量。

系统时间模式重写桥接传感器和必要时间戳，不发布仿真 `/clock`；暂停 Gazebo 时系统时间上的超时继续。外部定位必须使用一致时间基准。定位跳变取消保护默认 false，TF 丢失保护仍存在。

## 固定路径任务

`/navigation_service` 类型为 `byd_custom_msgs/action/NavigationService`。直线与贝塞尔的 `max_speed` 均为有限正数；同任务所有段方向一致，1 前进、2 倒车，速度符号由方向决定。任务不重置机器人位姿。

起点距离、航向与起步保护可能导致先转向或低速接近，不能假定 Action 接受后立即达到 1.5 m/s。当前 fixed_path 使用专用目标检查器，终点航向只作诊断；停车入口、目标容差与独立定位精度分开评价。具体参数以传入 YAML 与 Launch 重写为准，见[规控包](../nav2_regulated_modules/README.md)。

以下单段命令保留旧示例字段，**坐标来自历史地图**，发送前须核对当前地图、通行区域、定位和起点，不作为当前出生位姿说明：

```bash
ros2 action send_goal --feedback /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: 'myworld_fixed_path_001', navi_segment: [{segment_type: 1, segment_name: 'diagonal_corridor', segment_id: 'segment_001', node1: {x: -2.8, y: -1.7, z: 0.0}, node2: {x: 1.91, y: -4.83, z: 0.0}, control_pos1: {x: -0.933333, y: -2.933333, z: 0.0}, control_pos2: {x: 0.933333, y: -4.166667, z: 0.0}, max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0}]}"
```

### 三段连续路径：直线—曲线—直线

下面的独立 Action 沿用上例的仿真起点 `(-2.8, -1.7)`、终点 `(1.91, -4.83)`、
`0.20 m/s` 限速与后退方向。第一段直线在 `(-1.4, -2.4)` 接入三次贝塞尔曲线，
曲线在 `(0.5, -3.8)` 接入末段直线；相邻段端点相同，两个曲线控制点分别沿
前后直线的切线方向布置，避免连接处出现几何折角。先启动上述 `fixed_path` 模式，
确认机器人仍在首段 `node1` 附近且相关 Lifecycle 节点已进入 `active`，再在另一终端发送：

```bash
ros2 action send_goal --feedback \
  /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: 'myworld_line_curve_line_001', navi_segment: [
    {segment_type: 1, segment_name: 'line_start', segment_id: 'segment_001',
     node1: {x: -2.8, y: -1.7, z: 0.0}, node2: {x: -1.4, y: -2.4, z: 0.0},
     control_pos1: {x: 0.0, y: 0.0, z: 0.0}, control_pos2: {x: 0.0, y: 0.0, z: 0.0},
     max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0},
    {segment_type: 2, segment_name: 'bezier_middle', segment_id: 'segment_002',
     node1: {x: -1.4, y: -2.4, z: 0.0}, node2: {x: 0.5, y: -3.8, z: 0.0},
     control_pos1: {x: -0.7, y: -2.75, z: 0.0}, control_pos2: {x: -0.205, y: -3.285, z: 0.0},
     max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0},
    {segment_type: 1, segment_name: 'line_end', segment_id: 'segment_003',
     node1: {x: 0.5, y: -3.8, z: 0.0}, node2: {x: 1.91, y: -4.83, z: 0.0},
     control_pos1: {x: 0.0, y: 0.0, z: 0.0}, control_pos2: {x: 0.0, y: 0.0, z: 0.0},
     max_load_speed: 0.0, max_speed: 0.20, motion_direction: 2, dwell_time: 0}
  ]}"
```

`segment_type: 1` 的控制点由路径生成代码根据直线起终点重新计算，因此这里填零；
`segment_type: 2` 按 `node1 → control_pos1 → control_pos2 → node2` 生成曲线。
该几何来自旧实验，不对应当前默认地图和出生位置；仅在核对地图与起始位姿后使用，不能通过重启假定机器人会回到这些坐标。
此命令仅提供协议与几何连续的输入示例，未据此验证地图通行或最终到点精度。


## 周期状态、日志与只读检查

[collision Action 例子](../nav2_regulated_modules/tools/COLLISION_TEST.md) 以 50 Hz 独立周期发布状态，可设置两程前进／倒车。工具端点来自实验，需按地图核对。首次无状态默认为 4；收到后的断流达到 0.5 秒停车，不等同于 Collision Monitor 的传感器保护。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
ros2 param get /regulated_navigator operation_mode
ros2 topic info /collision_state --verbose
ros2 topic echo /odometry --once
ros2 topic echo /motion_state --once
```

UART 空闲时可能不周期发零速，停稳需要新鲜轮速等证据。统一日志参数见[速度日志](../nav2_regulated_modules/doc/velocity_logging.md)。独立状态源启停的最新验收未完成，不能由本 README 推断通过。

场景资源见 [models/](models/)、[params/](params/)、[urdf/](urdf/)、[rviz/](rviz/)，依赖与安装见 [package.xml](package.xml)、[CMakeLists.txt](CMakeLists.txt)。
