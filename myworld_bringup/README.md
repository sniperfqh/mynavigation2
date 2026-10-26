# myworld_bringup

This package owns the `myworld2` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 当前入口与前提

[entry.launch.py](launch/entry.launch.py) 提供机器人内部 TF、三模式规控包装及可选的 Ignition Gazebo 仿真和传感器桥接；当前不启动 AMCL，不发布 Gazebo 底盘定位 TF。外部定位须提供新鲜 `map→base_link`，本包不替代定位。

两种导航模式共用同一张地图、机器人 TF 外参、雷达和速度反馈接口；任务输入、规划／控制流程与碰撞开关按模式选择：

| 项目 | 自主模式 `autonomous` | 固定路径模式 `fixed_path` |
|---|---|---|
| 任务输入 | `/navigate_to_pose`、`/navigate_through_poses` 或 `/goal_pose` | `/navigation_service`，接收业务分段路径 |
| 任务生成的路径 | `GridBasedAstar` 规划，可选路径平滑 | 按 `NaviSegment[]` 生成直线／三次贝塞尔，不调用全局规划或路径平滑 Action |
| 实际跟踪控制器 | `MPPI`，100 Hz、`model_dt=0.01` | `FixedPathController`，100 Hz |
| 目标检查器 | `stopped_goal_checker`，默认位置容差 0.03 m | `fixed_path_goal_checker`，默认位置容差 0.01 m |
| 地图与定位 | `/userdata/map/pbstream/out.yaml`；外部 Cartographer 提供 `map→base_link` | 相同 |
| 仿真与显示 | 后台 Gazebo 服务端、传感器桥接、速度转换、Nav2、RViz2；不打开 Gazebo 窗口 | 相同 |
| 外部 `/collision_state` 门控 | 关闭，`enable_collision_state` 仅对固定路径生效 | 默认启用 |
| Collision Monitor | 默认关闭 | 默认关闭 |
| 碰撞区域可视化 | 默认关闭 | 默认开启；可视化不产生碰撞状态 |

这里的控制器和容差取自默认 [myworld2.yaml](params/myworld2.yaml) 与入口覆盖；显式参数文件或启动参数可改变实际值。两种模式共用外围配置，任务处理流程仍不同，不能只替换 Action 名称并复用另一种消息结构。

## 两种导航模式的启动

在 `nav2_ws` 加载本工作空间环境；首次使用先按[源码仓库构建说明](../README.md#目录与构建)完成构建，不叠加 `learningnav2_ws` 环境：

```bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

自主模式：

```bash
ros2 launch myworld_bringup entry.launch.py operation_mode:=autonomous
```

固定路径模式：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path headless:=true use_rviz:=true \
  enable_collision_state:=true use_collision_monitor:=false
```

`operation_mode` 默认 autonomous，也可选 fixed_path 或 remote。自主／固定路径委托 nav2_regulated_modules；remote 在 `use_gazebo:=true` 时使用 ChassisControl→Twist 仿真网关，不启动导航控制链。已有同场景运行时不重复启动。

在另一个已加载本地 Cartographer 工作空间环境的终端，按原定位方式单独启动 Cartographer；其地图／pbstream 须与上述地图坐标系一致。Gazebo 服务端和桥接提供 `/scan`、`/odometry`，转换节点提供 `/motion_state`，Cartographer 消费定位数据后发布 `map→base_link`。启动顺序可使两侧先等待数据，发送任务前应确认传感器、TF 和规控生命周期均就绪。

模式启动后不能热切换。切换时先退出旧导航／仿真入口，再启动另一模式，避免重复 Gazebo、桥接、控制器和 RViz；原有 Cartographer 的地图、TF 和时间基准仍需与本次仿真一致。

默认自主模式直接使用 [myworld2.yaml](params/myworld2.yaml) 中的 MPPI：100 Hz 控制、0.01 s 模型步长、150 步／500 条候选、1.5 s 预测，前进速度上限 1.5 m/s。fixed_path 仍选择 FixedPathController，remote 不启动导航链。入口不再读取、合并或生成 `noamcl_*.yaml`，AMCL 加载保持禁用；显式传入其他完整 `params_file` 时，需在文件中配置好坐标系与反馈话题。`myworld2_mppi.yaml` 仅保留为参数参考，不再自动加载。

自主模式默认启动 Nav2、RViz2、后台 Gazebo 服务端、传感器桥接和 `/odometry`→`/motion_state` 转换，不打开 Gazebo 窗口，不启动碰撞监控或碰撞区域可视化。固定路径仿真保持原行为。激光与速度反馈由仿真提供，`map→base_link` 定位 TF 仍由用户单独启动的本地 Cartographer 提供。需要打开 Gazebo 窗口时使用：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=autonomous use_gazebo:=true headless:=false use_rviz:=true
```

## 默认资源与参数覆盖

| 项目 | entry 当前生效选择 |
|---|---|
| 地图 | autonomous／fixed_path 共用 `/userdata/map/pbstream/out.yaml`；remote 使用 models/myworld2/myworld2.yaml，可用 `map` 覆盖 |
| 世界 | models/myworld2/world_only.sdf，可用 `world_file` 覆盖 |
| 参数 | 直接传递 params/myworld2.yaml，可用完整 `params_file` 覆盖；不生成 noamcl 临时参数文件 |
| 时间 | `use_sim_time=false`，使用系统时间；设 true 恢复仿真时钟桥接 |
| 界面 | 各模式均默认 `use_gazebo=true`、`headless=true`，启动后台仿真且不打开 Gazebo 窗口；导航模式默认 `use_rviz=true` |
| collision | fixed_path 默认启用外部状态门控与区域可视化；自主模式门控和可视化关闭，Collision Monitor 各模式均默认 false |
| 启动延迟 | autonomous／fixed_path 栈共用 `navigation_stack_start_delay`，默认沿用 `fixed_path_stack_start_delay=3.0`；RViz 5 s；延迟不保证外部定位已就绪 |

`map:=...` 对两种导航模式均可显式覆盖默认地图。`ign_partition` 默认按 ROS_DOMAIN_ID 与启动 PID 生成；外部 Ignition 工具需连接相同 partition。只有显式设置 `use_gazebo:=false` 时，才关闭仿真与桥接，并要求外部提供 `/scan`、`/odometry`、`/motion_state`；Cartographer 本身不生成这些传感器与速度数据。

## 反馈与速度链

```text
Gazebo /odom → ROS 桥接重映射 → /odometry (Odometry)
  → odom_to_motion_state.py → /motion_state (MotionState，继承 Header)
外部定位 → map→base_link；Robot State Publisher → 机器人内部静态 TF
autonomous: NavigateToPose／NavigateThroughPoses／goal_pose
  → regulated_navigator → Planner → 路径 Smoother → FollowPath
fixed_path: NavigationService(NaviSegment[]) → 业务 Path → FollowPath
FollowPath → Controller Server → /cmd_vel_nav
  → velocity_smoother → /cmd_vel → Gazebo DiffDrive
  → controlpub → /control_to_uart
```

控制器反馈和导航器旧速度日志的话题为 `/odometry`；平滑器与底盘闭环读取 `/motion_state`，导航器的旧速度日志仅在对应日志开关启用时订阅。ROS 侧话题直接配置在 `params/myworld2.yaml`，桥接负责将 Gazebo `/odom` 重映射为 ROS `/odometry`；入口不再读取 YAML 并动态修改话题。默认控制／平滑频率及 Gazebo 里程计配置为 100 Hz，实际墙钟接收频率需要单独测量。

系统时间模式重写桥接传感器和必要时间戳，不发布仿真 `/clock`；暂停 Gazebo 时系统时间上的超时继续。外部定位必须使用一致时间基准。定位跳变取消保护默认 false，TF 丢失保护仍存在。

## 自主路径任务

默认自主模式使用 `GridBasedAstar`（NavFn A*）规划，`simple_smoother` 平滑路径，再由 MPPI 跟踪。MPPI 的 0.01 s 模型步长对应 100 Hz 控制配置，150 步预测 1.5 s、500 条候选；配置频率不等于实测运行频率。

| 接口 | ROS 类型 | 输入内容 |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 单个 `pose`，使用地图坐标系 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 有序的 `poses[]` |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 导航目标或外部目标话题 |

发送目标前确认 `header.frame_id=map`、目标在当前地图可通行区域，并确认定位 TF 新鲜。可用 RViz 的导航目标工具或自己的 Action 客户端下发任务；消息字段见 [NavigateToPose.action](../nav2_msgs/action/NavigateToPose.action) 和 [NavigateThroughPoses.action](../nav2_msgs/action/NavigateThroughPoses.action)。固定路径的 `navi_segment`、`motion_direction`、`max_speed` 属于另一套 Action 协议，不能作为这些自主 Action 的字段。

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

[collision Action 例子](../nav2_regulated_modules/tools/COLLISION_TEST.md) 面向固定路径外部状态门控，以 50 Hz 独立周期发布状态，可设置两程前进／倒车。工具端点来自实验，需按地图核对。首次无状态默认为 4；收到后的断流达到 0.5 秒停车，不等同于 Collision Monitor 的传感器保护；自主模式默认不使用该状态门控。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
ros2 param get /regulated_navigator operation_mode
ros2 param get /controller_server odom_topic
ros2 param get /velocity_smoother odom_topic
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --field header
ros2 topic info /collision_state --verbose
ros2 topic echo /odometry --once
ros2 topic echo /motion_state --once
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link base_scan
```

`/collision_state` 的发布者检查用于固定路径门控；两个 `tf2_echo` 命令会持续输出，检查后用 Ctrl+C 退出。当前 RViz 雷达显示订阅 `sensor_msgs/msg/LaserScan` 的 `/scan`，并以点样式绘制，默认显示坐标系为 `map`。

| 现象 | 优先核对 |
|---|---|
| 雷达点不显示 | `/scan` 是否有发布者和新鲜消息；`use_gazebo` 是否被显式关闭；后台服务端与 `simulation_sensor_bridge` 是否都在运行 |
| 雷达有消息但无法显示在 map 下 | `map→base_link` 与 `base_link→base_scan` 是否连通，消息 frame 与时间戳是否匹配 |
| Nav2 等待 `map` TF | 本地 Cartographer 是否已启动并收到雷达／里程计；定位地图是否匹配、时钟是否一致 |
| RViz 有地图但缺少 map 定位 TF | Map Server 发布 `/map` 不会自动生成 `map→base_link`，仍需 Cartographer 定位 |

UART 空闲时可能不周期发零速，停稳需要新鲜轮速等证据。统一日志参数见[速度日志](../nav2_regulated_modules/doc/velocity_logging.md)。独立状态源启停的最新验收未完成，不能由本 README 推断通过。

场景资源见 [models/](models/)、[params/](params/)、[urdf/](urdf/)、[rviz/](rviz/)，依赖与安装见 [package.xml](package.xml)、[CMakeLists.txt](CMakeLists.txt)。
