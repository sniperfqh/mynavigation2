# myworld_bringup

This package owns the `myworld2` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, and parameter assets in this
  package.
- Use `entry.launch.py` for navigation and `simulation_only.launch.py` for simulation only.

## 当前入口与前提

[entry.launch.py](launch/entry.launch.py) 提供机器人内部 TF、三模式规控包装及可选的 Ignition Gazebo 仿真和传感器桥接；当前不启动 AMCL，不发布 Gazebo 底盘定位 TF。外部定位须提供新鲜 `map→base_link`，本包不替代定位。

该入口的雷达、IMU、万向轮和底盘固定关系由 `robot_state_publisher` 发布到 `/tf_static`；左右驱动轮读取 `/joint_states`，通过 `/tf` 发布动态旋转关系，RViz 可随关节状态显示车轮转动。外部定位仍负责动态 `map→base_link`。外部定位若也发布同名内部 TF，仍需分别统一其发布者和父坐标系。

两种导航模式共用同一张地图、机器人 TF 外参、雷达和速度反馈接口；任务输入、规划／控制流程与碰撞开关按模式选择：

| 项目 | 自主模式 `autonomous` | 固定路径模式 `fixed_path` |
|---|---|---|
| 任务输入 | `/navigate_to_pose`、`/navigate_through_poses` 或 `/goal_pose` | `/navigation_service`，接收业务分段路径 |
| 任务生成的路径 | `SmacHybrid` 规划，可选路径平滑 | 按 `NaviSegment[]` 生成直线／三次贝塞尔，不调用全局规划或路径平滑 Action |
| 实际跟踪控制器 | `MPPI` ID 对应自主转向／接近包装，内部为 MPPI，50 Hz、`model_dt=0.02` | `FixedPathController`，100 Hz |
| 目标检查器 | `stopped_goal_checker`，位置容差 0.03 m，连续达标停稳 0.5 s | `fixed_path_goal_checker`，默认位置容差 0.01 m |
| 地图与定位 | `/userdata/map/pbstream/out.yaml`；外部 Cartographer 提供 `map→base_link` | 相同 |
| 仿真与显示 | 后台 Gazebo 服务端、传感器桥接、速度转换、Nav2、RViz2；不打开 Gazebo 窗口 | 相同 |
| 外部 `/collision_state` 门控 | 关闭，`enable_collision_state` 仅对固定路径生效 | 默认启用 |
| Collision Monitor | 默认关闭 | 默认关闭 |
| 碰撞区域可视化 | 默认关闭 | 默认开启；可视化不产生碰撞状态 |

这里的控制器和容差取自默认 [myworld2.yaml](params/myworld2.yaml) 与入口覆盖；显式参数文件或启动参数可改变实际值。两种模式共用外围配置，任务处理流程仍不同，不能只替换 Action 名称并复用另一种消息结构。

## 仅启动仿真和传感器

使用 [simulation_only.launch.py](launch/simulation_only.launch.py) 加载 `myworld2/world_only.sdf` 仓库环境和差速车模型；默认打开 Gazebo 和 RViz，并通过静态地图服务发布对应的占据栅格，不启动规划、控制、Cartographer 或 AMCL：

```bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch myworld_bringup simulation_only.launch.py
```

Gazebo 可通过 `headless:=true` 在后台运行；不需要 RViz 时可传入 `use_rviz:=false`。该入口提供以下仿真接口：

| 接口 | 类型与用途 |
|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan`，雷达扫描，坐标系 `base_scan` |
| `/scan/points` | `sensor_msgs/msg/PointCloud2`，同一仿真雷达的点云 |
| `/imu`、`/joint_states` | 仿真 IMU 和左右轮关节状态 |
| `/odometry` | Gazebo 里程计 `nav_msgs/msg/Odometry` |
| `/motion_state` | 由 `/odometry` 转换的车辆线速度和角速度；升降及货架字段为零 |
| `/map` | 来自 `models/myworld2/myworld2.yaml` 的静态占据栅格 |
| `/cmd_vel` | 可选的 `geometry_msgs/msg/Twist` 速度输入，由 Gazebo 差速底盘接收 |

默认桥接 Gazebo 的 `odom → base_footprint`，并发布仿真用的单位变换 `map → odom`；机器人模型继续发布 `base_footprint → base_link → base_scan` 等内部 TF。静态地图由 Map Server 加载、启动文件管理其生命周期，不执行定位或建图。默认 RViz 使用 [simulation_only.rviz](rviz/simulation_only.rviz) 显示地图、车体和 `/scan`；点云 `/scan/points` 可在显示列表中启用。默认 Fixed Frame 为 `map`；关闭地图时改为 `odom`，关闭仿真里程计 TF 时改为 `base_link`，也可用 `rviz_fixed_frame` 指定其他已有坐标系。关闭仿真里程计 TF 后，地图和车体之间还需要外部定位 TF 才能同时显示。

默认 `use_sim_time:=false`，桥接消息使用系统时间；设置 `use_sim_time:=true` 时桥接 `/clock` 并使用 Gazebo 时间，其他消费节点须使用相同时间基准。`world_file` 和 `map` 可分别覆盖世界及地图文件，二者应使用一致的世界坐标；`publish_map:=false` 关闭内置地图及 `map → odom`，`ign_partition` 可覆盖默认的独立 Gazebo 分区。当前车辆 SDF 没有相机传感器，保留的相机桥接接口不会自行产生图像。

若外部定位直接发布 `map → base_link`，可设置 `publish_odom_tf:=false`，关闭 Gazebo 里程计 TF 并采用 `base_link` 为模型根节点；外部系统提供 `/map` 时另设 `publish_map:=false`，同时检查外部模型不要重复发布同名内部 TF。独立仿真与 `entry.launch.py` 不应在同一 ROS 域重复运行，以免多个发布者使用相同话题和坐标系。

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

默认自主模式在 [myworld2.yaml](params/myworld2.yaml) 基础上，通过规控入口既有 RewrittenYaml 应用 `autonomous_mppi_profile=true`：50 Hz 控制／速度平滑／默认诊断、0.02 s 模型步长、75 步／300 条候选、1.5 s 预测，前进速度上限为 0.5 m/s。仅自主模式选择 `AutonomousMPPIController` 和连续停稳检查；fixed_path 仍选择原 FixedPathController、原参数与原 GoalChecker，remote 不启动导航链。入口不生成 `noamcl_*.yaml`，AMCL 保持禁用；自定义参数文件需配置坐标系与反馈话题，可用 `autonomous_mppi_profile:=false` 关闭自主 MPPI 包装配置。该开关不关闭自主模式的 50 Hz／MPPI 时间网格、TF 时效和进度修复。`myworld2_mppi.yaml` 仅保留为历史参数参考，不再自动加载。

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

控制器反馈和导航器旧速度日志的话题为 `/odometry`；平滑器与底盘闭环读取 `/motion_state`，导航器的旧速度日志仅在对应日志开关启用时订阅。ROS 侧话题直接配置在 `params/myworld2.yaml`，桥接负责将 Gazebo `/odom` 重映射为 ROS `/odometry`；入口不再读取 YAML 并动态修改话题。自主模式控制器、速度平滑器和默认诊断快照为 50 Hz；固定路径模式保持原设置，Gazebo 里程计配置保持 100 Hz，实际墙钟接收频率需要单独测量。

系统时间模式重写桥接传感器和必要时间戳，不发布仿真 `/clock`；暂停 Gazebo 时系统时间上的超时继续。外部定位必须使用一致时间基准。定位跳变取消保护默认 false，TF 丢失保护仍存在。

## 自主路径任务

自主模式 RViz 默认启用 `MPPI Trajectories`，订阅 `/trajectories`（`visualization_msgs/msg/MarkerArray`），显示 MPPI 采样候选及优化轨迹；仅在 MPPI 跟踪计算时更新，起步旋转或末端专用控制阶段可能保留上次轨迹。默认每 15 条候选取一条、每 5 个时间点取一点，以减少可视化负载；轨迹可视化仍可能影响 50 Hz 控制周期。固定路径仍使用原 RViz 配置。

默认自主模式通过入口覆盖导航器规划器 ID，使用 `SmacHybrid`（Hybrid-A*）规划，`simple_smoother` 平滑路径，执行“起步转向对齐 → MPPI 跟踪 → 末端位置收敛 → 制动停稳 → 终点朝向对齐 → 连续停稳检查”。MPPI 的 0.02 s 模型步长对应 50 Hz 控制配置，75 步预测 1.5 s、300 条候选；配置频率不等于实测运行频率。

自主包装复用 RotationShim，内部明确选择 MPPI；距离终点 1 m 内连续降低 MPPI 速度约束，进入 0.15 m 后由专用末端位置控制接管，超过 0.25 m 才退回 MPPI。末端线速度／角速度上限默认分别为 0.05 m/s／0.1 rad/s，可由参数文件覆盖；朝向位置目标误差 >20° 时先原地对齐，<10° 后前进。进入位置容差的一半（默认 1.5 cm）后制动，轮速连续 0.2 s 达标再对齐终点航向；末端旋转根据当前角速度预留制动角度，越过目标后先给零速，待角速度减小后再修正。漂出 3 cm 后重新收敛位置。位置仍需 ≤3 cm、航向约 ≤9°、线／角速度 ≤0.01 m/s／0.05 rad/s，且连续达标 0.5 s；没有放宽容差或锁存失效的位置精度。末端前进、旋转和实际轮速下的制动路径都执行足迹碰撞预测，普通 MPPI 碰撞代价项仍生效；实际超调量需结合底盘验证。

自主模式同时修复两层进度判定：Controller 使用 PoseProgressChecker，Navigator 识别位置移动、朝向误差下降及末端距离改善。新任务或真正恢复执行重建进度基准，周期换路径不重置卡死时间。距终点 0.3 m 内暂停普通周期重规划，失败后的恢复仍可重规划。自主 TF 按源时间戳检查 0.3 s 时效，失效停车并等待连续稳定后恢复；固定路径相关逻辑保持原实现。以上实现只经过定向构建和非运动检查，运行效果需另行验证。

自主末端插件支持以下 YAML 参数，放在 `controller_server.ros__parameters.MPPI` 下；自主包装会保留显式配置。进入距离必须大于目标位置容差、小于 `autonomous_approach_distance`，三个参数均须为有限正数。

| 参数 | 默认值 | 用途 |
|---|---|---|
| `autonomous_terminal_distance` | 0.15 m | 进入专用位置控制；退出距离为该值加 0.1 m |
| `autonomous_terminal_max_linear_velocity` | 0.05 m/s | 末端前进上限，仍受外部限速约束 |
| `autonomous_terminal_max_angular_velocity` | 0.1 rad/s | 自主包装默认的末端位置及终点航向对齐转速上限 |

启动摘要分别显示控制／平滑／诊断频率；阶段变化立即打印 `Autonomous terminal phase=...`，同阶段摘要最多每秒一次。诊断快照频率不代表控制器实际发布频率，普通零速仍由 S 曲线平滑制动。

自主起步转向使用 Controller Server 已获取的同一帧机器人位姿计算路径朝向，不再给采样路径点写 `now()` 后重复查询 `map→base_link`。同坐标系直接计算几何关系；跨坐标系使用机器人位姿时间戳进行零等待查询，缓存不足则报告控制失败，由 Controller Server 输出零速。这样去除原 RotationShim 默认 0.1 s 的 TF 等待，避免轻微未来外推拖垮 50 Hz 的 20 ms 周期；Navigator 原有的 0.3 s 定位时效保护仍保留。该修复不保证消除 CPU、代价地图等待等其他原因的错周期，未修改共享 TF 工具或固定路径逻辑。

| 接口 | ROS 类型 | 输入内容 |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 单个 `pose`，使用地图坐标系 |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 有序的 `poses[]` |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 导航目标或外部目标话题 |

发送目标前确认 `header.frame_id=map`、目标在当前地图可通行区域，并确认定位 TF 新鲜。可用 RViz 的导航目标工具或自己的 Action 客户端下发任务；消息字段见 [NavigateToPose.action](../nav2_msgs/action/NavigateToPose.action) 和 [NavigateThroughPoses.action](../nav2_msgs/action/NavigateThroughPoses.action)。固定路径的 `navi_segment`、`motion_direction`、`max_speed` 属于另一套 Action 协议，不能作为这些自主 Action 的字段。

自主 Action 的 `distance_remaining` 使用“下游路径估计”和“当前请求目标的直线距离”的较大值，防止末端路径采样提前归零；ETA 包含转向及停稳估计，不参与成功判断。真正成功仍需位置、航向、轮速连续达标。`controller_server.log` 的 `Autonomous arrival waiting=POSITION/HEADING/VELOCITY/SETTLING` 可用于定位等待条件，`number_of_recoveries` 同时统计控制恢复和定位恢复。`nanosec: 0` 本身不是错误，应与 Duration 的 `sec` 一起读取。

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
