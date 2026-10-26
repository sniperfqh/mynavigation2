# mynavigation2

本仓库是 nav2_demo 项目当前开发的 Nav2 源码，直接位于 `nav2_ws/src/navigation2/`，没有另一份工作空间源码副本。基于 ROS 2 Humble，包含标准 Nav2、无 BT 三模式规控、自定义底盘接口和仿真场景。

## 入口选择

| 用途 | 入口 | 定位与环境 |
|---|---|---|
| 推荐规控：remote／autonomous／fixed_path | [nav2_regulated_modules](nav2_regulated_modules/README.md) | 外部定位、传感器、轮速和底盘通信；不启动仿真 |
| 当前仓库场景仿真 | [myworld_bringup](myworld_bringup/README.md) | Ignition、桥接和三模式包装；定位须外部启动 |
| 标准 BT Nav2 兼容入口 | [myagv_test_bringup](myagv_test_bringup/README.md) | 默认面向实车、外部定位 |
| TurtleBot3 仿真 | [mytb3_bringup](mytb3_bringup/README.md) | 独立仿真时钟与场景配置 |
| 另一套 myworld 仿真 | [myworld_bringup_sim](myworld_bringup_sim/README.md) | 保留兼容入口，不等同当前三模式包装 |
| 键盘直接控制底盘 | [myagv_keyboard_control](myagv_keyboard_control/README.md) | 交互终端；直接输出 ControlRes |

模式启动后不能热切换。导航模式要有新鲜的定位 TF 与速度反馈；启动进程、发现 Action 或加载配置均不等于车辆已具备运行条件。

## 安装依赖

以下命令适用于 Ubuntu 22.04／ROS 2 Humble，执行前需已安装 ROS 2 Humble 并配置 ROS 2 官方 apt 软件源。

安装 Nav2 运行依赖：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

本仓库的 TEB Local Planner 还依赖 g2o；全量构建包含该包时先安装：

```bash
sudo apt install ros-humble-libg2o
```

碰撞监视器的激光／点云半径离群点过滤还使用 PCL 的 `common`、`filters` 模块，依赖已在 [nav2_collision_monitor/package.xml](nav2_collision_monitor/package.xml) 和 [CMakeLists.txt](nav2_collision_monitor/CMakeLists.txt) 声明；按下节通过 rosdep 安装对应依赖。

源码其余依赖按下节的 `rosdep install` 命令安装，依赖声明见各包的 `package.xml`。仿真另需 Ignition Gazebo 与 ros_gz 组件，入口说明见 [myworld_bringup](myworld_bringup/README.md) 和 [TurtleBot3 仿真](navigation2_ignition_gazebo_turtlebot3/README.md)。完成源码构建后，按下节加载本工作空间的 `install/local_setup.bash`。

## 目录与构建

源码含 55 个 ROS 包；54 个由当前 colcon 发现，nav2_system_tests 因 COLCON_IGNORE 默认跳过。嵌套 DWB 子包和 turtlebot3 按各自 package.xml 的名称构建，不能用仓库目录名代替包名。

命令从外层 nav2_demo 项目根目录开始；已在 nav2_ws 时跳过 cd。不在本源码仓库或包目录中运行 colcon，也不同时叠加 learningnav2_ws 环境。

```bash
cd nav2_ws
source /opt/ros/humble/setup.bash
# 首次准备依赖；需要已配置 ROS 2 / rosdep，实际安装由使用者执行。
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

仅构建规控及其工作空间依赖：

```bash
# 在 nav2_ws，先加载 /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to nav2_regulated_modules \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

依赖基线为 Ubuntu 22.04／ROS 2 Humble；仿真另需 Ignition Gazebo 与 ros_gz 组件，TEB 另依赖 g2o。具体依赖看各包 manifest 和构建声明。本轮未安装依赖；历史全量构建关闭了测试，不能据此声称所有测试通过。构建临时文件和 ROS 日志可通过 TMPDIR、ROS_LOG_DIR 指向可写数据盘，避免系统盘不足。

## 推荐运行与数据流

加载环境后运行一个入口，地图路径按部署覆盖：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous
```

改为 `operation_mode:=fixed_path` 接收分段固定路径，改为 `remote` 进入键盘底盘直控。直接规控入口的导航模式默认使用 `/userdata/map/pbstream/out.yaml`，该文件必须存在，或传 `map:=实际地图绝对路径`；remote 模式不执行地图规划。

直接规控入口默认读取 [regulated_modules.yaml](nav2_regulated_modules/params/regulated_modules.yaml) ：自主导航选择 `GridBasedAstar`（NavFn A*）＋`RPP`，固定路径选择 `FixedPathController`＋`fixed_path_goal_checker`。当前控制器和平滑器配置频率均为 100 Hz；配置值与节点实际运行频率需分别核验。仓库虽有 TEB 源码，但这份规控 YAML 的控制器列表未注册 TEB。

当前 `myworld_bringup entry.launch.py` 默认 `use_rviz:=true`、`use_sim_time:=false`，由外部定位提供 TF。自主／固定路径共用 `/userdata/map/pbstream/out.yaml` 地图、map／base_link 坐标系、反馈话题、RViz 配置和规控启动装配，导航栈默认均延迟 3 秒，RViz 延迟 5 秒。各模式均默认 `use_gazebo:=true`、`headless:=true`，启动后台 Gazebo 服务端、传感器桥接和速度反馈转换，不打开 Gazebo 窗口；自主模式的碰撞监控与区域可视化仍默认关闭。fixed_path 默认启用碰撞状态门控和区域可视化，Collision Monitor 仍默认关闭；remote 的默认地图为包内 `myworld2.yaml`，各模式均可通过 `map` 显式覆盖。显式 `use_gazebo:=false` 可改为接入外部 ROS 数据。入口直接传递 `params_file`，不再读写或生成 `noamcl_*.yaml`；坐标系与反馈话题直接配置在 `myworld2.yaml`，AMCL 保持禁用。

默认自主模式直接使用 [myworld2.yaml](myworld_bringup/params/myworld2.yaml) 中的 MPPI：控制频率 100 Hz、模型步长 0.01 s、150 步预测 1.5 s，前进速度上限 1.5 m/s。fixed_path 仍选择 FixedPathController，remote 不启动导航链；显式传入完整 `params_file` 时直接使用该文件，不再自动修改其坐标系或反馈话题。直接规控入口的实车参数文件仍默认使用 RPP。此处为启动配置，实际运行效果另行验收。

### 当前场景的自主与固定路径仿真

在 `nav2_ws` 加载 `/opt/ros/humble/setup.bash` 和 `install/local_setup.bash` 后，按任务选择以下一个入口。两种模式不能同时启动或热切换；切换前先退出旧导航／仿真入口。本地 Cartographer 在独立终端按原方式启动，提供新鲜 `map→base_link`，其定位地图须与 `/userdata/map/pbstream/out.yaml` 一致。

自主模式，使用 RViz 导航目标或标准导航 Action：

```bash
ros2 launch myworld_bringup entry.launch.py operation_mode:=autonomous
```

固定路径模式，使用业务分段路径 Action：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path headless:=true use_rviz:=true \
  enable_collision_state:=true use_collision_monitor:=false
```

| 项目 | 场景自主模式 | 场景固定路径模式 |
|---|---|---|
| 任务接口 | `/navigate_to_pose`、`/navigate_through_poses` 或 `/goal_pose` | `/navigation_service` |
| 路径与跟踪 | `GridBasedAstar` → 路径平滑 → MPPI | `NaviSegment[]` → 业务 Path → FixedPathController |
| 默认目标检查器 | `stopped_goal_checker`，位置容差 0.03 m | `fixed_path_goal_checker`，位置容差 0.01 m |
| 后台 Gazebo、桥接、速度转换 | 默认启动，不打开 Gazebo 窗口 | 相同 |
| RViz2／地图／时钟 | RViz2 开启；地图 `/userdata/map/pbstream/out.yaml`；系统时间 | 相同 |
| 外部碰撞状态／区域可视化 | 关闭 | 默认开启 |
| Collision Monitor | 默认关闭 | 默认关闭 |

仿真提供 `/scan`（LaserScan）、`/odometry`（Odometry），转换节点提供 `/motion_state`（MotionState）；Cartographer 提供定位 TF。Map Server 发布 `/map` 不会生成定位 TF。雷达点缺失时先查 `/scan` 发布者与桥接，Nav2 等待 `map` TF 时再查 Cartographer 的数据输入、TF 连通性与时间基准。显式 `use_gazebo:=false` 会同时关闭仿真、桥接和速度转换，使用者须另行提供对应 ROS 数据。

自主任务类型为 `nav2_msgs/action/NavigateToPose` 或 `NavigateThroughPoses`；固定路径为 `byd_custom_msgs/action/NavigationService`，两者的 Goal 字段不同。启动、任务接口、历史路径示例及只读排查命令见 [myworld_bringup/README.md](myworld_bringup/README.md)。本节描述场景配置，直接 `nav2_regulated_modules` 入口的参数文件和默认 RPP 控制器见上文。

```text
autonomous: 导航目标 → regulated_navigator → Planner → 可选 Smoother → FollowPath
fixed_path: NavigationService(NaviSegment[]) → 方向化 Path → FollowPath
FollowPath → Controller Server → /cmd_vel_nav → Velocity Smoother
  → /cmd_vel → controlpub → /control_to_uart
```

启用 Collision Monitor 后，平滑器先发 `/cmd_vel_collision_in`，监控过滤后发 `/cmd_vel`。标准 BT 入口由 bt_navigator 编排，是否调用 SmoothPath 取决于 BT。路径平滑与速度平滑是不同功能。

| 接口 | 类型／含义 |
|---|---|
| `/navigation_service` | `byd_custom_msgs/action/NavigationService`；仅 fixed_path 接受任务 |
| `/navigate_to_pose`、`/navigate_through_poses` | 标准 Nav2 导航 Action；由所选入口提供 |
| `/follow_path` | `nav2_msgs/action/FollowPath`；Controller Server |
| `/cmd_vel_nav`、`/cmd_vel` | Twist；控制输出与平滑／过滤后速度 |
| `/odometry` | `nav_msgs/msg/Odometry`；当前 myworld entry 为控制器提供的仿真反馈；直接规控导航器旧速度日志的默认反馈话题 |
| `/motion_state` | `byd_custom_msgs/msg/MotionState`；直接规控默认的控制器、平滑器与底盘闭环反馈；myworld entry 将仿真里程计转换后供平滑器与底盘闭环使用 |
| `/downstream/chassis_control` | ChassisControl；底盘指令支路，具体消费者随模式变化 |
| `/control_to_uart` | ControlRes；ROS 底盘输出，不代表本仓库实现 UART 驱动 |

以上为无命名空间入口约定，参数和 remapping 可改变实际名称。定位需提供 map→base_link 或完整等效 TF 链；合成 laserpub／map2base_tf 只用于隔离测试，当前生产入口不自动启动。

反馈类型要与消费者匹配：控制器的 `OdomSubscriber`、平滑器的 `OdomSmoother` 和统一诊断按话题末级名称是否为 `motion_state` 选择 MotionState／Odometry；MotionState 速度取 `v_car/w_car`。导航器的 `velocity_odom_topic` 则仍是固定 Odometry 类型，只在启用旧速度日志时订阅，不能把它直接改成 MotionState 话题。实际控制反馈以 `controller_server.odom_topic` 为准，平滑反馈以 `velocity_smoother.odom_topic` 为准。

## 固定路径、collision 与测试

NavigationService 支持直线／三次贝塞尔；各段同向，方向 1 前进、2 倒车，max_speed 始终为正。任务限速取各段最小值，再受 fixed_path_max_speed=1.5 限制。起步航向和速度保护可能先转向或低速跟踪；Action 成功与独立速度／定位验收分别记录。

固定路径直接使用业务分段生成的 Path，不调用全局规划或路径平滑 Action，也不自动搜索绕障路线。`motion_direction=2` 时按路径切线反向编码车头姿态，并以负线速度执行；`max_load_speed`、`dwell_time` 当前未在这条固定路径执行链中实现对应的限速／停留调度，不能仅凭消息字段存在就认为已生效。

当前直接规控参数的起步上限为 `start_offset_speed_limit=0.30 m/s`，仍受更低任务限速约束。保护距离按沿路径投影进度计算：至少从首次有效投影位置再前进 `start_speed_limit_distance=0.50 m`，并且横向／运动方向航向误差连续 10 个周期达标后才解除；这不是车辆到起点的直线距离。起步保护在同一路径解除后不重复开启。

终点参数与成功条件按当前源码分开理解：

- 直接规控默认 `goal_stop_entry_tolerance=0.008 m`；进入停车入口或满足末段越过终点平面条件后，控制器锁存零速并停止继续转向。位置精度仍按 `xy_goal_tolerance=0.01 m` 检查，实际轮速由下游执行链决定。
- `terminal_convergence_enabled=true` 时，优先要求当前位置精度、定位／速度反馈时效及停稳条件连续通过 1 秒复核；已锁停且新鲜反馈持续停稳 2 秒、定位仍新鲜时，也可经兜底返回成功并记录 WARN。因此必须同时检查误差、超限标志和复核日志。
- 直接规控默认终点减速度 `0.20 m/s²`、末段接近速度 `0.015 m/s`，动态制动裕量为低速 `0.010 m`／高速 `0.050 m`，过渡速度 `0.50 m/s`。仿真包装有独立参数与覆盖逻辑，不能把两套配置的调参结果直接混用。
- 终点基础前视为 `goal_terminal_lookahead_dist=0.20 m`，参考速度 `0.75 m/s`，速度增益 `0.10 s`，距离限制为 `0.15～0.25 m`；末端收敛开启时使用实测速度幅值计算。正常前视到末段前视的混合区间为距终点 `0.50～0.10 m`。
- 当前 `selectCarrot()` 仍从最近离散路径点累计长度选点，路径不足前视距离时沿终点切线外推虚拟跟踪点。外推距离为“前视距离减去最近路径点到终点的剩余弧长，再取非负值”；它随位置与参数变化，不是固定延长 15 厘米，也不改变业务终点。连续投影／段内插值的优化方案尚未实现。

实现入口见 [固定路径控制器](nav2_regulated_modules/src/fixed_path_controller.cpp) 、[末端前视混合](nav2_regulated_modules/include/nav2_regulated_modules/detail/terminal_lookahead.hpp) 和 [Controller Server 到点复核](nav2_controller/src/controller_server.cpp) 。

外部 collision 门控在 fixed_path 默认启用，自主／遥控不启用。状态 4／3／2 分别为任务速度上限的 100%／65%／35%，1／0 停止；首次从未收到消息按 4，收到后非法值或断流达到 0.5 秒停车。恢复 4 只解除限制，不创建任务或恢复已取消任务。

Collision Monitor 与上述门控独立，直接规控和当前 myworld 入口 `use_collision_monitor` 均默认 false；区域可视化不产生碰撞状态。Launch 的 enable_collision_state 默认 true 与独立 C++ 节点 collision_state_enabled 的回退 false 属于不同配置层。

- [规控完整说明](nav2_regulated_modules/README.md)：模式、反馈、默认配置与门控状态表。
- [50 Hz collision Action 例子](nav2_regulated_modules/tools/COLLISION_TEST.md)：周期 4→3→2→1→0、1.5 m/s、往返倒车及结果分类。
- [测试工具](nav2_regulated_modules/tools/README.md)：现有环境观察与隔离仿真实验的区别。
- [分开验收](nav2_regulated_modules/doc/action_only_acceptance.md)：速度停稳、定位覆盖与精度独立评价。
- [固定路径架构](nav2_regulated_modules/doc/fixed_path_architecture.md)：既有模块职责与流程；其中旧的 Collision Monitor 默认开启描述以本文当前配置为准。

`collision_action_test.py` 已区分实车与仿真：默认 `--runtime sim` 使用 `/odometry`／Odometry，默认巡航触发、两程前进并持续运行；`--runtime real` 使用 `/motion_state`／MotionState，默认周期调度、去程前进返程倒车、四程。实车必须显式提供 `--a X Y --b X Y`，工具不替现场选择路线。`--feedback-topic` 可覆盖话题，反馈类型由 runtime 决定；`--odom-topic` 只供仿真兼容使用。

`--schedule periodic` 选择独立周期调度，发布调用目标频率为 50 Hz；五态保持时间默认仿真为 4／3／3／3／3 秒，实车为 12／4／4／3／3 秒，可用 `--state-durations` 覆盖。实际时序、各方向／状态响应和同一 Action 停后恢复分别记录，未覆盖的观察项标为 UNAVAILABLE。

截至本次文档核对，隔离真实节点配合合成反馈的测试、当前地图仿真测试、实车验证是不同证据层级。周期仿真仅取得部分减速／停止有效证据；独立状态源启停验收因 5 秒起步窗内一直转向而 UNAVAILABLE，断流停车和同任务恢复的完整两程验收仍未完成。历史原始报告由外层项目维护，本文不替代报告。

## 日志与排查

统一诊断默认文件快照 100 Hz、终端摘要 1 Hz，默认 `/tmp/nav2_logs`；可用 `log_dir` 指向数据盘。ROS Launch 日志由 ROS_LOG_DIR 管理。`enable_velocity_diagnostics` 默认 true，`enable_legacy_velocity_logs` 默认 false。详情见[速度日志](nav2_regulated_modules/doc/velocity_logging.md)。

`velocity … schema=2` 是各话题最新值的周期快照，同一话题／类型的多个角色会合并显示。`count` 重复表示没有新消息，`age` 是单调时钟计算的接收年龄；`status=valid` 只说明已收到、速度数值有限且未过期，不验证速度是否物理合理，也不证明停稳。各话题回调不是严格同步采样，不能把单行内的指令／反馈差值直接当作同一时刻的控制误差；`wz` 对应车体角速度，`w_shelf` 是另一物理量。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
ros2 param get /regulated_navigator operation_mode
ros2 param get /controller_server odom_topic
ros2 param get /velocity_smoother odom_topic
ros2 param get /velocity_smoother collision_state_enabled
ros2 topic info /collision_state --verbose
```

UART 适配节点在有效输入到来后才发布，空闲时可能静默；零输出、真实反馈停稳、定位质量及任务状态需分别核查。配置改变不自动更新已启动进程。

## 全部包索引

以下按 package.xml 的包名列出，嵌套目录保持真实结构。

| 包 | 类型与职责 | 构建状态 |
|---|---|---|
| [aws_robomaker_small_warehouse_world](aws_robomaker_small_warehouse_world/README.md) | 资源包：提供 AWS 小型仓库世界、模型与地图资源，用于仿真场景搭建；不实现导航控制器。 | colcon 可发现 |
| [byd_custom_msgs](byd_custom_msgs/README.md) | 接口包：定义底盘命令、运动反馈、分段路径和固定路径 Action，是规控与底盘之间的消息契约。 | colcon 可发现 |
| [controlpub](controlpub/README.md) | 适配节点：将 Twist 速度输入转换为 ControlRes，供外部底盘通信链路使用；本包不实现串口驱动。 | colcon 可发现 |
| [costmap_queue](nav2_dwb_controller/costmap_queue/README.md) | 算法库：提供代价地图栅格按距离展开的队列工具，服务于 DWB 距离场计算。 | colcon 可发现 |
| [dwb_core](nav2_dwb_controller/dwb_core/README.md) | 控制插件与核心库：实现 DWB 局部控制框架，组合轨迹生成器与评分器选择速度指令。 | colcon 可发现 |
| [dwb_critics](nav2_dwb_controller/dwb_critics/README.md) | 轨迹评分插件：提供障碍、路径、目标、朝向及振荡等评分器，由 DWB Controller 装载。 | colcon 可发现 |
| [dwb_msgs](nav2_dwb_controller/dwb_msgs/README.md) | 接口包：定义 DWB 轨迹、评分与局部规划评估相关消息和服务。 | colcon 可发现 |
| [dwb_plugins](nav2_dwb_controller/dwb_plugins/README.md) | 轨迹生成插件：提供 DWB 标准及受加速度约束的轨迹生成器，供 dwb_core 采样使用。 | colcon 可发现 |
| [laserpub](laserpub/README.md) | 测试节点：发布可配置的合成二维 LaserScan，供隔离功能验证使用；不能替代真实激光观测。 | colcon 可发现 |
| [map2base_tf](map2base_tf/README.md) | 测试节点：周期发布固定位置的 map→base_link 动态 TF，供隔离接口验证；不实现定位算法。 | colcon 可发现 |
| [myagv_keyboard_control](myagv_keyboard_control/README.md) | 遥控节点：读取终端按键，按速度、加速度和 jerk 约束生成 ControlRes 底盘命令。 | colcon 可发现 |
| [myagv_test_bringup](myagv_test_bringup/README.md) | 启动包：提供面向外部定位与真实底盘的标准 BT Nav2 兼容入口、参数和可视化资源。 | colcon 可发现 |
| [mytb3_bringup](mytb3_bringup/README.md) | 仿真启动包：组织 TurtleBot3 的 Ignition Gazebo、机器人描述、传感器桥接、定位与导航场景。 | colcon 可发现 |
| [myworld_bringup](myworld_bringup/README.md) | 仿真启动包：组织仓库场景仿真与三模式规控入口；当前 entry 使用外部定位和系统时间适配。 | colcon 可发现 |
| [myworld_bringup_sim](myworld_bringup_sim/README.md) | 仿真启动包：保留另一套 myworld 场景启动、定位与导航配置；入口行为独立于 myworld_bringup。 | colcon 可发现 |
| [nav2_amcl](nav2_amcl/README.md) | 定位节点：使用粒子滤波、运动模型与激光地图匹配估计二维位姿，向导航提供定位变换。 | colcon 可发现 |
| [nav2_behavior_tree](nav2_behavior_tree/README.md) | 行为树插件库：提供规划、控制、恢复、条件与流程控制 BT 节点，供 BT Navigator 装载。 | colcon 可发现 |
| [nav2_behaviors](nav2_behaviors/README.md) | 行为服务器：通过行为插件提供旋转、后退、等待、按航向行驶及辅助遥控等动作。 | colcon 可发现 |
| [nav2_bringup](nav2_bringup/README.md) | 标准启动包：提供标准 Nav2 的导航、定位、组合节点和示例仿真启动资源。 | colcon 可发现 |
| [nav2_bt_navigator](nav2_bt_navigator/README.md) | 导航协调节点：执行行为树，将导航 Action 编排为规划、控制与恢复任务；与本项目无 BT 协调器是不同入口。 | colcon 可发现 |
| [nav2_collision_monitor](nav2_collision_monitor/README.md) | 速度过滤节点：依据 LaserScan、PointCloud2、Range 等传感器与配置区域，对输入速度实施停车、减速或接近保护。 | colcon 可发现 |
| [nav2_common](nav2_common/README.md) | 构建与启动工具库：提供 Nav2 通用 CMake 宏与 Python Launch 参数重写工具，包括 RewrittenYaml。 | colcon 可发现 |
| [nav2_constrained_smoother](nav2_constrained_smoother/README.md) | 平滑插件：通过 Ceres 优化路径的平滑性、曲率和障碍物约束，由 Smoother Server 加载。 | colcon 可发现 |
| [nav2_controller](nav2_controller/README.md) | 控制服务器：实现 FollowPath Action，加载控制器、进度检查器和目标检查器，计算并发布速度。 | colcon 可发现 |
| [nav2_core](nav2_core/README.md) | 插件接口库：定义 GlobalPlanner、Controller、Smoother、Behavior、GoalChecker 等 Nav2 插件基类。 | colcon 可发现 |
| [nav2_costmap_2d](nav2_costmap_2d/README.md) | 代价地图库与节点：组合静态、障碍、体素、膨胀等层及代价过滤器，为规划与控制提供环境代价。 | colcon 可发现 |
| [nav2_dwb_controller](nav2_dwb_controller/nav2_dwb_controller/README.md) | 元包：聚合 DWB 控制器相关包的依赖，不提供独立运行节点。 | colcon 可发现 |
| [nav2_graceful_controller](nav2_graceful_controller/README.md) | 控制插件：通过平滑控制律跟踪路径，由 Controller Server 加载并生成速度。 | colcon 可发现 |
| [nav2_lifecycle_manager](nav2_lifecycle_manager/README.md) | 生命周期管理节点：按配置顺序启动、激活、停用和监视生命周期节点，并通过 bond 检测连接状态。 | colcon 可发现 |
| [nav2_map_server](nav2_map_server/README.md) | 地图服务节点：提供地图加载、发布、保存及 Costmap Filter 信息服务。 | colcon 可发现 |
| [nav2_mppi_controller](nav2_mppi_controller/README.md) | 控制插件：通过模型预测路径积分采样控制序列，并按 critics 评分选择路径跟踪动作。 | colcon 可发现 |
| [nav2_msgs](nav2_msgs/README.md) | 接口包：定义标准 Nav2 导航、规划、路径跟踪、平滑、行为及生命周期相关消息、服务和 Action。 | colcon 可发现 |
| [nav2_navfn_planner](nav2_navfn_planner/README.md) | 规划插件：提供栅格 Dijkstra／A* 路径搜索，由 Planner Server 加载。 | colcon 可发现 |
| [nav2_planner](nav2_planner/README.md) | 规划服务器：实现 ComputePathToPose／ComputePathThroughPoses Action，并管理全局规划插件与全局代价地图。 | colcon 可发现 |
| [nav2_regulated_modules](nav2_regulated_modules/README.md) | 规控协调与插件包：提供 remote、autonomous、fixed_path 三模式入口、无 BT 任务协调、专用固定路径控制器和诊断工具。 | colcon 可发现 |
| [nav2_regulated_pure_pursuit_controller](nav2_regulated_pure_pursuit_controller/README.md) | 控制插件：提供受曲率、代价与目标接近约束的 Pure Pursuit 路径跟踪控制器。 | colcon 可发现 |
| [nav2_rotation_shim_controller](nav2_rotation_shim_controller/README.md) | 控制插件：在需要时先对齐路径朝向，再把控制交给配置的内部控制器。 | colcon 可发现 |
| [nav2_route](nav2_route/README.md) | 路网服务器与插件：从路网图搜索路线，并通过边评分和路线操作插件处理路线任务；不同于栅格 Planner Server。 | colcon 可发现 |
| [nav2_rviz_plugins](nav2_rviz_plugins/README.md) | RViz 插件：提供导航目标工具、Nav2 面板与粒子云显示，供 RViz 加载。 | colcon 可发现 |
| [nav2_simple_commander](nav2_simple_commander/README.md) | Python 客户端库：提供 BasicNavigator 及导航、路径跟踪、路点任务的 Python 示例与应用演示。 | colcon 可发现 |
| [nav2_smac_planner](nav2_smac_planner/README.md) | 规划插件：提供二维 A*、Hybrid-A* 和 State Lattice 搜索，支持不同车辆运动约束。 | colcon 可发现 |
| [nav2_smoother](nav2_smoother/README.md) | 平滑服务器与插件：实现 SmoothPath Action，加载简单平滑与 Savitzky–Golay 等路径平滑插件。 | colcon 可发现 |
| [nav2_system_tests](nav2_system_tests/README.md) | 系统测试资源：包含导航系统级测试脚本、场景和模型；当前 COLCON_IGNORE 使其不参加默认 colcon 构建。 | 默认忽略 |
| [nav2_theta_star_planner](nav2_theta_star_planner/README.md) | 规划插件：提供 Theta* 任意角路径搜索，通过栅格视线检查减少路径方向约束。 | colcon 可发现 |
| [nav2_util](nav2_util/README.md) | 公共运行库：提供生命周期节点、Action Server、TF、里程计、几何、速度及 collision 状态等公共工具。 | colcon 可发现 |
| [nav2_velocity_smoother](nav2_velocity_smoother/README.md) | 速度平滑节点：处理控制速度输入，执行开环／闭环平滑及本项目可选 jerk 限制与 collision 目标门控。 | colcon 可发现 |
| [nav2_voxel_grid](nav2_voxel_grid/README.md) | 体素算法库：提供按位压缩的三维体素标记、清除与射线追踪结构，服务于 VoxelLayer。 | colcon 可发现 |
| [nav2_waypoint_follower](nav2_waypoint_follower/README.md) | 路点服务器：把路点列表编排为导航任务，并在路点执行等待、拍照或输入触发等插件。 | colcon 可发现 |
| [nav_2d_msgs](nav2_dwb_controller/nav_2d_msgs/README.md) | 接口包：提供二维位姿、路径、速度等公共消息类型，供 DWB 系列使用。 | colcon 可发现 |
| [nav_2d_utils](nav2_dwb_controller/nav_2d_utils/README.md) | 工具库：提供二维导航消息转换、路径处理和 TF 辅助函数。 | colcon 可发现 |
| [navigation2](navigation2/README.md) | 元包：聚合 Nav2 栈依赖；源码仓库名称 navigation2 与本目录中的同名 ROS 元包应区分。 | colcon 可发现 |
| [spdlog_wrapper](spdlog_wrapper/README.md) | 日志库：封装 spdlog、YAML 配置与 ROS 日志转接，供规控与底盘适配等模块复用。 | colcon 可发现 |
| [teb_local_planner](teb_local_planner/README.md) | 控制插件：通过 Timed Elastic Band 优化轨迹时间、障碍距离与运动学约束，作为 Nav2 控制插件使用。 | colcon 可发现 |
| [teb_msgs](teb_msgs/README.md) | 接口包：定义 TEB 优化轨迹、反馈和障碍物消息，供 teb_local_planner 使用。 | colcon 可发现 |
| [turtlebot3](navigation2_ignition_gazebo_turtlebot3/turtlebot3/README.md) | 仿真资源包：安装 Ignition Gazebo TurtleBot3 的 Launch、地图、模型和参数；ROS 包名为 turtlebot3。 | colcon 可发现 |

## 外部贡献：Fork＋Pull Request

本仓库公开地址：

```text
https://github.com/zpy560/mynavigation2
```

普通外部贡献者不需要本仓库的写权限。推荐使用“Fork 到个人账号、在个人 Fork 开发、向本仓库
`main` 分支提交 Pull Request”的方式贡献代码。

### Fork仓库

贡献者登录 GitHub 后打开：

```text
https://github.com/zpy560/mynavigation2/fork
```

选择自己的个人账号并创建 Fork。完成后，贡献者会得到：

```text
https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2
```

其中 `CONTRIBUTOR_ACCOUNT` 需要替换为贡献者自己的 GitHub 用户名。

### 克隆个人Fork并添加上游仓库

```bash
git clone https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2.git
cd mynavigation2

git remote add upstream https://github.com/zpy560/mynavigation2.git
git remote -v
```

远端职责：

| 远端 | 仓库 | 用途 |
| --- | --- | --- |
| `origin` | 贡献者自己的 Fork | 推送贡献者的开发分支 |
| `upstream` | `zpy560/mynavigation2` | 获取本仓库最新代码 |

### 创建开发分支

不要直接在个人 Fork 的 `main` 分支开发。先创建能够表达改动目的的分支：

```bash
git switch -c fix/map-server-lifecycle
```

其他分支名示例：

```text
feat/keyboard-dev-tty
docs/update-bringup-guide
fix/dual-lidar-tf
```

### 修改、验证并提交

完成修改后，先检查变更范围和验证结果：

```bash
git status
git diff --check
git diff
```

只暂存本次贡献相关的文件：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 修复具体问题并说明影响范围"
```

推荐的提交前缀：

| 前缀 | 用途 |
| --- | --- |
| `feat:` | 新增功能 |
| `fix:` | 修复问题 |
| `docs:` | 更新文档 |
| `refactor:` | 不改变功能的结构调整 |
| `chore:` | 构建、依赖或工程维护 |

### 推送个人分支

```bash
git push -u origin fix/map-server-lifecycle
```

贡献者只向自己的 `origin` 推送，不需要也不应直接向 `zpy560/mynavigation2` 的 `main` 分支推送。

### 创建Pull Request

推送后，在 GitHub 页面点击 `Compare & pull request`，并确认目标关系：

```text
base repository: zpy560/mynavigation2
base branch:     main

head repository: CONTRIBUTOR_ACCOUNT/mynavigation2
compare branch:  fix/map-server-lifecycle
```

PR 描述至少应包含：

- 修改了什么。
- 为什么需要修改。
- 影响哪些包、节点、Topic、Action、Service 或参数。
- 使用了哪些验证命令。
- 哪些运行环境尚未验证。

提交后的 PR 会显示在：

```text
https://github.com/zpy560/mynavigation2/pulls
```

### 根据审查意见更新PR

如果维护者提出修改意见，贡献者继续在同一个开发分支修改并推送即可：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 根据审查意见修正具体问题"
git push
```

新的提交会自动追加到现有 PR，不需要重新创建 PR。

### 同步上游main分支

当 PR 开发期间上游 `main` 有新提交时，可以把上游更新合并到当前开发分支：

```bash
git fetch upstream
git switch fix/map-server-lifecycle
git merge upstream/main
git push
```

如果出现冲突，应在本地解决冲突、重新验证后再推送，不要在不了解影响时覆盖上游文件。

### 权限边界

- 外部贡献者可以 Fork 公开仓库并提交 PR。
- 外部贡献者默认不能直接推送本仓库，也不能自行合并 PR。
- 仓库维护者负责审查、要求修改、批准、关闭或合并 PR。
- 长期可信任的协作者可以单独授予仓库写权限，但普通外部贡献优先使用 Fork＋PR。
- PR 被合并前，变更不会进入本仓库的 `main` 分支。
