# /car_op_status 模拟与按状态切换碰撞框

用两个工具把「行为解析模块发状态」和「边界随状态变」跑通，全部只读现有设计，
不改 `nav2_collision_monitor` 源码。生产 `myworld2.yaml` 的三角色已切为订阅
`car_op_zones/stop|slow_l2|slow_l1` 动态框（见 `production-statebox-wiring.md`），
本工具是那个动态框的发布端。

| 工具 | 角色 |
|---|---|
| [car_op_status_sim.py](car_op_status_sim.py) | 顶替行为解析模块，按接口文档周期发 `/car_op_status` |
| [car_op_zones.py](car_op_zones.py) | 订阅状态，算出三级边界，发 `PolygonStamped` |

边界数值一个都不在这两个脚本里：全部 `import` [doc/collision_zones/gen_zones.py](../../doc/collision_zones/gen_zones.py)，
取它的 `STATES[*]["layers"]`。设计改了，工具跟着改。

## 接口契约（按文档）

| 项 | 值 |
|---|---|
| 话题 | `/car_op_status` |
| 类型 | `std_msgs/msg/Int32` |
| 发布者 | 行为解析模块（本工具在演示时代替它） |
| 订阅者 | 调度模块 |
| 周期 | 100 ms |

`car_op_status_sim.py --period` 默认 `0.1`，即文档的 100 ms。

## 状态映射

`CarGmstatus` 枚举即是 9 个设计状态，一一对应、差 1：枚举值 0–8 ↔ 设计状态号 1–9。
空载/载货、正走/倒走已内建在每个设计状态里，不再需要额外补轴。

| 枚举值 | 设计状态 | key | 形状/轴 |
|---:|---|---|---|
| 0 | 1 空载正走 | idle_forward | rect / v |
| 1 | 2 空载倒走 | idle_backward | rect / v |
| 2 | 3 正走抓料框 | grab_approach | rect / 固定 |
| 3 | 4 搬货倒走 | cargo_backward | rect / v |
| 4 | 5 搬货正走 | cargo_forward | rect / v |
| 5 | 6 静止 | stationary | circle / 固定 |
| 6 | 7 原地旋转 | spin_in_place | circle / w |
| 7 | 8 去充电 | dock_charge | rect / v（屏蔽前方） |
| 8 | 9 充电结束离开 | undock_leave | rect / v（屏蔽后方） |

枚举值 `v` 的 Int32 载荷即线上值；消费端 `状态号 = 枚举值 + 1`。
`/motion_state` 里的 `v_car`／`w_car` 仍用于在该状态的档位里选档（按 `|v|` 或 `|w|`），
但不再用来推「方向」「载货」——那两个轴已经由状态本身决定。

## 档位

带 `v`／`w` 轴的状态按 `|v_car|`（或 `|w_car|`）落入速度档（L3/L2/L1 三层净空随之变）；
轴为「固定」的状态（抓料框、静止）单档。载货与否由状态决定：只有 状态4/5（搬货走）
的边界自货框外缘量起，其余均自车体。

实车要**自动换状态/换档**，必须同时有两个输入：

- `/car_op_status`（Int32，100 ms）由行为解析模块发布；没有外部发布者时，
  launch 里的 `car_op_status_sim` 固定发 0（状态1 空载正走）。
- `/motion_state`（`byd_custom_msgs/MotionState`）由运控/底盘发布，`v_car`/`w_car`
  决定档位；`car_op_zones` 以 BEST_EFFORT（SensorDataQoS）订阅，兼容运控的
  BEST_EFFORT 与仿真的 RELIABLE 发布端。没有它时 `v=w=0`，永远第 0 档。

两个输入都缺时，框会一直停在「状态1 / 第0档」，启动日志会提示未收到
`/motion_state`。验证：`ros2 topic info /car_op_status -v`、
`ros2 topic echo /motion_state --once`。

## 跑起来

两个工作空间不要叠加加载；本工具在 `nav2_ws` 里。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

两个终端：

```bash
# 终端 1：消费端（含自带配色的 MarkerArray，供 RViz 看）
# 仿真车体用 sim；实车用 real（缺省即 real）
ros2 run nav2_collision_monitor car_op_zones.py --profile sim

# 终端 2：状态源，每个枚举值停 3 秒循环
ros2 run nav2_collision_monitor car_op_status_sim.py --mode cycle --hold 3.0 --mock-motion
```

`--mock-motion` 会一并模拟 `/motion_state`，所以脱离整车也能跑。
要接真实底盘就去掉它，改订阅真实的 `/motion_state`。

### 尺寸档：仿真 / 实车

`car_op_zones.py --profile sim|real`，默认 `real`（漏传时按更保守的实车尺寸）：

| 档位 | 车体外廓 | 空载 tier0 L3 前/后/侧 | 静止 L3 半径 |
|---|---|---|---|
| `sim` | 0.52 × 0.36 m（diffbot） | 0.56 / 0.41 / 0.28 | 0.35 |
| `real` | 1.2 × 0.8 m（footprint） | 0.90 / 0.60 / 0.50 | 0.75 |

两档的**净空值相同**，边界 = 车体外缘 + 原设计净空，所以实车框整体更大。
净空表分别为 `spec_table.md`（sim）与 `spec_table_real.md`（real）。
launch 已固定：`myworld_bringup/entry.launch.py` 传 `--profile sim`，
`regulated_modules.launch.py` 传 `--profile real`。

只想看一个状态（`--value` 取枚举值 0–8）：
- `--value 0` = 状态1 空载正走
- `--value 6` = 状态7 原地旋转
- `--value 7` = 状态8 去充电（屏蔽前方）

```bash
ros2 run nav2_collision_monitor car_op_status_sim.py --mode hold --value 6 --mock-motion --mock-w 1.2
```

跑固定序列（CSV：`枚举值,保持秒数`，取值 0–8）：

```bash
printf '0,2\n3,2\n6,3\n7,3\n' > /tmp/sched.csv
ros2 run nav2_collision_monitor car_op_status_sim.py --mode script --file /tmp/sched.csv --mock-motion
```

## 看得见 vs 真的会停

这是两条独立路径，别混。

**看得见**（默认，`car_op_zones.py` 不带 `--drive-monitor`）：
发到 `collision_stop_zone` / `collision_slowdown_zone` / `collision_slowdown_l1`，
恰好是 `collision_boundary_visualizer` 默认订阅的三个话题，RViz 里能看见框变。
另外还发 `/car_op_zones/markers` 的 `MarkerArray`，按设计文档锁定的配色
（L1 绿 `#43a047` / L2 橙 `#fb8c00` / L3 红 `#e53935`）画线框和净空标签。

> 自带 markers 的原因：`collision_boundary_visualizer` 按话题名猜颜色，
> `collision_slowdown_zone` 与 `collision_slowdown_l1` 都含 `slow`，会被判成同色同标签，
> L1/L2 分不出来。要看设计图的配色，用 `/car_op_zones/markers`。

在 RViz 里加一个 `MarkerArray` 显示项订阅 `/car_op_zones/markers` 即可。

**真的会停**（`--drive-monitor` + demo YAML）：
`nav2_collision_monitor/params/car_op_zones_demo.yaml` 单独给一个 `collision_monitor`，
`PolygonStopL3` 用 `type: "polygon"` + `polygon_sub_topic` 订阅 `car_op_zones/stop`，
所以在 RViz 看到框的同时，碰撞判定用的就是同一个框。

```bash
ros2 run nav2_collision_monitor collision_monitor --ros-args \
  --params-file $(ros2 pkg prefix nav2_collision_monitor)/share/nav2_collision_monitor/params/car_op_zones_demo.yaml \
  -r __node:=collision_monitor_demo
```

它是**生命周期节点**，起来后还要手动 configure + activate，否则不建订阅：

```bash
ros2 lifecycle set /collision_monitor_demo configure
ros2 lifecycle set /collision_monitor_demo activate
```

它还有个坑：只有 `cmd_vel` 有流入时才会跑周期并重发框。空转的话看不到任何输出，
需要给点速度：

```bash
ros2 topic pub -r 20 /cmd_vel_collision_in geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

安全性：demo YAML 的 `cmd_vel_out_topic` 是 `cmd_vel_car_op_demo`，
**不接 `cmd_vel`**，避免演示时误驱动实车。要真接入速度链须自己改这一行，
并先确认没有第二个节点也在发 `cmd_vel`。

## 翻译成 regulated／运控枚举：obstacle_status_translator.py

上面两条路径（看得见 / demo 里真会停）都还是让 `collision_monitor` 自己截速度。
正式方向相反：**避障只发状态，减速／停车由真实运控模块执行**。翻译器就是这段接口：

| 项 | 主接口（regulated） | 兼容接口（原运控） |
|---|---|---|
| 话题 | `/collision_state` | `/obstacle_status_int` |
| 类型 | `std_msgs/msg/UInt8` | `std_msgs/msg/Int32` |
| 发布者 | 避障（本工具） | 避障（本工具） |
| 订阅者 | `regulated_navigator` / `controller_server` / `velocity_smoother` | 运控 |
| 周期 | 默认 25 Hz；SLOWDOWN 40 Hz；STOP 80 Hz | 同左 |
| QoS | RELIABLE，depth 10 | 同左 |

主接口与 `nav2_util::CollisionState` 对齐：regulated 的订阅端是
BEST_EFFORT + VOLATILE depth 1，RELIABLE 发布端对它兼容；`collision_state_timeout`
为 0.5 s，超时后 regulated 按 0（停车）处理。兼容接口用 `--legacy-status-topic ''`
可关闭。

| 枚举 | 含义 | regulated 行为 | 触发条件 |
|---:|---|---|---|
| 0 | ERROR | 停车 | 雷达断流超过 `--scan-timeout`，或动态框缺失／超过 `--zone-timeout` |
| 1 | STOP | 停车 | 落点在 `car_op_zones/stop` 内 |
| 2 | SLOWDOWN | 限速到任务速度 35% | 落点在 `car_op_zones/slow_l2` 内 |
| 3 | WARNING | 限速到任务速度 65% | 落点在 `car_op_zones/slow_l1` 内 |
| 4 | CLEAR | 不限制 | 三级框内都没有落点 |

`enable_collision_state:=true`（entry 默认，fixed_path 生效）时，regulated 会真正
按枚举减速／停车——即仿真里现在能直接看到避障动作。只想验证协议、不想改变
仿真行为时，传 `enable_collision_state:=false`。

优先级 L3 > L2 > L1；每级至少 `--min-points`（默认 3）个落点才算命中。
枚举变化时立即补发一帧，其余按当前档位的固定周期发布（`timer_period_ns` 随枚举切换，
不靠 10 ms 轮询判 due——那样实测只有约 22 Hz）。

翻译器**不依赖** `collision_monitor`：CM 的状态是边沿触发的，启动时给不出 CLEAR，
也没法按 25/40/80 Hz 主动上报。它自己订阅 `/scan` 和三个动态框，用 TF 把每条
LaserScan 的落点按**该帧自己的时间戳**投到 `base_link`，再做点在多边形内判断。

实车 2D 雷达会扫到自己的车体、支架和另一颗雷达；这些点按定义就落在 L3 停止框内
（L3 = 车体外缘 + 净空），单帧噪点即可让枚举在 4/CLEAR 与 1/STOP 之间反复跳。
翻译器默认按 `--profile`（sim 0.52×0.36 m / real 1.2×0.8 m）把投影后位于车体
矩形（外扩 `--body-margin`，默认 0.05 m）内的点丢掉，再做三级判定；可传
`--no-body-filter` 关闭。注意 `collision_monitor` 的 `scan_blind_zone` 剔除区
只作用于速度门，翻译器不订阅它，开不开都不影响枚举。

翻译器保留可选的透传能力：订阅 `/cmd_vel_collision_in`（`velocity_smoother` 输出），
收到就原样发到 `/cmd_vel`，不改数值；超过 `--cmd-timeout`（默认 0.5 s）补一帧零速。
但仿真和实车的 launch **都传 `--no-passthrough`**：翻译器只发枚举，不发布 `/cmd_vel`，
减速／停车一律由下游按枚举执行。

### 实车：regulated_modules.launch.py 或 obstacle_status.launch.py

实车有两个等价入口，**二选一，不要同时启动**（否则同名节点和双发布者）：

```bash
# 方式 A：和规控一起起（实车当前入口）
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=fixed_path use_sim_time:=false \
  use_collision_monitor:=true use_collision_velocity_gate:=false

# 方式 B：只起碰撞检测模块
ros2 launch nav2_collision_monitor obstacle_status.launch.py \
  use_sim_time:=false scan_topic:=/c200_lidar_node/scan \
  use_collision_monitor:=true
```

- `use_collision_monitor`（默认 false）是**碰撞检测模块总开关**，两个入口语义一致。
  传 true 才启动 `car_op_zones` / 默认状态源 / `obstacle_status_translator`。
- `use_obstacle_status`（默认 true）只控制枚举输出：开启时启动 translator；传 false 只保留
  动态框和状态源，不发布枚举话题。
- `use_collision_velocity_gate`（默认 false）是独立的 Nav2 `collision_monitor` **速度门**开关。
  状态模块不改速度；需要 CM 截速度时才开它。两个都开会让 CM 和 regulated 的
  `collision_state` 门控同时作用，默认只选一个。
- 两个都开时，CM 的 `velocity_polygon` 和 `car_op_zones` 都会发布
  `collision_stop_zone` / `collision_slowdown_zone` / `collision_slowdown_l1`，
  可视化会出现双发布者；确实要同时开就关掉 `use_collision_visualization`，
  或给 CM 改 `polygon_pub_topic`。
- 模块**只发状态、不改速度**：翻译器固定带 `--no-passthrough`，不发布 `/cmd_vel`，
  减速/停车由运控按 `/obstacle_status_int` 执行。
- 默认状态源同样带 `--yield-to-other-publishers`；现场有自己的 `/car_op_status` 时会自动让位。
  没有状态源时传 `use_car_op_status_sim:=false`。
- 前置条件：`base_link→<scan frame>` 静态 TF 必须存在，否则翻译器一直发 0（ERROR）。

速度门和状态模块的输入相近（雷达 + 碰撞框），输出不同：前者直接改速度；
后者输出固定频率的枚举。速度门的状态边沿触发，不能当枚举/心跳用。

### 随 entry.launch 一起起

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path use_collision_monitor:=true
```

`use_collision_monitor` 是碰撞检测模块的总开关（entry 里默认 false）：传 true 时启动
`car_op_zones.py --drive-monitor`、默认状态源和 `obstacle_status_translator.py`。
`use_obstacle_status` 默认 true，只控制 translator 是否启动；传 false 时保留动态框和
状态源，但不发布枚举。translator 固定带 `--no-passthrough`，模块**不改速度**；
原来 Nav2 `collision_monitor` 的速度门拆成独立开关 `use_collision_velocity_gate`
（默认 false），确有需要时才开。

`car_op_zones` 必须有状态源才会发框。launch 默认起一个持有
`car_op_status_value`（默认 0，空载正走）的 `car_op_status_sim`，所以启动后就有
状态 0 的边界框。这个默认源带 `--yield-to-other-publishers`：

- 窗口 4 再起状态源时，它检测到 `/car_op_status` 有第二个发布者，自动暂停；
- 窗口 4 退出后，它恢复发布状态 0。

所以“默认框 → 命令切换成别的状态框”仍是同一个状态话题在换值，不会两套框打架：

```bash
ros2 run nav2_collision_monitor car_op_status_sim.py --mode cycle --hold 6
```

如果外部状态源是别的进程/节点，效果相同——默认源只看话题上是否出现第二个发布者。
老版本工具没有这个让位逻辑时，需要给窗口 1 传 `use_car_op_status_sim:=false`，
否则两个发布者会让动态框持续跳闪。单窗口固定状态仍可用
`car_op_status_value:=0..8`。

### TF 前置条件

`/scan` 和 `/odometry` 用墙钟，Gazebo 的 `odom→base_footprint` 原来用仿真钟，
直接按 scan 时间戳查 TF 会外推失败。`entry.launch.py` 的墙钟模式因此改为：

```text
gazebo_tf_bridge_raw → /tf_raw → tf_wallclock_reframe.py → /tf
```

`tf_wallclock_reframe.py` 把所有变换的时间戳改写成墙钟，并把子帧
`base_footprint` 改名为 `base_link`（entry 的 URDF 已经把 `base_joint` 翻成
`base_link→base_footprint`，不改名会形成同名父子环）。`base_link→base_scan`
与 `map→odom` 都是 `/tf_static`，静态变换对任意时间戳有效，翻译器只订阅
`/tf_static`，不用承受约 50 Hz 的 `/tf` 回调。

### 单独跑

```bash
ros2 run nav2_collision_monitor obstacle_status_translator.py \
  --scan-topic /scan \
  --stop-topic /car_op_zones/stop \
  --slow-l2-topic /car_op_zones/slow_l2 \
  --slow-l1-topic /car_op_zones/slow_l1 \
  --zone-frame base_link \
  --status-topic /collision_state --status-type uint8 \
  --legacy-status-topic /obstacle_status_int \
  --cmd-in-topic /cmd_vel_collision_in --cmd-out-topic /cmd_vel
```

`--status-topic`／`--status-type` 默认就是 `/collision_state` + `uint8`，上面的
写法只是显式列出。只发原运控接口时传
`--status-topic /obstacle_status_int --status-type int32 --legacy-status-topic ''`。

注意不要和 `collision_action_test.py` 同时起：它也发布 `/collision_state`，
该工具会检查该话题必须恰好 1 个发布者。

`ros2 run ... --ros-args -p use_sim_time:=true` 也可用：脚本用 `parse_known_args`
接收 launch 追加的 `--ros-args`，不会被 argparse 拒掉。

### 已实测

在 `myworld2` 里用 Gazebo 生成/删除方块，逐级退开：

| 场景 | 枚举 | 实测频率 |
|---|---:|---:|
| 方框在停止框内 | 1 | 80.008 Hz |
| 退到减速框 | 2 | 40.001 Hz |
| 退到预警框 | 3 | 24.995 Hz |
| 退到 2.3 m 外 | 4 | 25.006 Hz |
| 杀掉 `car_op_zones` | 0 | —（框超时） |
| 重启 `car_op_zones` | 4 | 24.999 Hz |

`/cmd_vel_collision_in` 给 0.1 m/s 时 `/cmd_vel` 原样出现 0.1，停发后 0.5 s 补零
（这是手动运行、未加 `--no-passthrough` 时的透传实测）。launch 里固定
`--no-passthrough`，翻译器不发布 `/cmd_vel`；速度门默认也不启动，`/cmd_vel`
仍由 `velocity_smoother` 单独发布。

### 用 Cartographer 做定位（窗口 2）

entry 默认认为外部定位（Cartographer）负责 `map→base_link`：不启动 Gazebo TF
桥／reframe，也不发静态 `map_odom_anchor`，只保留 `base_link→base_scan`、
`base_link→base_footprint` 等固定变换。本车 Cartographer 直接发
`map→base_link`（没有独立的 `odom` 帧），导航参数里的 `odom_frame_id`
已被适配成 `map`。

正常启动不需要额外参数：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path use_collision_monitor:=true
```

只有完全不用外部定位、想让 entry 自己用 Gazebo 里程计凑出定位时才传
`use_external_localization:=false`。

窗口 2 不要用整份 `cartographer.launch.py`：它自带的 `robot_state_publisher`
发 `base_footprint→base_link`，和 entry 的 `base_link→base_footprint` 反向成环；
两个 `static_transform_publisher` 也和 entry 的 `base_scan` 重复。只起节点即可：

```bash
source /home/forlinx/lidar_slam/colcon_ws/install/setup.bash
ros2 run cartographer_ros cartographer_node --ros-args \
  --params-file /home/forlinx/lidar_slam/colcon_ws/install/cartographer_ros/share/cartographer_ros/configuration_files/cartographer.yaml \
  -r scan:=/scan -r /odometry:=/odometry
```

它自带 `/initialpose` 订阅，窗口 1 RViz 的 **2D Pose Estimate** 可以直接用。
实测：`map→base_link` 由 Cartographer 发布且时间戳为墙钟，`/tf_static` 里
只有 entry 的 `base_link→base_scan` 等固定边，翻译器仍保持 CLEAR 25 Hz，
`/initialpose` 会触发 `HandleInitialPose SetInitialPose ok!`。
窗口 3 的 `NavigationService` 目标可被接受，小车起步并前进 0.101 m（进度 1.37%），
取消后速度归零、状态回到 CLEAR。

## 为什么 demo YAML 必须单独一份

`velocity_polygon` 现在是主配置的默认模式（`myworld2.yaml`、`regulated_modules.yaml`），
原 `*_vp.yaml` 对比配置已删除。
`VelocityPolygon::getParameters` 固定传 `use_dynamic_sub_topic=false`
（`velocity_polygon.cpp:47`），所以 **velocity_polygon 无法订阅 `polygon_sub_topic`**，
只能按速度档自己切。要按外部状态整帧换框，只能用 `type: "polygon"`（示例见
`params/car_op_zones_demo.yaml`）。

另外 `Polygon::dynamicParametersCallback` 只接受 `enabled`（bool），
运行中改不了顶点 —— 所以换框只能靠重发整帧，不能靠改参数。

## 自检

```bash
ros2 run nav2_collision_monitor car_op_zones.py --self-check --profile sim
ros2 run nav2_collision_monitor car_op_zones.py --self-check --profile real
```

不启动 ROS，只做两件事：

1. 逐点核对 9 状态 × 各档 × 3 层的顶点与 `gen_zones` 的 `layers` 一致（87 组）
2. 逐格核对 `gen_zones` 的净空与对应档位的 `spec_table*.md` 一致（87 格）

任一处漂移都会列出行号并返回非零。改完设计后先跑这个再跑仿真。
