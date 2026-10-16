# nav2_regulated_modules

## 职责与结构

推荐的规控统一入口，提供无 BT 任务协调、规划／平滑／控制客户端、专用固定路径控制器、目标检查器、速度诊断与碰撞区域可视化。仿真世界和传感器桥接由场景包负责，外部定位、激光、轮速和底盘通信需另行提供。

| 目录／声明 | 内容 |
|---|---|
| [launch/](launch/) | `regulated_modules.launch.py` 与 RViz 入口 |
| [params/regulated_modules.yaml](params/regulated_modules.yaml) | 节点参数、规划／控制插件和反馈话题 |
| [src/](src/)／[include/](include/) | 协调器、规划与控制模块、固定路径跟踪与停止逻辑 |
| [fixed_path_controller_plugin.xml](fixed_path_controller_plugin.xml) | `nav2_regulated_modules::FixedPathController` 与 `FixedPathGoalChecker` |
| [tools/README.md](tools/README.md)／[test/](test/) | 现有环境观察、隔离实验及开发验证 |
| [doc/](doc/) | 架构、日志与分开验收方法 |

依赖标准 Nav2 服务器、插件、`byd_custom_msgs`、`controlpub`、`myagv_keyboard_control` 及 ROS 2 通信库，完整依赖见 [package.xml](package.xml)，安装可执行目标见 [CMakeLists.txt](CMakeLists.txt)。

## 构建与启动

从项目根目录执行；两个工作空间不要叠加加载：

```bash
cd nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to nav2_regulated_modules \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous
```

`BUILD_TESTING=OFF` 适用于运行构建，不执行测试。模式启动后不能热切换；停止旧入口后再切换。

| 模式 | 输入 | 行为 |
|---|---|---|
| `remote` | 键盘终端 | 使用键盘包直接生成底盘 ControlRes；线／角速度默认限幅 0.3 m/s、0.3 rad/s |
| `autonomous`（默认） | `/navigate_to_pose`、`/navigate_through_poses` 与目标话题 | regulated_navigator 调用 Planner／Smoother／Controller，无行为树 |
| `fixed_path` | `/navigation_service` | 由 NaviSegment[] 生成路径，使用专用控制器与目标检查器跟踪 |

直接入口不启动 Gazebo、AMCL、合成 TF 或合成雷达。默认地图为 `/userdata/map/pbstream/out.yaml`；用 `map:=...` 覆盖。可用 `ros2 launch nav2_regulated_modules regulated_modules.launch.py --show-args` 查看安装版本支持的参数，实际值受 YAML、Launch 重写及显式实参共同影响。

## 数据流与反馈

```text
autonomous: 导航目标 → regulated_navigator → Planner → 可选 Smoother → FollowPath
fixed_path: NavigationService → 方向化 Path → FollowPath
FollowPath → Controller Server → /cmd_vel_nav
  → Velocity Smoother → /cmd_vel → controlpub → /control_to_uart
启用 Collision Monitor 时：平滑器 → /cmd_vel_collision_in → Collision Monitor → /cmd_vel
```

控制器、导航器默认读取 `/odometry` 的 Odometry；平滑器及底盘闭环使用 `/motion_state` 的 MotionState。`v_car/w_car` 映射到线／角速度；反馈过期、定位缺失、取消和正常到点分别处理。TF 须提供新鲜 `map→base_link`，或由明确的 TF 链组合得到；不把原始里程计位置当作地图定位。

`regulated_navigator` 另监听 `/downstream/chassis_control`，收到该支路有效命令才处理底盘请求。部署时须核对最终控制源，避免键盘直控与导航转发同时竞争 `/control_to_uart`。

## 固定路径与终点行为

- [NavigationService](../byd_custom_msgs/action/NavigationService.action) 接收 `task_id` 和 [NaviSegment[]](../byd_custom_msgs/msg/NaviSegment.msg)。支持直线与三次贝塞尔；相邻段连续，同一 Action 所有段同方向。
- `max_speed` 为有限正数，所有段取最小值，再受 `fixed_path_max_speed=1.5` 限制。方向 1／2 分别生成前进／倒车车体姿态和正／负速度，不修改初始定位或 Gazebo 位姿。
- 起点总距离上限 1.2 m；首次横向误差决定直接跟踪或严格对齐策略。横向误差较小时仍需满足航向条件才进入直线跟踪，大航向误差可能先原地转向；收到 Action 不代表立即达速。
- 每条路径起步保护距离默认 0.5 m，速度上限 0.3 m/s；横向和航向误差满足条件并稳定后解除，当前 YAML 的释放航向门限为 20°、稳定周期 10。
- 当前项目 YAML 的停车入口 0.008 m 与目标检查器位置容差 0.01 m 分开；位置／终点平面及停稳参与终点判断，终点航向作诊断。内部成功不等于独立地图精度或实车精度通过。

启动与 Action 几何例子见[仿真入口说明](../myworld_bringup/README.md)；控制实现见 [fixed_path_controller.cpp](src/fixed_path_controller.cpp) 与 [fixed_path_goal_checker.cpp](src/fixed_path_goal_checker.cpp)。

## collision 状态与 Collision Monitor

两项功能独立：外部 `/collision_state` 门控接收 `std_msgs/msg/UInt8`；Collision Monitor 依据传感器区域过滤速度，本包不从激光自动生成外部 collision 状态。

| 项目 | 直接规控入口 | `myworld_bringup/entry.launch.py` |
|---|---|---|
| `enable_collision_state` | 默认 true，仅 fixed_path 生效 | 默认 true，仅 fixed_path 生效 |
| `use_collision_monitor` | 默认 false | 默认 false |
| `use_collision_visualization` | 默认 true，导航模式 | 默认 true，导航模式 |

`enable_collision_state` 由 Launch 转成三节点的 `collision_state_enabled`。项目 YAML 为 true；通用 C++ 节点独立运行的内置回退仍为 false，不能混写这两层默认值。自主和遥控模式不启用外部状态门控。

| 有效状态 | 含义 | 相对任务速度上限 |
|---|---|---:|
| 4 | 正常 | 100% |
| 3 | 一级减速 | 65% |
| 2 | 二级减速 | 35% |
| 1、0 | 停止 | 0 |

启动、configure 或 reset 后从未收到消息时为 4，保留 `received=false`、`age=-1`、`reason=initial`，不因没有首条消息而超时。首次收到后按实际值处理，非法值或接收间隔达到 0.5 s 按 0 平滑停车；超时使用单调时钟。订阅为 BEST_EFFORT／VOLATILE／KeepLast(1)。

状态 4 只解除限制，不创建任务或速度。停止期间暂停对应无进展计时；恢复时仅在原任务及速度输入仍有效时继续，不重建路径或恢复已取消任务。门控在 S 曲线目标生成前，要求 `jerk_limited_smoothing=true`；参数修改需要重启或生命周期重新配置。

已有有效任务时，可在同环境终端持续发布状态；下面命令可能使车辆按原任务运动：

```bash
ros2 topic pub --rate 50 /collision_state std_msgs/msg/UInt8 '{data: 4}'
```

周期例子见 [COLLISION_TEST.md](tools/COLLISION_TEST.md)：50 Hz 发布 `4→3→2→1→0`，默认保持 4／3／3／3／3 秒；请求 1.5 m/s，支持去程前进、返程倒车。有限运行示例（先核对地图端点与起步位姿）：

```bash
ros2 run nav2_regulated_modules collision_action_test.py \
  --schedule periodic --return-direction backward --max-legs 4 \
  --output-dir "$PWD/collision-runs/$(date +%Y%m%d-%H%M%S)"
```

该周期工具退出时会发布停止并取消自身任务，不能用它的 Ctrl+C 代替“独立状态源断流、原 Action 保持”的验收。配置 50 Hz 与实测发布调用频率、独立订阅接收频率应分别报告。

## 日志、工具与证据

统一速度诊断默认开启：文件快照 100 Hz、终端摘要 1 Hz；默认日志目录 `/tmp/nav2_logs`，可通过 `log_dir` 改到数据盘。`ROS_LOG_DIR` 管理 ROS Launch 日志，两者独立。`enable_legacy_velocity_logs=false` 默认抑制重复摘要，关闭统一诊断后回退旧记录。

- [速度日志](doc/velocity_logging.md)：字段、异步滚动、S 曲线及缺失／过期状态。
- [固定路径架构](doc/fixed_path_architecture.md)：既有模块与控制流程分析，其中旧的 Collision Monitor 默认开启描述以本 README 的当前默认表为准。
- [分开验收](doc/action_only_acceptance.md)：速度停稳与定位采样质量分开评价。
- [测试工具](tools/README.md)：工具运行前提及会修改环境的实验边界。

本文为当前源码静态说明。本轮文档更新不执行构建或运动。已有隔离真实节点＋合成反馈回归不等同于实车验收；现有短路线周期测试只取得部分减速／停止证据，独立状态源启停验收因起步转向占满 5 秒基准窗而 UNAVAILABLE，不能声称两程全部通过。动态验证记录由外层项目 ROADMAP 与报告维护。
