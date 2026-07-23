# mynavigation2

本仓库是 `nav2_demo` 项目的 Nav2 源码真源。当前主要运行入口是：

```text
myagv_test_bringup/launch/entry.launch.py
```

该入口面向实车或板端运行：Nav2 不启动 AMCL，不启动自定义 `locationpub` / `laserpub`，定位和雷达数据由外部系统提供。

## 安装依赖

以下命令适用于 Ubuntu 22.04 和 ROS 2 Humble。执行前需已配置 ROS 2 官方 apt 软件源。

安装 Nav2 运行依赖：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

TEB Local Planner 还依赖 g2o：

```bash
sudo apt install ros-humble-libg2o
```

## 编译命令

所有编译命令都必须在 `nav2_ws` 工作空间执行，不要在 `navigation2` 源码仓库内直接编译。

首次编译或需要重建整个工作空间时：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j4"
colcon build --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --parallel-workers 1 --packages-select myagv_test_bringup --cmake-args -DBUILD_TESTING=OFF
限制核编译的数量
```

编译 `myagv_test_bringup` 及工作空间内它依赖的包：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to myagv_test_bringup \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

依赖已经编译完成，只重新编译 `myagv_test_bringup` 时：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select myagv_test_bringup \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

编译完成后，在当前终端加载新的 install 空间：

```bash
source install/setup.bash
```

## 1. 启动命令

推荐从 install 空间启动：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch myagv_test_bringup entry.launch.py
```

板端无图形界面时关闭 RViz：

```bash
ros2 launch myagv_test_bringup entry.launch.py use_rviz:=False
```

默认关键参数：

```text
map:=myagv_test_bringup/maps/out.yaml
params_file:=myagv_test_bringup/params/nav2_params.yaml
use_sim_time:=False
autostart:=true
use_rviz:=True
use_simulator:=False
robot_name:=odom
```

## 2. entry.launch.py 实际启动内容

当前 `entry.launch.py` 有效启动的主要节点和 launch：

```text
map_server
lifecycle_manager_localization
navigation_launch.py
rviz_launch.py
tf2_ros static_transform_publisher: odom -> base_link
```

可选启动：

```text
use_simulator:=True 时启动 gzserver
use_simulator:=True 且 headless:=False 时启动 gzclient
```

当前保留为注释、不启动：

```text
locationpub_cmd
laserpub_cmd
robot_state_publisher_cmd
joint_state_publisher_cmd
robot_description_publisher.py
```

因此目标机运行该 launch 时不依赖：

```text
xacro
joint_state_publisher
robot_state_publisher
```

RViz 中 `RobotModel` 已关闭，主要通过 TF 坐标系、地图、路径、代价地图和激光数据显示导航状态。


## 3. Nav2 数据流

完整运行链路：

```text
1. 地图
   maps/out.yaml
     -> map_server
     -> /map

2. 自研定位
   localization system
     -> TF map -> odom

3. 本体坐标
   entry.launch.py
     -> TF odom -> base_link

4. 激光雷达
   c200 lidar driver
     -> /c200_lidar_node1/scan
     -> global_costmap / local_costmap obstacle layer

5. 导航目标
   RViz Nav2 Goal 或上层系统
     -> /navigate_to_pose action
     -> bt_navigator

6. 全局规划
   planner_server
     -> global_costmap + /map + TF
     -> nav_msgs/Path

7. 路径平滑
   smoother_server
     -> smoothed path

8. 局部控制
   controller_server
     -> local_costmap + path + TF
     -> /cmd_vel_nav

9. 速度平滑
   velocity_smoother
     -> /cmd_vel

10. 底盘执行
    chassis driver
      -> robot motion
```

## 4. Nav2 节点链路

`entry.launch.py` 通过 `navigation_launch.py` 启动导航节点：

```text
controller_server
smoother_server
planner_server
behavior_server
bt_navigator
waypoint_follower
velocity_smoother
lifecycle_manager_navigation
```

职责：

- `bt_navigator`：接收 `NavigateToPose` / `NavigateThroughPoses` action，运行行为树。
- `planner_server`：根据地图、global costmap 和 TF 生成全局路径。
- `smoother_server`：对规划路径做平滑。
- `controller_server`：根据路径、local costmap 和 TF 输出 `/cmd_vel_nav`。
- `velocity_smoother`：把 `/cmd_vel_nav` 平滑为 `/cmd_vel`。
- `behavior_server`：规划或控制失败时执行恢复行为。
- `waypoint_follower`：执行多路点任务。
- `lifecycle_manager_navigation`：管理导航节点生命周期。

## 5. 控制器配置

当前 `myagv_test_bringup/params/nav2_params.yaml` 加载多个控制器：

```text
DWB
RPP
MPPI
GracefulController
RotationShimController
```

默认选择：

```yaml
bt_navigator:
  ros__parameters:
    selected_controller: "RPP"
```

切换控制器时，只改 `selected_controller`，取值必须来自：

```yaml
controller_server:
  ros__parameters:
    controller_plugins:
      - DWB
      - RPP
      - MPPI
      - GracefulController
      - RotationShimController
```

建议：

- `RPP`：当前默认控制器，适合实车低速路径跟踪。
- `DWB`：适合传统采样轨迹和 critic 调试。
- `MPPI`：计算量更大，适合局部轨迹优化实验。
- `RotationShimController`：适合先对齐路径方向再跟踪。
- `GracefulController`：适合验证平滑几何控制。

## 6. 启动后检查

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

检查雷达：

```bash
ros2 topic echo /c200_lidar_node1/scan --once
```

检查地图：

```bash
ros2 topic echo /map --once
```

检查速度输出：

```bash
ros2 topic echo /cmd_vel
```

检查生命周期：

```bash
ros2 lifecycle nodes
```

判断标准：

- `map -> base_link` 能持续查到。
- `/c200_lidar_node1/scan` 有数据。
- 激光 `frame_id` 能接入 TF 树。
- `/map` 正常发布。
- 导航目标发送后 `/cmd_vel` 有输出。
- Nav2 lifecycle 节点进入 active 状态。

## 7. 终端发送导航目标

发送目标前，先确认 `bt_navigator` 已进入 active 状态，且两个 Action Server 可用：

```bash
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses
```

以下坐标取自项目根目录 `test.md` 中的导航测试案例。它们用于说明 Action 命令格式；
在实车地图 `myagv_test_bringup/maps/out.yaml` 上运行前，必须先确认目标点位于可通行区域。

### 7.1 单点 Action 导航

向 `/navigate_to_pose` 发送一个 `PoseStamped` 目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 63.481, y: -12.4484, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```
63.481, -12.4484, 0

### 7.2 多点 Action 导航

向 `/navigate_through_poses` 一次发送一组 `PoseStamped` 目标，机器人按数组顺序导航：

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "{
  poses: [
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 0.570, y: -0.50, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 1.78, y: 0.50, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: -0.60, y: -1.74, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
      }
    }
  ],
  behavior_tree: ''
}" --feedback
```

`behavior_tree: ''` 表示使用 `bt_navigator` 对应导航类型的默认行为树；`--feedback`
用于在终端持续显示剩余距离、导航时间和恢复次数等反馈。

## 8. nav2_regulated_modules 启动说明

`nav2_regulated_modules` 是不使用行为树的自研规控导航入口。它保留 Map Server、Global／Local
Costmap、Planner Server、Smoother Server、Controller Server、Velocity Smoother 和 Lifecycle
管理，但不启动以下节点：

- `bt_navigator`
- `behavior_server`
- `waypoint_follower`
- AMCL

自研 `regulated_navigator` 直接编排：

```text
NavigateToPose／NavigateThroughPoses／goal_pose
  → ComputePathToPose／ComputePathThroughPoses
  → SmoothPath
  → FollowPath
  → /cmd_vel_nav
  → velocity_smoother
  → /cmd_vel
  → controlpub
  → /control_to_uart
```

### 8.1 启动前提

启动前必须确保：

- 已在 `nav2_ws/` 完成所需包的构建。
- 外部定位持续发布动态 `map -> base_link`。
- 激光雷达发布 `/c200_lidar_node1/scan`，消息类型为 `sensor_msgs/msg/LaserScan`。
- 激光消息的 `frame_id` 能接入 `base_link`；当前 `laserpub` 自测节点直接使用 `base_link`。
- 真实底盘控制节点能够接收 `/control_to_uart`。

每个终端先加载环境：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/nav2_logs
export SPDLOG_WRAPPER_LOG_DIR=/tmp/nav2_logs
```

`ROS_LOG_DIR` 和 `SPDLOG_WRAPPER_LOG_DIR` 指向可写目录，避免默认日志目录权限不足导致节点退出。

### 8.2 生产环境启动

生产环境先启动自研定位、雷达驱动和底盘通信，再启动规控导航：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py
```

启动文件默认使用：

- 地图：`nav2_regulated_modules/maps/out.yaml`。
- 参数：`nav2_regulated_modules/params/regulated_modules.yaml`。
- RViz：默认启动。
- 时间源：系统时间，`use_sim_time:=false`。
- 进程模式：非组合模式，`use_composition:=False`。
- Lifecycle：自动激活，`autostart:=true`。

使用外部地图和参数文件：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  map:=/absolute/path/to/map.yaml \
  params_file:=/absolute/path/to/regulated_modules.yaml \
  use_rviz:=True \
  use_sim_time:=false
```

无界面启动：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py use_rviz:=False
```


### 8.3 启动后检查

检查核心 Lifecycle 节点：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
```

正常情况下均应返回：

```text
active [3]
```

检查定位、激光、地图和动作接口：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /c200_lidar_node1/scan --once
ros2 topic echo /map --once
ros2 action list -t
```

动作列表至少应包含：

```text
/compute_path_to_pose
/compute_path_through_poses
/smooth_path
/follow_path
/navigate_to_pose
/navigate_through_poses
```

ROS 图中不应存在 `/bt_navigator`、`/behavior_server` 和 `/waypoint_follower`。

### 8.4 发送自测目标

向固定 TF 当前位姿发送目标，可以验证单点 Action 的完整成功链路：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.569, y: 0.541, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```

这里的 `behavior_tree: ''` 表示不请求行为树。`regulated_navigator` 为兼容标准 Nav2 Action 消息
保留该字段，但会拒绝任何非空行为树 XML。

也可以使用 RViz 的 `2D Goal Pose`，它通过 `/goal_pose` 发送
`geometry_msgs/msg/PoseStamped`。Topic 入口没有外层 Action 结果，但规划、平滑和控制链与单点
Action 相同。

### 8.5 停止顺序

测试完成后，在各终端按 `Ctrl+C`，建议按以下顺序停止：

```text
regulated_modules.launch.py
laserpub 或真实雷达驱动
map2baseTF 或自研定位
底盘通信节点
```

停止规控 Launch 时，Lifecycle 会先停用 `regulated_navigator`，取消下游 Action 并发布零速度。


## 9. 外部贡献：Fork＋Pull Request

本仓库公开地址：

```text
https://github.com/zpy560/mynavigation2
```

普通外部贡献者不需要本仓库的写权限。推荐使用“Fork 到个人账号、在个人 Fork 开发、向本仓库
`main` 分支提交 Pull Request”的方式贡献代码。

### 9.1 Fork仓库

贡献者登录 GitHub 后打开：

```text
https://github.com/zpy560/mynavigation2/fork
```

选择自己的个人账号并创建 Fork。完成后，贡献者会得到：

```text
https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2
```

其中 `CONTRIBUTOR_ACCOUNT` 需要替换为贡献者自己的 GitHub 用户名。

### 9.2 克隆个人Fork并添加上游仓库

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

### 9.3 创建开发分支

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

### 9.4 修改、验证并提交

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

### 9.5 推送个人分支

```bash
git push -u origin fix/map-server-lifecycle
```

贡献者只向自己的 `origin` 推送，不需要也不应直接向 `zpy560/mynavigation2` 的 `main` 分支推送。

### 9.6 创建Pull Request

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

### 9.7 根据审查意见更新PR

如果维护者提出修改意见，贡献者继续在同一个开发分支修改并推送即可：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 根据审查意见修正具体问题"
git push
```

新的提交会自动追加到现有 PR，不需要重新创建 PR。

### 9.8 同步上游main分支

当 PR 开发期间上游 `main` 有新提交时，可以把上游更新合并到当前开发分支：

```bash
git fetch upstream
git switch fix/map-server-lifecycle
git merge upstream/main
git push
```

如果出现冲突，应在本地解决冲突、重新验证后再推送，不要在不了解影响时覆盖上游文件。

### 9.9 权限边界

- 外部贡献者可以 Fork 公开仓库并提交 PR。
- 外部贡献者默认不能直接推送本仓库，也不能自行合并 PR。
- 仓库维护者负责审查、要求修改、批准、关闭或合并 PR。
- 长期可信任的协作者可以单独授予仓库写权限，但普通外部贡献优先使用 Fork＋PR。
- PR 被合并前，变更不会进入本仓库的 `main` 分支。

## 10. myagv_keyboard_control 键盘控制

`myagv_keyboard_control` 通过交互式终端读取方向键或 `WASD`，以固定周期直接发布
`byd_custom_msgs/msg/ControlRes` 到 `/control_to_uart`。该节点绕过 `/cmd_vel` 和
`controlpub`，适合底盘方向、速度符号和串口控制链的独立联调。

### 10.1 运行前提

运行前必须停止 `controlpub` 或其他 `/control_to_uart` 发布者，避免多个节点同时向底盘发送冲突
指令：

```bash
ros2 topic info /control_to_uart --verbose
```

节点需要直接读取交互式终端，必须在能够接收键盘输入的 Shell 中运行。

### 10.2 推荐启动方式

从工作空间加载 ROS 2 和本项目环境：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run myagv_keyboard_control myagv_keyboard_control_node
```

默认以 `50 Hz` 发布控制指令，线速度绝对值为 `0.2 m/s`，角速度绝对值为 `0.5 rad/s`，失键
`0.5 s` 后自动恢复为停车指令。

### 10.3 按键映射

| 按键 | 功能 | `v` | `w` |
| --- | --- | ---: | ---: |
| `W` 或 `↑` | 前进 | `+linear_speed` | `0.0` |
| `S` 或 `↓` | 后退 | `-linear_speed` | `0.0` |
| `A` 或 `←` | 原地左转 | `0.0` | `+angular_speed` |
| `D` 或 `→` | 原地右转 | `0.0` | `-angular_speed` |
| `Space` 或 `X` | 停车 | `0.0` | `0.0` |
| `Q` | 发布停车指令并退出 | `0.0` | `0.0` |

`v_lift` 和 `w_rotation` 始终发布为 `0.0`。

### 10.4 覆盖控制参数

可以在启动时覆盖默认速度、发布频率和失键停车时间：

```bash
ros2 run myagv_keyboard_control myagv_keyboard_control_node --ros-args \
  -p publish_rate:=50.0 \
  -p linear_speed:=0.1 \
  -p angular_speed:=0.3 \
  -p command_timeout:=0.8
```

可用参数：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `output_topic` | `/control_to_uart` | 最终底盘控制 Topic |
| `publish_rate` | `50.0` | 周期发布频率，单位 Hz |
| `linear_speed` | `0.2` | 前进和后退速度绝对值，单位 m/s |
| `angular_speed` | `0.5` | 左右转角速度绝对值，单位 rad/s |
| `command_timeout` | `0.5` | 最后一次方向输入后的停车超时，`0.0` 表示关闭超时 |

### 10.5 检查输出

另开一个已经加载工作空间环境的终端：

```bash
ros2 topic echo /control_to_uart
ros2 topic hz /control_to_uart
ros2 topic info /control_to_uart --verbose
```

默认输出接口：

```text
Topic: /control_to_uart
Type:  byd_custom_msgs/msg/ControlRes
Rate:  50 Hz
```

### 10.6 实车安全要求

首次测试应架空驱动轮或断开动力执行机构，先检查前进、后退和左右转的速度符号，再连接真实底盘。
终端失去焦点、SSH 中断或节点异常退出后，不能只依赖软件自动停车；底盘控制器还应具备独立的通信
超时停车保护。

结束控制时按 `Q`，节点会先发布停车指令再退出。不要直接关闭终端代替正常停车流程。

### 10.7 ros2 launch 当前限制

当前不推荐使用：

```bash
ros2 launch myagv_keyboard_control keyboard_control.launch.py
```

`launch_ros` 启动的子进程没有继承交互式标准输入，`emulate_tty=True` 只处理标准输出和标准错误，
不会把键盘输入连接给节点，因此当前实现会报：

```text
stdin is not a terminal
```

在节点完成 `/dev/tty` 兼容修复前，必须使用 `ros2 run` 从当前交互式 Shell 启动。
