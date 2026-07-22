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

## 3. 外部系统必须提供的数据

### 3.1 自研定位

当前 launch 不启动 AMCL。外部自研定位系统必须提供：

```text
TF: map -> odom
```

当前 launch 内部静态发布：

```text
TF: odom -> base_link
```

Nav2 需要最终能查询：

```text
map -> odom -> base_link
```

如果底盘、里程计或融合定位系统已经发布真实动态 `odom -> base_link`，必须停用 `entry.launch.py` 中的 `static_robot_to_base_link_cmd`，否则同一段 TF 会冲突。

### 3.2 激光雷达

当前 launch 不启动 `laserpub`。真实雷达驱动必须发布：

```text
/c200_lidar_node1/scan
```

消息要求：

```text
类型: sensor_msgs/msg/LaserScan
frame_id: 必须能通过 TF 接到 base_link
```

推荐 TF：

```text
map -> odom -> base_link -> lidar_frame
```

### 3.3 底盘

Nav2 最终输出：

```text
/cmd_vel
```

底盘驱动需要订阅 `/cmd_vel` 并执行速度命令。

## 4. Nav2 数据流

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

## 5. Nav2 节点链路

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

## 6. 控制器配置

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

## 7. 启动后检查

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

## 8. 终端发送导航目标

发送目标前，先确认 `bt_navigator` 已进入 active 状态，且两个 Action Server 可用：

```bash
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses
```

以下坐标取自项目根目录 `test.md` 中的导航测试案例。它们用于说明 Action 命令格式；
在实车地图 `myagv_test_bringup/maps/out.yaml` 上运行前，必须先确认目标点位于可通行区域。

### 8.1 单点 Action 导航

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

### 8.2 多点 Action 导航

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

## 9. nav2_regulated_modules 启动说明

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

### 9.1 启动前提

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

### 9.2 生产环境启动

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

### 9.3 map2baseTF 与 laserpub 自测启动

没有接入真实定位和雷达时，可以用项目内自测节点检查接口链路。

终端 1 启动固定 TF：

```bash
ros2 run map2base_tf map2baseTF
```

该节点默认持续发布：

```text
map -> base_link
x = 0.569 m
y = 0.541 m
yaw = 0 rad
```

终端 2 启动测试激光：

```bash
ros2 run laserpub laserpub
```

测试激光发布：

```text
Topic: /c200_lidar_node1/scan
frame_id: base_link
```

终端 3 启动规控导航：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py use_rviz:=False
```

固定 `map -> base_link` 不模拟车辆运动，只适合验证 TF、Costmap、Action 和状态机链路。发送远离
固定位置的目标后，规控层会因为位姿长期无进展而进入清图重规划，最终可能返回 `ABORTED`；这不
代表真实底盘导航效果。

### 9.4 启动后检查

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

### 9.5 发送自测目标

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

### 9.6 停止顺序

测试完成后，在各终端按 `Ctrl+C`，建议按以下顺序停止：

```text
regulated_modules.launch.py
laserpub 或真实雷达驱动
map2baseTF 或自研定位
底盘通信节点
```

停止规控 Launch 时，Lifecycle 会先停用 `regulated_navigator`，取消下游 Action 并发布零速度。

## 10. 普通用户与 root 会话的 ROS 2 Topic 发现差异测试

本节用于诊断以下现象：设备上的普通用户能够发现后台启动的自定义 ROS 2 Topic，但某个 root
会话中的 Nav2 和 `ros2 topic list` 无法发现这些 Topic；同一套软件通过 `ssh root@<ARM_IP>`
登录 ARM 设备后，又能正常接收和发布相关 Topic。

这种现象发生在 DDS 发现层，暂时不应通过修改 Nav2 的雷达参数、TF、QoS 或 namespace 处理。
同一 ARM 设备、同一 UID、同一软件包在不同 root 会话中表现不同，优先检查会话环境、ROS 2
daemon、DDS 配置、工作空间加载顺序和网络命名空间。

### 10.1 测试原则

分别在以下两个 root 会话中执行完全相同的检查：

- 失败会话：无法发现普通用户后台 Topic 的本地 root、`sudo`、`su` 或后台服务环境。
- 成功会话：通过 `ssh root@<ARM_IP>` 登录后能够正常发现 Topic 的 root 环境。

所有 Topic 列表测试都使用 `--no-daemon`，避免已有 ROS 2 daemon 保留旧的
`ROS_DOMAIN_ID` 或 RMW 配置而干扰结果。

### 10.2 对比用户、工作空间和 ROS／DDS 环境

在失败会话和成功会话中分别执行：

```bash
id

printf 'HOME=%s\nSHELL=%s\n' "$HOME" "$SHELL"

command -v ros2
ros2 pkg prefix myagv_test_bringup

printenv | sort | rg \
'^(ROS_|RMW_|CYCLONEDDS|FASTRTPS|FASTDDS|AMENT_PREFIX_PATH|COLCON_PREFIX_PATH)'
```

重点比较：

```text
ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY
ROS_DISCOVERY_SERVER
RMW_IMPLEMENTATION
CYCLONEDDS_URI
FASTRTPS_DEFAULT_PROFILES_FILE
FASTDDS_DEFAULT_PROFILES_FILE
AMENT_PREFIX_PATH
COLCON_PREFIX_PATH
```

如果 `ROS_DOMAIN_ID` 不同，两种会话位于不同 DDS Domain，彼此不会发现。若
`RMW_IMPLEMENTATION` 或 DDS XML 路径不同，则需要继续检查两个会话是否使用了不同的网卡、
multicast、Discovery Server、静态 Peer 或共享内存配置。

`ros2 pkg prefix myagv_test_bringup` 应指向本次 ARM 编译后的工作空间。若失败会话解析到
`/opt/ros/humble` 或另一个旧工作空间，说明该会话没有加载正确的 `install/setup.bash`。

### 10.3 排除 ROS 2 daemon 残留

在两个 root 会话中分别检查 daemon 的启动参数：

```bash
ps -eo user,pid,args | rg '[_]ros2_daemon'
```

然后停止当前用户的 daemon，并直接创建临时 DDS Participant 查询 ROS 图：

```bash
ros2 daemon stop

ros2 topic list \
  --no-daemon \
  --spin-time 5 \
  -t
```

结果判定：

- 普通 `ros2 topic list` 失败，但带 `--no-daemon` 后正常：daemon 使用了旧 Domain 或旧 RMW。
- 带 `--no-daemon` 后仍然失败：继续检查 DDS 环境或网络命名空间。
- 成功和失败会话中的 daemon 参数不同：以成功 SSH root 会话的 Domain 和 RMW 为核对基准。

### 10.4 检查网络命名空间

在普通用户 Publisher、失败 root 会话和成功 SSH root 会话中分别执行：

```bash
readlink /proc/$$/ns/net
```

查找自定义 Topic Publisher 的 PID：

```bash
ps -eo pid,user,args | rg '自定义话题节点名'
```

再检查 Publisher 所在的网络命名空间：

```bash
readlink /proc/1234/ns/net
```

上例中的 `1234` 需要替换为实际 Publisher PID。

正常情况下，各进程应返回相同的 namespace 编号，例如：

```text
net:[4026531840]
```

如果编号不同，说明进程可能运行在 Docker、Podman、设置了 `PrivateNetwork=true` 的 systemd
服务或其他隔离环境中。即使进程都是 root，DDS multicast、UDP、localhost 和共享内存也可能无法
跨越该隔离边界。

### 10.5 最小跨会话复现实验

普通用户终端启动 ROS 2 官方示例 Publisher：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

失败 root 会话停止 daemon 并查询 Topic：

```bash
source /opt/ros/humble/setup.bash
source /path/to/nav2_ws/install/setup.bash

ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5 -t
```

`/path/to/nav2_ws` 需要替换为 ARM 设备上的实际工作空间路径。

再由失败 root 会话启动一个仅供 root 图发现测试的 Topic：

```bash
ros2 run demo_nodes_cpp talker \
  --ros-args \
  -r chatter:=/root_chatter
```

另开同环境 root 会话再次查询：

```bash
ros2 topic list --no-daemon --spin-time 5 -t
```

如果 root 能看到 `/root_chatter`，但看不到普通用户的 `/chatter`，则可以排除 Nav2 源码，重点检查
跨用户 DDS 环境、Fast DDS 共享内存和网络隔离。如果成功 SSH root 会话同时能看到二者，则继续对比
成功和失败 root 会话的 RMW 与 DDS XML。

### 10.6 检查 Fast DDS 共享内存错误

ROS 2 Humble 通常使用 `rmw_fastrtps_cpp`。Fast DDS 在同一主机上可以使用共享内存，普通用户和
root 混合运行时可能因为共享内存段或端口权限不同而出现通信问题。

只读检查共享内存对象和 ROS 日志：

```bash
ls -la /dev/shm | rg 'fastrtps|fastdds|dds'

rg -n \
'RTPS_TRANSPORT_SHM|open_and_lock_file|Permission denied|Failed init_port' \
/root/.ros/log \
/home/<普通用户名>/.ros/log
```

检查期间不要直接删除 `/dev/shm` 中的 Fast DDS 文件；正在运行的 ROS 2 进程可能仍在使用这些
对象。由于成功 SSH root 会话已经能够与普通用户 Topic 通信，共享内存权限应排在 Domain、daemon、
DDS XML 和网络命名空间之后检查。

### 10.7 结果判定表

| 测试结果 | 原因判断 |
| --- | --- |
| 两个会话的 `ROS_DOMAIN_ID` 不同 | DDS Domain 隔离 |
| RMW 或 DDS XML 路径不同 | DDS 发现和传输配置不一致 |
| 网络 namespace 编号不同 | 容器、systemd 或其他网络隔离 |
| `--no-daemon` 正常，普通查询失败 | ROS 2 daemon 保留了旧环境 |
| `ros2 pkg prefix` 结果不同 | ROS 工作空间或安装副本加载错误 |
| root 只能看到 root 自己发布的 Topic | 跨用户 DDS 或 Fast DDS SHM 问题 |
| SSH root 正常，`sudo ros2 launch` 异常 | `sudo` 环境重置、`HOME` 或 setup 加载差异 |
| 所有环境和 namespace 均一致但结果仍不同 | 检查 Fast DDS 日志、启动顺序和实际进程环境 |

### 10.8 修复方向

定位差异后，优先让所有 ROS 2 节点使用相同普通用户、相同 `ROS_DOMAIN_ID`、相同 RMW 和相同 DDS
配置运行。不要仅为访问串口而把整个 Nav2 栈提升为 root；应给普通用户授予目标设备的最小访问
权限，或只隔离运行确实需要硬件权限的驱动节点。

`controlpub` 只负责把 `/cmd_vel` 转发到 `/control_to_uart`，本身不直接访问串口设备，通常不需要
root 权限。修改设备组或 udev 规则属于系统配置变更，执行前必须先确认具体设备路径、当前属主和
权限范围。

### 10.9 给forlinx日志权限
sudo chown -R forlinx:forlinx /home/byd/logs
chmod -R 755 /home/byd/logs

## 11. 外部贡献：Fork＋Pull Request

本仓库公开地址：

```text
https://github.com/zpy560/mynavigation2
```

普通外部贡献者不需要本仓库的写权限。推荐使用“Fork 到个人账号、在个人 Fork 开发、向本仓库
`main` 分支提交 Pull Request”的方式贡献代码。

### 11.1 Fork仓库

贡献者登录 GitHub 后打开：

```text
https://github.com/zpy560/mynavigation2/fork
```

选择自己的个人账号并创建 Fork。完成后，贡献者会得到：

```text
https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2
```

其中 `CONTRIBUTOR_ACCOUNT` 需要替换为贡献者自己的 GitHub 用户名。

### 11.2 克隆个人Fork并添加上游仓库

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

### 11.3 创建开发分支

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

### 11.4 修改、验证并提交

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

### 11.5 推送个人分支

```bash
git push -u origin fix/map-server-lifecycle
```

贡献者只向自己的 `origin` 推送，不需要也不应直接向 `zpy560/mynavigation2` 的 `main` 分支推送。

### 11.6 创建Pull Request

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

### 11.7 根据审查意见更新PR

如果维护者提出修改意见，贡献者继续在同一个开发分支修改并推送即可：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 根据审查意见修正具体问题"
git push
```

新的提交会自动追加到现有 PR，不需要重新创建 PR。

### 11.8 同步上游main分支

当 PR 开发期间上游 `main` 有新提交时，可以把上游更新合并到当前开发分支：

```bash
git fetch upstream
git switch fix/map-server-lifecycle
git merge upstream/main
git push
```

如果出现冲突，应在本地解决冲突、重新验证后再推送，不要在不了解影响时覆盖上游文件。

### 11.9 权限边界

- 外部贡献者可以 Fork 公开仓库并提交 PR。
- 外部贡献者默认不能直接推送本仓库，也不能自行合并 PR。
- 仓库维护者负责审查、要求修改、批准、关闭或合并 PR。
- 长期可信任的协作者可以单独授予仓库写权限，但普通外部贡献优先使用 Fork＋PR。
- PR 被合并前，变更不会进入本仓库的 `main` 分支。
