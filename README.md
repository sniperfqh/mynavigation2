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
sudo apt install ros-humble-nav2
```

TEB Local Planner 还依赖 g2o：

```bash
sudo apt install ros-humble-libg2o
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

## 8. 常见问题

### 8.1 找不到 `xacro`

当前 `entry.launch.py` 不运行时导入 `xacro`。如果仍报：

```text
ModuleNotFoundError: No module named 'xacro'
```

优先确认目标机运行的是最新 install 空间，并且 `myagv_test_bringup` 已重新构建、重新 source。

### 8.2 找不到 `joint_state_publisher`

当前 `entry.launch.py` 不启动 `joint_state_publisher`。如果仍报：

```text
package 'joint_state_publisher' not found
```

优先确认目标机没有使用旧 install 包。

### 8.3 普通用户不能运行，su 后可以运行

该现象通常不是 launch 代码问题，而是普通用户和 root 环境不同。重点检查：

```bash
whoami
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH
echo $LD_LIBRARY_PATH
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
echo $ROS_LOCALHOST_ONLY
which ros2
ros2 pkg prefix myagv_test_bringup
groups
ls -ld ~/.ros ~/.ros/log
```

推荐修复：

```bash
sudo chown -R $USER:$USER /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
sudo chown -R $USER:$USER ~/.ros
sudo usermod -aG dialout,plugdev,video $USER
```

执行 `usermod` 后需要重新登录。

### 8.4 TF 冲突

如果外部系统已经发布：

```text
odom -> base_link
```

则不要同时保留 launch 内的静态：

```text
static_robot_to_base_link_cmd
```

否则 TF 会出现重复发布，Nav2 位姿可能跳变或 costmap 异常。

## 9. 同步检查

修改 `myagv_test_bringup` 后，必须确认源码和工作区副本一致：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo
diff -qr -x __pycache__ -x '*.pyc' -x .DS_Store \
  navigation2/myagv_test_bringup \
  nav2_ws/src/navigation2/myagv_test_bringup
```

修改仓库根 README 后，也要同步：

```bash
diff -u navigation2/README.md nav2_ws/src/navigation2/README.md
```
