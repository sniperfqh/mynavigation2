# Nav2 Simple (Python3) Commander

中文注解：该包提供 Nav2 的 Python 简化调用接口，让应用代码可以用同步风格 API 调用导航、规划、恢复行为和代价地图服务。

## Overview

The goal of this package is to provide a "navigation as a library" capability to Python3 users. We provide an API that handles all the ROS2-y and Action Server-y things for you such that you can focus on building an application leveraging the capabilities of Nav2. We also provide you with demos and examples of API usage to build common basic capabilities in autonomous mobile robotics.

中文注解：本节说明包的定位：把 ROS 2 action、service、lifecycle 等细节封装起来，便于用 Python 编写移动机器人业务逻辑。

This was built by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) at [Samsung Research](https://www.sra.samsung.com/), with initial prototypes being prepared for the Keynote at the [2021 ROS Developers Day](https://www.theconstructsim.com/ros-developers-day-2021/) conference (code can be found [here](https://github.com/SteveMacenski/nav2_rosdevday_2021)).

![](media/readme.gif)

## API

See its [API Guide Page](https://navigation.ros.org/commander_api/index.html) for additional parameter descriptions.

中文注解：完整参数解释以官方 API 文档为准；下面表格列出本包最常用的 BasicNavigator 方法。

The methods provided by the basic navigator are shown below, with inputs and expected returns. If a server fails, it may throw an exception or return a `None` object, so please be sure to properly wrap your navigation calls in try/catch and check results for `None` type.

中文注解：调用规划、平滑、导航等接口时，要处理 action/server 失败返回 `None` 或异常的情况。

New as of September 2023: the simple navigator constructor will accept a `namespace` field to support multi-robot applications or namespaced Nav2 launches.

中文注解：构造函数支持 `namespace`，用于多机器人或带命名空间的 Nav2 bringup。

| Robot Navigator Method            | Description                                                                |
| --------------------------------- | -------------------------------------------------------------------------- |
| setInitialPose(initial_pose)      | Sets the initial pose (`PoseStamped`) of the robot to localization.        |
| goThroughPoses(poses, behavior_tree='') | Requests the robot to drive through a set of poses (list of `PoseStamped`).|
| goToPose(pose, behavior_tree='')  | Requests the robot to drive to a pose (`PoseStamped`).                     |
| followWaypoints(poses)            | Requests the robot to follow a set of waypoints (list of `PoseStamped`). This will execute the specific `TaskExecutor` at each pose.   |
| followPath(path, controller_id='', goal_checker_id='') | Requests the robot to follow a path from a starting to a goal `PoseStamped`, `nav_msgs/Path`.     |
| spin(spin_dist=1.57, time_allowance=10)   | Requests the robot to performs an in-place rotation by a given angle.      |
| backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10) | Requests the robot to back up by a given distance.         |
| cancelTask()                       | Cancel an ongoing task request.|
| isTaskComplete()                   | Checks if task is complete yet, times out at `100ms`.  Returns `True` if completed and `False` if still going.                  |
| getFeedback()                     | Gets feedback from task, returns action server feedback object. |
| getResult()				        | Gets final result of task, to be called after `isTaskComplete` returns `True`. Returns action server result object. |
| getPath(start, goal, planner_id='', use_start=False) | Gets a path from a starting to a goal `PoseStamped`, `nav_msgs/Path`.      |
| getPathThroughPoses(start, goals, planner_id='', use_start=False) | Gets a path through a starting to a set of goals, a list of `PoseStamped`, `nav_msgs/Path`. |
| smoothPath(path, smoother_id='', max_duration=2.0, check_for_collision=False) | Smooths a given `nav_msgs/msg/Path` path. |
| changeMap(map_filepath)           | Requests a change from the current map to `map_filepath`'s yaml.           |
| clearAllCostmaps()                | Clears both the global and local costmaps.                                 |
| clearLocalCostmap()               | Clears the local costmap.                                                  |
| clearGlobalCostmap()              | Clears the global costmap.                                                 |
| getGlobalCostmap()                | Returns the global costmap, `nav2_msgs/Costmap`                            |
| getLocalCostmap()                 | Returns the local costmap, `nav2_msgs/Costmap`                             |
| waitUntilNav2Active(navigator='bt_navigator, localizer='amcl') | Blocks until Nav2 is completely online and lifecycle nodes are in the active state. To be used in conjunction with autostart or external lifecycle bringup. Custom navigator and localizer nodes can be specified  |
| lifecycleStartup()                | Sends a request to all lifecycle management servers to bring them into the active state, to be used if autostart is `false` and you want this program to control Nav2's lifecycle. |
| lifecycleShutdown()               | Sends a request to all lifecycle management servers to shut them down.     |
| destroyNode()                     | Releases the resources used by the object.                                 |

A general template for building applications is as follows:

中文注解：下面模板展示典型调用顺序：初始化 rclpy、创建 navigator、设置初始位姿、等待 Nav2 active、规划/平滑/导航、轮询 feedback、读取结果。

``` python3

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

rclpy.init()

nav = BasicNavigator()
...
nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # if autostarted, else use `lifecycleStartup()`
...
path = nav.getPath(init_pose, goal_pose)
smoothed_path = nav.smoothPath(path)
...
nav.goToPose(goal_pose)
while not nav.isTaskComplete():
	feedback = nav.getFeedback()
	if feedback.navigation_duration > 600:
		nav.cancelTask()
...
result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
elif result == TaskResult.FAILED:
    print('Goal failed!')
```

## Usage of Demos and Examples

Make sure to install the `aws_robomaker_small_warehouse_world` package or build it in your local workspace alongside Nav2. It can be found [here](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world). The demonstrations, examples, and launch files assume you're working with this gazebo world (such that the hard-programmed shelf locations and routes highlighting the API are meaningful).

中文注解：示例坐标依赖 AWS warehouse world；如果换地图，需要同步修改 demo 中的货架、巡逻、巡检和目标点坐标。

Make sure you have set the model directory of turtlebot3 simulation and aws warehouse world to the `GAZEBO_MODEL_PATH`. There are 2 main ways to run the demos of the `nav2_simple_commander` API.

### Automatically

The main benefit of this is automatically showing the above demonstrations in a single command for the default robot model and world. This will make use of Nav2's default robot and parameters set out in the main simulation launch file in `nav2_bringup`.

``` bash
# Launch the launch file for the demo / example
ros2 launch nav2_simple_commander  security_demo_launch.py
```

This will bring up the robot in the AWS Warehouse in a reasonable position, launch the autonomy script, and complete some task to demonstrate the `nav2_simple_commander` API.

### Manually

The main benefit of this is to be able to launch alternative robot models or different navigation configurations than the default for a specific technology demonstration. As long as Nav2 and the simulation (or physical robot) is running, the simple python commander examples / demos don't care what the robot is or how it got there. Since the examples / demos do contain hard-programmed item locations or routes, you should still utilize the AWS Warehouse. Obviously these are easy to update if you wish to adapt these examples / demos to another environment.

``` bash
# Terminal 1: launch your robot navigation and simulation (or physical robot). For example
ros2 launch nav2_bringup tb3_simulation_launch.py world:=/path/to/aws_robomaker_small_warehouse_world/.world map:=/path/to/aws_robomaker_small_warehouse_world/.yaml

# Terminal 2: launch your autonomy / application demo or example. For example
ros2 run nav2_simple_commander demo_security
```

Then you should see the autonomy application running!

## Examples

The `nav2_simple_commander` has a few examples to highlight the API functions available to you as a user:

- `example_nav_to_pose.py` - Demonstrates the navigate to pose capabilities of the navigator, as well as a number of auxiliary methods.
- `example_nav_through_poses.py` - Demonstrates the navigate through poses capabilities of the navigator, as well as a number of auxiliary methods.
- `example_waypoint_follower.py` - Demonstrates the waypoint following capabilities of the navigator, as well as a number of auxiliary methods.
- `example_follow_path.py` - Demonstrates the path following capabilities of the navigator, as well as a number of auxiliary methods such as path smoothing.
## Demos

The `nav2_simple_commander` has a few demonstrations to highlight a couple of simple autonomy applications you can build using the `nav2_simple_commander` API:

- `demo_security.py` - A simple security robot application, showing how to have a robot follow a security route using Navigate Through Poses to do a patrol route, indefinitely. 
- `demo_picking.py` - A simple item picking application, showing how to have a robot drive to a specific shelf in a warehouse to either pick an item or have a person place an item into a basket and deliver it to a destination for shipping using Navigate To Pose.
- `demo_inspection.py` - A simple shelf inspection application, showing how to use the Waypoint Follower and task executors to take pictures, RFID scans, etc of shelves to analyze the current shelf statuses and locate items in the warehouse.

---

# Nav2 Simple Commander 中文翻译

## 概述

这个包的目标是为 Python3 用户提供一种“将导航作为库使用”的能力。它提供的 API 会替你处理 ROS 2 和 Action Server 相关的细节，让你可以专注于构建使用 Nav2 能力的应用程序。这个包也提供了 demo 和 API 使用示例，用来构建自主移动机器人中常见的基础能力。

该包由 [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) 在 [Samsung Research](https://www.sra.samsung.com/) 构建，最初的原型用于 [2021 ROS Developers Day](https://www.theconstructsim.com/ros-developers-day-2021/) 大会的主题演讲，代码可以在[这里](https://github.com/SteveMacenski/nav2_rosdevday_2021)找到。

## API

更多参数说明请参考它的 [API Guide Page](https://navigation.ros.org/commander_api/index.html)。

Basic Navigator 提供的方法如下，表中列出了输入和预期返回值。如果某个 server 失败，它可能抛出异常，也可能返回 `None` 对象。因此，请务必用 try/catch 正确包裹导航调用，并检查结果是否为 `None` 类型。

2023 年 9 月新增：Simple Navigator 构造函数支持 `namespace` 字段，用于支持多机器人应用或带命名空间的 Nav2 launch。

| Robot Navigator 方法 | 说明 |
| --- | --- |
| setInitialPose(initial_pose) | 将机器人的初始位姿（`PoseStamped`）设置给定位系统。 |
| goThroughPoses(poses, behavior_tree='') | 请求机器人依次经过一组位姿（`PoseStamped` 列表）。 |
| goToPose(pose, behavior_tree='') | 请求机器人导航到一个位姿（`PoseStamped`）。 |
| followWaypoints(poses) | 请求机器人跟随一组航点（`PoseStamped` 列表）。机器人会在每个位姿处执行指定的 `TaskExecutor`。 |
| followPath(path, controller_id='', goal_checker_id='') | 请求机器人沿着从起点到目标点的路径行驶，输入为 `nav_msgs/Path`。 |
| spin(spin_dist=1.57, time_allowance=10) | 请求机器人按给定角度原地旋转。 |
| backup(backup_dist=0.15, backup_speed=0.025, time_allowance=10) | 请求机器人按给定距离后退。 |
| cancelTask() | 取消正在执行的任务请求。 |
| isTaskComplete() | 检查任务是否完成，超时时间为 `100ms`。完成返回 `True`，仍在执行返回 `False`。 |
| getFeedback() | 获取任务反馈，返回 action server 的 feedback 对象。 |
| getResult() | 获取任务最终结果，应在 `isTaskComplete` 返回 `True` 后调用。返回 action server 的 result 对象。 |
| getPath(start, goal, planner_id='', use_start=False) | 获取从起点到目标点的路径，输入为 `PoseStamped`，返回 `nav_msgs/Path`。 |
| getPathThroughPoses(start, goals, planner_id='', use_start=False) | 获取从起点经过一组目标点的路径，目标点为 `PoseStamped` 列表，返回 `nav_msgs/Path`。 |
| smoothPath(path, smoother_id='', max_duration=2.0, check_for_collision=False) | 对给定的 `nav_msgs/msg/Path` 路径进行平滑。 |
| changeMap(map_filepath) | 请求将当前地图切换为 `map_filepath` 对应的 yaml 地图。 |
| clearAllCostmaps() | 清除全局和局部代价地图。 |
| clearLocalCostmap() | 清除局部代价地图。 |
| clearGlobalCostmap() | 清除全局代价地图。 |
| getGlobalCostmap() | 返回全局代价地图，类型为 `nav2_msgs/Costmap`。 |
| getLocalCostmap() | 返回局部代价地图，类型为 `nav2_msgs/Costmap`。 |
| waitUntilNav2Active(navigator='bt_navigator, localizer='amcl') | 阻塞等待 Nav2 完全上线，并等待 lifecycle 节点进入 active 状态。它可以配合 autostart 或外部 lifecycle bringup 使用，也可以指定自定义 navigator 和 localizer 节点。 |
| lifecycleStartup() | 向所有 lifecycle 管理 server 发送请求，使其进入 active 状态。适用于 autostart 为 `false` 且希望由本程序控制 Nav2 lifecycle 的情况。 |
| lifecycleShutdown() | 向所有 lifecycle 管理 server 发送请求，使其关闭。 |
| destroyNode() | 释放该对象使用的资源。 |

构建应用程序的一般模板如下：

``` python3

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

rclpy.init()

nav = BasicNavigator()
...
nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # 如果已 autostart，否则使用 `lifecycleStartup()`
...
path = nav.getPath(init_pose, goal_pose)
smoothed_path = nav.smoothPath(path)
...
nav.goToPose(goal_pose)
while not nav.isTaskComplete():
	feedback = nav.getFeedback()
	if feedback.navigation_duration > 600:
		nav.cancelTask()
...
result = nav.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
elif result == TaskResult.FAILED:
    print('Goal failed!')
```

## Demo 和示例的使用

请确保已经安装 `aws_robomaker_small_warehouse_world` 包，或者已经在本地工作空间中与 Nav2 一起构建它。它可以在[这里](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)找到。演示、示例和 launch 文件都假设你正在使用这个 Gazebo world，这样硬编码的货架位置和路线才有意义，才能有效展示 API。

请确保已经将 TurtleBot3 仿真和 AWS warehouse world 的模型目录设置到 `GAZEBO_MODEL_PATH`。运行 `nav2_simple_commander` API demo 主要有两种方式。

### 自动运行

这种方式的主要好处是，可以用一条命令为默认机器人模型和 world 自动展示上述演示。它会使用 `nav2_bringup` 主仿真 launch 文件中定义的 Nav2 默认机器人和参数。

``` bash
# 启动 demo / example 的 launch 文件
ros2 launch nav2_simple_commander  security_demo_launch.py
```

这会在 AWS Warehouse 中以合理的位置启动机器人，启动自主脚本，并完成一些任务来展示 `nav2_simple_commander` API。

### 手动运行

这种方式的主要好处是，可以为特定技术演示启动替代机器人模型或不同于默认值的导航配置。只要 Nav2 和仿真环境（或实体机器人）正在运行，Simple Python Commander 示例和 demo 并不关心机器人是什么，也不关心机器人是如何启动的。由于这些示例和 demo 确实包含硬编码的物品位置或路线，因此仍然应该使用 AWS Warehouse。如果你希望把这些示例和 demo 适配到另一个环境，显然也很容易更新这些内容。

``` bash
# 终端 1：启动机器人导航和仿真环境（或实体机器人）。例如：
ros2 launch nav2_bringup tb3_simulation_launch.py world:=/path/to/aws_robomaker_small_warehouse_world/.world map:=/path/to/aws_robomaker_small_warehouse_world/.yaml

# 终端 2：启动自主应用 demo 或示例。例如：
ros2 run nav2_simple_commander demo_security
```

随后你应该能看到自主应用正在运行。

## 示例

`nav2_simple_commander` 提供了几个示例，用来突出展示可供用户使用的 API 函数：

- `example_nav_to_pose.py` - 演示 navigator 的导航到位姿能力，以及若干辅助方法。
- `example_nav_through_poses.py` - 演示 navigator 的经过多位姿导航能力，以及若干辅助方法。
- `example_waypoint_follower.py` - 演示 navigator 的航点跟随能力，以及若干辅助方法。
- `example_follow_path.py` - 演示 navigator 的路径跟随能力，以及路径平滑等若干辅助方法。

## Demo

`nav2_simple_commander` 提供了几个演示，用来突出展示可以使用 `nav2_simple_commander` API 构建的简单自主应用：

- `demo_security.py` - 一个简单的安防机器人应用，展示如何让机器人使用 Navigate Through Poses 沿安防路线无限期执行巡逻路线。
- `demo_picking.py` - 一个简单的物品拣选应用，展示如何让机器人行驶到仓库中的指定货架，拣选物品，或让人员把物品放入篮子中，再将其送到发货目的地。该过程使用 Navigate To Pose。
- `demo_inspection.py` - 一个简单的货架巡检应用，展示如何使用 Waypoint Follower 和 task executor 对货架拍照、扫描 RFID 等，从而分析当前货架状态并定位仓库中的物品。

---

# 本项目源码结构详细解析

本节以当前仓库代码为准，重点解释 `launch/` 和 Python 包目录
`nav2_simple_commander/`。前面的上游说明用于快速查 API，本节用于读源码、改启动链路和
排查数据流。

## 1. 包的整体定位

`nav2_simple_commander` 不是新的导航算法，也不替代 Nav2 Server。它是位于业务程序和
Nav2 Action／Service 之间的 Python 客户端封装：

```text
Python 业务脚本
  -> BasicNavigator 同步风格 API
  -> ROS 2 Action / Service / Topic
  -> bt_navigator / planner_server / controller_server
     / smoother_server / behavior_server / map_server
```

包内两类目录的关系如下：

```text
launch/
  负责启动仿真、桥接、定位、Nav2 Server、RViz 和示例进程

nav2_simple_commander/
  robot_navigator.py：封装 Nav2 通信
  example_*.py：演示单项 API
  demo_*.py：演示业务流程
  costmap_2d.py / footprint_collision_checker.py / line_iterator.py：
    Python 代价地图与碰撞检查工具
```

`setup.py` 把 Python 文件注册为 `ros2 run` 可执行程序，并把 `launch/`、`params/`、
`rviz/`、`urdf/` 和 `models/` 安装到包的 share 目录。因此启动文件使用
`get_package_share_directory('nav2_simple_commander')` 时，读取的是安装空间内容，不是
源码目录本身。修改后必须重新构建并 source 工作空间，运行时才会看到新版本。

## 2. launch 目录总览

### 2.1 启动层次

所有 `*_example_launch.py` 和 `*_demo_launch.py` 都是顶层一键启动入口，基本链路一致：

```text
顶层 example / demo launch
  ├── Ignition Gazebo：world_only.sdf
  ├── ros_ign_bridge.launch.py：仿真与 ROS 2 消息桥接
  ├── robot_state_publisher：URDF -> /tf、/tf_static
  ├── RViz：可选
  ├── localization_launch.py
  │     ├── map_server
  │     ├── amcl
  │     └── lifecycle_manager_localization
  ├── navigation_launch.py
  │     ├── controller_server
  │     ├── smoother_server
  │     ├── planner_server
  │     ├── behavior_server
  │     ├── bt_navigator
  │     ├── waypoint_follower
  │     ├── velocity_smoother
  │     └── lifecycle_manager_navigation
  └── 对应 example_* 或 demo_* Python 进程
```

这套启动文件使用本包自带的：

- `models/myworld2/world_only.sdf`：Ignition 世界和机器人模型入口。
- `models/myworld2/myworld2.yaml`：静态地图描述。
- `params/myworld2.yaml`：定位和导航参数。
- `urdf/diffbot.urdf`：机器人 TF 结构。
- `rviz/nav2_default_view.rviz`：RViz 显示配置。

### 2.2 顶层示例 launch 的公共流程

以下文件除进程名和 Ignition partition 名外，启动结构基本相同：

- `nav_to_pose_example_launch.py`
- `nav_through_poses_example_launch.py`
- `waypoint_follower_example_launch.py`
- `follow_path_example_launch.py`
- `assisted_teleop_example_launch.py`
- `recoveries_example_launch.py`
- `security_demo_launch.py`
- `picking_demo_launch.py`
- `inspection_demo_launch.py`

公共流程分为七步。

#### 第一步：解析安装空间资源

```python
commander_dir = get_package_share_directory('nav2_simple_commander')
myworld_dir = os.path.join(commander_dir, 'models', 'myworld2')
```

随后组合地图、世界、参数、RViz 和 URDF 的绝对路径。URDF 在生成 LaunchDescription 时
直接读成字符串，再作为 `robot_description` 参数传给 `robot_state_publisher`。

#### 第二步：声明 launch 参数

顶层文件统一声明：

| 参数 | 默认值 | 实际作用 |
| --- | --- | --- |
| `use_rviz` | `True` | 控制是否启动 RViz |
| `use_sim_time` | `true` | 让 ROS 节点使用 `/clock` |
| `world_name` | `myworld2` | 当前只声明和读取，没有参与 world 路径或话题构造 |

`world_name` 当前不会切换世界。要更换世界，应修改 world 文件路径，或把该参数真正接入
路径生成逻辑。

#### 第三步：隔离 Ignition 实例

每个入口使用“功能名 + 当前进程 PID”生成独立 partition，例如：

```text
nav2_simple_commander_nav_to_pose_<pid>
nav2_simple_commander_security_<pid>
```

该值同时写入 `IGN_PARTITION` 和 `GZ_PARTITION`，避免多个示例启动时串到同一套仿真
Transport 网络。

`IGN_GAZEBO_RESOURCE_PATH` 则加入 ROS share、本包 models 和 myworld2 目录，使 SDF 中的
模型 URI 能被解析。

#### 第四步：启动 Ignition Gazebo

```text
ign gazebo -r -v 3 <world_only.sdf>
```

`-r` 表示启动后直接运行仿真，`-v 3` 设置 Ignition 日志等级。这里使用
`ExecuteProcess`，Gazebo 不是 ROS Lifecycle Node。

#### 第五步：启动桥接、TF 和 RViz

- `ros_ign_bridge.launch.py` 把速度、里程计、TF、时钟和传感器消息跨到 ROS 2。
- `robot_state_publisher` 从 URDF 发布机器人关节和静态 TF。
- RViz 受 `use_rviz` 条件控制。
- `SPDLOG_WRAPPER_LOG_DIR=/tmp/nav2_logs` 指定项目中 spdlog wrapper 的日志目录。

#### 第六步：包含定位与导航 launch

顶层入口把地图、参数和仿真时间传给 `localization_launch.py`，再把参数和仿真时间传给
`navigation_launch.py`。

顶层文件还向定位 launch 传入 `localization_child_frame`、`localization_x/y/z/yaw` 和
`localization_tf_time_offset`。但当前 `localization_launch.py` 没有声明或使用这些参数，
因此它们不会改变 AMCL 初始姿态或 TF；真正生效的定位初值来自参数文件和
`BasicNavigator.setInitialPose()`。

#### 第七步：启动对应 Python 程序

顶层入口最后用 `Node(package='nav2_simple_commander', executable=...)` 启动业务脚本。
各入口的唯一业务差异如下：

| launch 文件 | 启动的可执行程序 | 演示目标 |
| --- | --- | --- |
| `nav_to_pose_example_launch.py` | `example_nav_to_pose` | 单目标导航、取消和新目标 |
| `nav_through_poses_example_launch.py` | `example_nav_through_poses` | 多目标连续导航 |
| `waypoint_follower_example_launch.py` | `example_waypoint_follower` | 航点跟随和航点任务 |
| `follow_path_example_launch.py` | `example_follow_path` | 规划、平滑并直接跟踪路径 |
| `assisted_teleop_example_launch.py` | `example_assisted_teleop` | 启动辅助遥控行为 |
| `recoveries_example_launch.py` | `demo_recoveries` | 导航后显式调用后退和旋转 |
| `security_demo_launch.py` | `demo_security` | 循环安防巡逻 |
| `picking_demo_launch.py` | `demo_picking` | 货架取件和配送 |
| `inspection_demo_launch.py` | `demo_inspection` | 货架航点巡检 |

### 2.3 `localization_launch.py`

该文件只负责定位子系统，不启动传感器或机器人模型。

主要输入：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `namespace` | 空 | 多机器人顶层命名空间 |
| `map` | `myworld_bringup/maps/out.yaml` | Map Server 加载的地图 |
| `use_sim_time` | `false` | 是否使用仿真时钟 |
| `params_file` | `myworld_bringup/params/myworld2.yaml` | 节点参数文件 |
| `autostart` | `true` | Lifecycle Manager 是否自动激活节点 |
| `use_composition` | `False` | 独立进程或组件模式 |
| `container_name` | `nav2_container` | 组件模式目标容器 |
| `use_respawn` | `False` | 独立进程崩溃后是否重启 |
| `log_level` | `info` | ROS 日志等级 |

`RewrittenYaml` 会在运行时覆盖参数文件中的：

```text
use_sim_time <- launch 参数
yaml_filename <- map 参数
```

独立进程模式启动 `map_server`、`amcl` 和
`lifecycle_manager_localization`。Lifecycle Manager 只管理前两个节点，并按
`autostart` 执行 configure 和 activate。

组件模式改用 `LoadComposableNodes`，把同样的三个组件加载到
`<namespace>/<container_name>`。本文件不会创建 Component Container，因此单独把
`use_composition` 设为 `True` 之前，必须由外部先启动对应容器。

`/tf` 和 `/tf_static` 被重映射成相对名称 `tf`、`tf_static`，使 namespace 能参与话题
解析。

### 2.4 `navigation_launch.py`

该文件负责除定位外的 Nav2 主导航链路，参数机制和 composition 分支与定位 launch 相同。

`RewrittenYaml` 覆盖：

```text
use_sim_time <- launch 参数
autostart    <- launch 参数
```

Lifecycle Manager 管理以下节点：

| 节点 | 输入和职责 | 主要输出 |
| --- | --- | --- |
| `controller_server` | 路径、TF、里程计、局部代价地图；调用控制器插件 | `cmd_vel_nav` |
| `smoother_server` | 接收路径并调用 Smoother 插件 | 平滑后的 Path |
| `planner_server` | 目标、TF、全局代价地图；调用 Planner 插件 | 全局 Path |
| `behavior_server` | TF、局部代价地图、Footprint；执行恢复行为 | 行为结果、`cmd_vel` |
| `bt_navigator` | 接收导航 Action，执行行为树 | 导航反馈和结果 |
| `waypoint_follower` | 逐个执行 waypoint 及任务插件 | 航点进度和结果 |
| `velocity_smoother` | 限制速度、加速度和减速度 | 最终 `cmd_vel` |

正常控制速度链是：

```text
controller_server/cmd_vel
  --remap--> cmd_vel_nav
  -> velocity_smoother
  --remap--> cmd_vel
  -> 仿真桥接或底盘
```

`behavior_server` 没有把 `cmd_vel` 重映射到 `cmd_vel_nav`，因此 Spin、BackUp、
DriveOnHeading 和 AssistedTeleop 直接发布最终 `cmd_vel`，不经过这里的速度平滑器。

独立进程模式允许 `use_respawn=True`；组件模式则加载 C++ 组件到外部容器，不应用
`use_respawn`。当前顶层 example／demo launch 没有创建容器，所以默认使用独立进程模式。

另一个工程耦合点是：两个底层 launch 的默认资源目录都指向 `myworld_bringup`。顶层
Simple Commander 入口显式传入本包参数和地图，因此正常一键启动不依赖这些默认路径；
如果单独运行底层 launch，则需要确保 `myworld_bringup` 可发现，或显式传入 `map` 和
`params_file`。

### 2.5 `ros_ign_bridge.launch.py`

该文件启动一个 `ros_gz_bridge parameter_bridge`，桥接方向由参数字符串中的括号决定：

| 数据 | Ignition／Gazebo 到 ROS 2 | ROS 2 到 Ignition／Gazebo |
| --- | --- | --- |
| `/cmd_vel` | 否 | 是 |
| `/odom` | 是 | 否 |
| `/odom/tf` | 是 | 否 |
| `/clock` | 是 | 否 |
| `/joint_states` | 是 | 否 |
| `/scan`、`/scan/points` | 是 | 否 |
| `/imu` | 是 | 否 |
| `/camera/rgb/image_raw`、`camera_info` | 是 | 否 |

`/odom/tf` 被重映射为 ROS 2 的 `tf`。因此完整运动闭环是：

```text
Nav2 /cmd_vel
  -> ros_gz_bridge
  -> Ignition diff drive
  -> Ignition /odom、/odom/tf、/scan
  -> ros_gz_bridge
  -> Nav2 定位、代价地图与控制器
```

### 2.6 `warehouse.world`

这是一个传统 Gazebo world 资源文件，包含仓库场景模型、光照、地面和仿真参数。当前
一键启动链路实际使用 `models/myworld2/world_only.sdf`，没有引用该文件。因此修改
`warehouse.world` 不会影响当前 myworld2 示例，除非显式把 Gazebo 启动路径切换到它。

## 3. Python 包目录总览

### 3.1 `__init__.py`

该文件只用于把目录标记为 Python package，没有运行逻辑，也没有统一导出类。业务代码
应继续显式导入：

```python
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
```

### 3.2 `robot_navigator.py`：核心通信封装

`BasicNavigator` 继承 `rclpy.node.Node`。它没有自己的后台线程，而是通过
`spin_until_future_complete()` 和 `spin_once()` 驱动 Action、Service 和订阅回调。

#### 初始化的数据端点

Action Client：

| Client | Server | 对应 Nav2 节点 |
| --- | --- | --- |
| `NavigateToPose` | `navigate_to_pose` | `bt_navigator` |
| `NavigateThroughPoses` | `navigate_through_poses` | `bt_navigator` |
| `FollowWaypoints` | `follow_waypoints` | `waypoint_follower` |
| `FollowPath` | `follow_path` | `controller_server` |
| `ComputePathToPose` | `compute_path_to_pose` | `planner_server` |
| `ComputePathThroughPoses` | `compute_path_through_poses` | `planner_server` |
| `SmoothPath` | `smooth_path` | `smoother_server` |
| `Spin` | `spin` | `behavior_server` |
| `BackUp` | `backup` | `behavior_server` |
| `AssistedTeleop` | `assisted_teleop` | `behavior_server` |

Topic：

| 方向 | 名称 | 作用 |
| --- | --- | --- |
| 发布 | `initialpose` | 把初始位姿交给 AMCL |
| 订阅 | `amcl_pose` | 确认 AMCL 已接受初始定位并产生位姿 |

`amcl_pose` 使用 Reliable、Transient Local、Keep Last 1，新的 Commander 实例也能收到
AMCL 保留的最近位姿。

Service Client：

| 服务 | 作用 |
| --- | --- |
| `map_server/load_map` | 动态切换静态地图 |
| `global_costmap/clear_entirely_global_costmap` | 清空全局代价地图 |
| `local_costmap/clear_entirely_local_costmap` | 清空局部代价地图 |
| `global_costmap/get_costmap` | 读取全局 Costmap 消息 |
| `local_costmap/get_costmap` | 读取局部 Costmap 消息 |

#### 一次异步导航任务的数据流

以 `goToPose()` 为例：

```text
PoseStamped + behavior_tree
  -> 等待 navigate_to_pose Action Server
  -> send_goal_async()
  -> spin 到 Server 接受或拒绝 Goal
  -> 保存 goal_handle
  -> 保存 get_result_async() 返回的 result_future
  -> 调用方循环 isTaskComplete()
       ├── 每次最多 spin 100 ms
       ├── getFeedback() 读取统一 feedback 缓存
       └── 业务代码可取消任务或执行其他逻辑
  -> getResult() 把 ROS GoalStatus 转为 TaskResult
```

`goToPose()`、`goThroughPoses()`、`followWaypoints()`、`followPath()`、`spin()`、
`backup()` 和 `assistedTeleop()` 都采用该模型：函数会阻塞到 Goal 被接受，但不会阻塞到
任务执行结束。

`BasicNavigator` 只保存一组 `goal_handle`、`result_future`、`feedback` 和 `status`，因此
一个实例同一时间只适合跟踪一个前台任务。发送新任务会覆盖本地跟踪状态；即使 Server
支持抢占，业务程序也无法再通过这个实例轮询旧任务结果。

#### 同步规划、平滑和 Service 调用

`getPath()`、`getPathThroughPoses()` 和 `smoothPath()` 会一直 spin 到 Action Result，属于
同步阻塞调用。失败或拒绝返回 `None`，成功只向上层返回结果中的 Path。

`changeMap()`、清图和读取 Costmap 也会等待 Service 完成。调用端不需要自己管理 Future，
但应避免在要求低延迟的回调线程里直接调用这些方法。

#### 初始化定位与等待 Nav2

标准顺序为：

```text
setInitialPose(PoseStamped)
  -> 转成 PoseWithCovarianceStamped
  -> 发布 initialpose

waitUntilNav2Active()
  -> 等待 amcl lifecycle 为 active
  -> 重复发布 initialpose，直到收到 amcl_pose
  -> 等待 bt_navigator lifecycle 为 active
```

`waitUntilNav2Active()` 默认只显式检查 `amcl` 和 `bt_navigator`。其他 Server 是否可用，由
具体 API 中的 `wait_for_server()` 或 `wait_for_service()` 再保证。

当 bringup 使用 `autostart=False` 时，可调用 `lifecycleStartup()`。它会扫描 ROS graph 中
所有 `nav2_msgs/srv/ManageLifecycleNodes` 服务并发送 STARTUP；`lifecycleShutdown()` 则向
它们发送 SHUTDOWN。

#### 结果与取消

`cancelTask()` 对当前 `goal_handle` 调用 `cancel_goal_async()`。`isTaskComplete()` 把任何
非成功终态都视为“任务已完成”，`getResult()` 再映射为：

```text
STATUS_SUCCEEDED -> TaskResult.SUCCEEDED
STATUS_ABORTED   -> TaskResult.FAILED
STATUS_CANCELED  -> TaskResult.CANCELED
其他状态          -> TaskResult.UNKNOWN
```

#### 当前实现边界

- `destroy_node()` 销毁了大部分 Action Client，但没有显式销毁
  `assisted_teleop_client`；父类销毁节点时仍会回收实体，但显式清理列表并不完整。
- 所有 `wait_for_server()` 和 `wait_for_service()` 都是无限等待，没有全局超时；服务名、
  namespace 或 bringup 错误时，调用会持续阻塞并打印等待日志。
- 新任务会覆盖单一的任务状态槽。并发任务应使用多个 Navigator 实例或自行管理 Action
  Client，不能共享一个 `BasicNavigator`。

### 3.3 API 示例文件

#### `example_nav_to_pose.py`

流程：设置 `map` 坐标系初始位姿，等待 Nav2 Active，构造单个 Goal，调用
`goToPose()`，循环打印 ETA 和恢复次数，演示超时取消以及发送新 Goal，最后读取
`TaskResult` 并关闭 Lifecycle。

代码中的坐标是 myworld2 地图硬编码值。切换地图后必须同时修改初始位姿和目标位姿。

#### `example_nav_through_poses.py`

构造三个 `PoseStamped`，调用 `goThroughPoses()` 交给
`NavigateThroughPosesNavigator`。反馈包含当前导航时长、ETA 和恢复次数。示例还演示取消
以及用新的单点列表替换当前多点任务。

它与 Waypoint Follower 的区别是：该接口由 BT Navigator 处理一组导航目标，不会在每个
目标点自动运行 Waypoint Task Executor。

#### `example_waypoint_follower.py`

把三个 Pose 发送给 `followWaypoints()`。`waypoint_follower` 逐点导航，反馈中的
`current_waypoint` 表示正在处理的索引；到点后可按参数加载的 Task Executor 执行等待、
拍照或输入等任务。

示例在运行中发送新 waypoint 列表，随后只跟踪新任务 Future。

#### `example_follow_path.py`

该文件展示完整的“规划—平滑—跟踪”分层调用：

```text
getPath(initial_pose, goal_pose)
  -> planner_server /compute_path_to_pose
smoothPath(path)
  -> smoother_server /smooth_path
followPath(smoothed_path, controller_id='DWB')
  -> controller_server /follow_path
```

与 `goToPose()` 不同，`followPath()` 跳过 BT Navigator 和全局规划阶段，只执行给定 Path。
调用前必须检查 `getPath()` 和 `smoothPath()` 是否返回 `None`，否则失败结果会继续传入下游。

#### `example_assisted_teleop.py`

该文件设置初始位姿并调用 `assistedTeleop(time_allowance=20)`，让 Behavior Server 进入辅助
遥控状态。当前示例循环中只有“应发布遥控 Twist”的注释，没有创建
`cmd_vel_teleop` Publisher，也没有产生人工速度。因此单独运行它只会启动 Action 并等待，
真实遥控数据必须由键盘、手柄或另一个节点发布。

### 3.4 业务 Demo 文件

#### `demo_recoveries.py`

先导航到预设死胡同目标，然后依次显式调用：

```text
backup(0.5 m, 0.1 m/s)
spin(3.14 rad)
```

它演示的是 Commander 直接调用 Behavior Server，不是只能由行为树触发恢复。最后根据恢复
结果决定是否回到起点。

#### `demo_security.py`

将二维坐标列表转换为 Pose 列表，通过 `goThroughPoses()` 执行巡逻。每轮结束后反转
`security_route`，从而沿相反顺序返回。单轮导航超过 180 秒会取消；成功后继续下一轮，
取消或失败则退出或报告异常。

#### `demo_picking.py`

`shelf_positions` 和 `shipping_destinations` 保存业务位置表。流程是：

```text
接收货架 ID 和配送点 ID
  -> goToPose(货架)
  -> 成功后模拟人工装货
  -> goToPose(配送点)
  -> 货架任务取消时回到 staging 初始点
```

坐标、请求内容和人工交互均为示例中的本地变量，不是实际订单接口。

#### `demo_inspection.py`

生成一组货架前方的巡检 Pose，调用 `followWaypoints()`。反馈显示当前巡检点进度，所有点
完成或任务取消后，再用 `goToPose(initial_pose)` 返回起点。拍照、RFID 等能力应由实际
Waypoint Task Executor 或外部业务节点实现，脚本本身没有传感器处理代码。

### 3.5 `costmap_2d.py`

`PyCostmap2D` 把 Nav2 Costmap／OccupancyGrid 风格消息包装为 NumPy 一维数组，保留：

- 宽高 Cell 数；
- 分辨率；
- 世界坐标原点；
- frame ID 和时间戳；
- 行优先代价数组。

核心换算：

```text
index = my * size_x + mx
wx = origin_x + (mx + 0.5) * resolution
wy = origin_y + (my + 0.5) * resolution
mx = floor((wx - origin_x) / resolution)
my = floor((wy - origin_y) / resolution)
```

类本身不做坐标越界检查。调用 `getCostXY()` 或 `setCost()` 前，应保证 `mx`、`my` 位于
地图范围内。

### 3.6 `footprint_collision_checker.py`

`FootprintCollisionChecker` 先通过 `setCostmap()` 注入 `PyCostmap2D`，再计算机器人多边形
边界上的最大代价值。

`footprintCostAtPose()` 的数据流是：

```text
局部坐标 Footprint
  -> 按 theta 旋转
  -> 平移到目标 (x, y)
  -> 每个世界坐标顶点转为 Costmap Cell
  -> LineIterator 沿每条多边形边采样
  -> 读取每个 Cell 的 cost
  -> 返回边界最大 cost
```

代价值常量与 Nav2 保持一致：`FREE_SPACE=0`、`INSCRIBED_INFLATED_OBSTACLE=253`、
`LETHAL_OBSTACLE=254`、`NO_INFORMATION=255`。

当前实现只检查 Footprint 边界，不填充多边形内部。更重要的是，名为
`worldToMapValidated()` 的函数实际只调用 `PyCostmap2D.worldToMap()`，没有检查 Cell 是否
越界。负索引可能按 NumPy 规则从数组末尾取值，正向越界可能抛出异常。将它用于安全判断
前，应由调用方验证坐标范围，或在该函数中补齐严格边界检查。

### 3.7 `line_iterator.py`

`LineIterator` 是与 ROS 无关的二维线段采样器。它从 `(x0, y0)` 开始，每次按
`step_size` 推进：非竖直线优先推进 x 并用直线方程计算 y，竖直线只推进 y；`clamp()`
保证最后一步不越过终点。

Footprint Collision Checker 默认以 `0.5` 个 Map Cell 为步长调用它。更小步长会增加采样
密度和计算量，更大步长可能漏掉窄障碍。

当前构造函数处理水平线的条件写成 `elif y1 == y1 and x1 != x0`，其中 `y1 == y1` 对普通
数值恒为真。它在当前分支顺序下仍能覆盖水平线，但语义应当是 `y1 == y0`；维护时不要把
该表达式误认为额外的几何判断。

## 4. 从一键 launch 到机器人运动的完整数据流

```text
ros2 launch nav2_simple_commander nav_to_pose_example_launch.py
  -> Ignition 加载 world_only.sdf
  -> bridge 建立 /clock、/scan、/odom、/tf、/cmd_vel 通道
  -> Map Server + AMCL 激活
  -> Nav2 Navigation Servers 激活
  -> example_nav_to_pose 创建 BasicNavigator
  -> initialpose -> AMCL -> amcl_pose + map->odom TF
  -> NavigateToPose Goal -> bt_navigator
  -> planner_server 生成 Path
  -> controller_server 计算 cmd_vel_nav
  -> velocity_smoother 输出 cmd_vel
  -> bridge 把 cmd_vel 交给 Ignition
  -> Ignition 更新机器人位姿、里程计和激光
  -> 新数据返回 AMCL、Costmap 和 Controller
  -> Action Feedback -> BasicNavigator.feedback
  -> Action Result -> TaskResult
```

## 5. 修改与排障建议

### 修改 Python API 或示例后

从 `nav2_ws/` 同步并重建该包：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
colcon build --symlink-install --packages-select nav2_simple_commander
source install/setup.bash
```

### Launch 看不到修改

`ros2 launch` 使用 install/share 中的文件。先确认当前环境解析到哪里：

```bash
ros2 pkg prefix nav2_simple_commander
```

然后重新构建并重新 source，不能只修改源码目录后直接判断 launch 没生效。

### 示例一直等待 Server

按依赖顺序检查：

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /bt_navigator
ros2 action list
ros2 service list | grep costmap
```

如果使用 namespace，`BasicNavigator(namespace='...')`、Nav2 bringup namespace 和 TF／话题
重映射必须一致。

### 仿真有速度但机器人不动

检查最终 `/cmd_vel`、桥接节点和 Ignition partition。顶层 launch 使用进程 PID 隔离
partition，手动启动 bridge 或 Gazebo 时必须处于同一个 `IGN_PARTITION`／`GZ_PARTITION`。

## 6. 当前包级一致性注意项

- `setup.py` 中版本为 `1.0.0`，`package.xml` 中版本为 `1.1.20`，发布或生成包元数据前应
  统一；本次只做文档解析，没有修改版本。
- `setup.py` 的 `console_scripts` 决定 `ros2 run` 可执行名。新增 example／demo 文件后，
  仅创建 Python 文件还不够，还需要增加入口并重新构建。
- 本包的示例坐标全部绑定 myworld2／仓库地图。更换地图时必须同步修改初始位姿、目标点、
  业务位置表和可能的机器人 Footprint。
