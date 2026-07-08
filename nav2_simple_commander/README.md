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
