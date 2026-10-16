# turtlebot3

## 当前项目用法

安装 Ignition Gazebo TurtleBot3 的 Launch、地图、模型和参数；ROS 包名为 turtlebot3。

包类型：仿真资源包。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[launch/](launch/)、[params/](params/)、[models/](models/)、[maps/](maps/)。

直接依赖（清单节选，完整约束见 manifest）：`robot_state_publisher`、`ros_gz_bridge`、`ros_gz_sim`、`tf2_ros`。

启动资源：[navigation2.launch.py](launch/navigation2.launch.py)、[robot_state_publisher.launch.py](launch/robot_state_publisher.launch.py)、[ros_ign_bridge.launch.py](launch/ros_ign_bridge.launch.py)、[simulation.launch.py](launch/simulation.launch.py)。执行前先加载工作空间环境，具体依赖由所选入口决定。

### 本项目接入说明

该包位于 navigation2_ignition_gazebo_turtlebot3/turtlebot3/，构建选包名称为 turtlebot3；不要使用目录名 navigation2_ignition_gazebo_turtlebot3 代替 ROS 包名。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
