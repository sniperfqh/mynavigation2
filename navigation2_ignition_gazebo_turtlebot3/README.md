# navigation2_ignition_gazebo_turtlebot3
在 Ignition Gazebo 仿真器中使用 Nav2 导航仿真的 TurtleBot 3。

使用 `ROS_LOCALHOST_ONLY=1 TURTLEBOT3_MODEL=waffle ros2 launch turtlebot3 simulation.launch.py` 同时启动仿真、Nav2 和 RViz2。

![TurtleBot3 截图](./docs/media/turtlebot3scr.png)

`/odom` topic、`odom` frame 和 `/odom/tf`（tf topic）定义在 `model.sdf` 中。`base_footprint` 在 `odom` frame 下的变换通过 `/odom/tf` 发布。`/odom` topic 发布 `odom` frame 在 `map` frame 下的变换。

使用 `ros2 run tf2_tools view_frames` 查看 tf 坐标系关系。

Ignition Gazebo 发布 `joint_states`，随后通过 `ros_ign_bridge` 转换为 ROS 2 topic，并由 `robot_state_publisher`（一个 ROS 2 节点）消费，用于计算和发布大部分 tf。

`/odom/tf` 被重映射到 `/tf`。

Ignition Gazebo topic 通过 `ros_ign_bridge` 与 ROS 2 topic 相互转换。

调用 `nav2_bringup` 来初始化基础服务和配置。

已在 Ignition Gazebo Fortress 和 ROS 2 Humble 上测试。

依赖项：
  - `ros-<distro>-navigation2`
  - `ros-<distro>-nav2-bringup`
  - `ros-<distro>-ros-ign-gazebo`
  - `ros-<distro>-ros-ign-bridge`

## 中文翻译

# navigation2_ignition_gazebo_turtlebot3

该包展示在 Ignition Gazebo 中运行 TurtleBot3 仿真与 Nav2 导航。使用原文命令可同时启动仿真、Nav2 和 RViz2。/odom、odom Frame 和 /odom/tf 在 model.sdf 中定义，Gazebo 通过 ros_ign_bridge 转换 joint_states 和其他 Topic，robot_state_publisher 根据关节状态发布 TF。/odom/tf 被重映射为 /tf，nav2_bringup 负责初始化基础服务和配置。

原文环境在 Ignition Gazebo Fortress 与 ROS 2 Humble 上测试。依赖包括 navigation2、nav2-bringup、ros-ign-gazebo 和 ros-ign-bridge。
