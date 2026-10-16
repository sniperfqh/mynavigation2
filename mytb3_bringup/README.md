# mytb3_bringup

This package owns the TurtleBot3 simulation launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, URDF, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 当前入口与资源

公开入口为 [entry.launch.py](launch/entry.launch.py)，原说明中的 `tb3_simulation_launch.py` 当前不存在。入口组织 Ignition Gazebo、TurtleBot3 模型、ROS 桥接、定位、导航和 RViz；模型默认取 `TURTLEBOT3_MODEL`，未设置时为 `waffle`。

从 `nav2_ws` 加载 ROS 2 Humble 与本工作空间后运行：

```bash
ros2 launch mytb3_bringup entry.launch.py use_rviz:=true
```

本场景默认 `use_sim_time=true`，与 myworld_bringup 当前系统时间／外部定位入口不同，不混用其 TF 与时钟设置。默认地图为 maps/turtlebot3_world.yaml，参数由模型选择 params/<model>.yaml；实际启动选项以 entry 源码为准。

## 目录与使用边界

[launch/](launch/) 包含仿真总入口、桥接、机器人 TF、定位和导航子入口；[params/](params/)、[maps/](maps/)、[models/](models/)、[urdf/](urdf/)、[rviz/](rviz/) 保存场景资源。本包不单独实现规划和控制算法，导航插件来自其他 Nav2 包。

构建依赖见 [package.xml](package.xml)，安装资源见 [CMakeLists.txt](CMakeLists.txt)，通用构建约定见[仓库说明](../README.md)。启动需要 Ignition 与 ros_gz 系列组件，资源完整不等同于本机运行验收。
