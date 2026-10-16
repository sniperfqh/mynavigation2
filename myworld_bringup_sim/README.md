# myworld_bringup_sim

This package owns the `myworld2` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 当前用途

这是独立保留的 myworld 仿真入口，不是 myworld_bringup 的同步副本。入口 [entry.launch.py](launch/entry.launch.py) 启动 Ignition、机器人描述、桥接、定位与导航，默认 `use_sim_time=true`，`IGN_PARTITION=myworld_bringup_sim`。

从已加载环境的 `nav2_ws` 终端运行：

```bash
ros2 launch myworld_bringup_sim entry.launch.py use_rviz:=true
```

Collision Monitor、其生命周期管理器和碰撞边界可视化在本入口中仍注释停用；平滑器直接输出 `/cmd_vel`。本入口没有 myworld_bringup 的三模式及 collision 参数转发契约，不照搬 `operation_mode` 或 `enable_collision_state` 命令。

## 资源与依赖

[launch/](launch/) 保存总入口、ROS 桥接、定位和导航子入口；[params/](params/)、[models/](models/)、[urdf/](urdf/)、[rviz/](rviz/) 保存场景配置。依赖与安装范围见 [package.xml](package.xml)、[CMakeLists.txt](CMakeLists.txt)。构建与推荐入口选择见[仓库总说明](../README.md)。本轮未运行此兼容场景。
