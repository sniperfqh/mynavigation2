# myagv_test_bringup

This package owns the `myagv_test` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, URDF, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 当前入口

公开入口为 [entry.launch.py](launch/entry.launch.py)，旧说明中的 `myagv_test.launch.py` 当前不存在。本包保留标准 BT Nav2 兼容链，与推荐的 [nav2_regulated_modules](../nav2_regulated_modules/README.md) 无 BT 三模式入口分开使用。

从 `nav2_ws` 加载 ROS 2 Humble 和本工作空间后运行：

```bash
ros2 launch myagv_test_bringup entry.launch.py
```

当前默认面向实车：不启动仿真、AMCL 或机器人 TF。外部系统提供定位 TF、激光和里程计；导航 `/cmd_vel` 由 controlpub 转到 `/control_to_uart`。默认地图来自本包 maps/out.yaml，参数来自 params/nav2_params.yaml；可用 `map`、`params_file` 覆盖。

## 资源与链路

- [launch/navigation_launch.py](launch/navigation_launch.py)：标准规划、控制、BT 与相关服务器启动。
- [params/](params/)：BT、控制器、代价地图等参数；是否调用 SmoothPath 取决于实际 BT。
- [maps/](maps/)、[rviz/](rviz/)、[urdf/](urdf/)、[models/](models/)：地图、显示与兼容资源；资源存在不表示默认加载。
- [package.xml](package.xml)、[CMakeLists.txt](CMakeLists.txt)：依赖与安装范围。

可只读检查 `/navigate_to_pose`、`/follow_path` 和生命周期状态。构建方法及入口差异见[仓库总说明](../README.md)。本轮仅核对文档，不启动后台或运行导航。
