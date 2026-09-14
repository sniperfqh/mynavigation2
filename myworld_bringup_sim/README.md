# myworld_bringup_sim

This package owns the `myworld2` launch flow.

- Keep scenario-specific launch files in `launch/`.
- Keep this scenario's map, model, RViz, and parameter assets in this
  package.
- Use `entry.launch.py` as the public entry point.

## 中文翻译

# myworld_bringup_sim

该包负责 myworld2 场景的 Launch 流程。场景专用 Launch 文件放在 launch/，地图、模型、RViz 和 URDF 资源放在本包中；公开入口是 entry.launch.py。

当前 `nav2_collision_monitor`、其生命周期管理器和碰撞边界可视化已注释停用；`velocity_smoother` 直接将平滑速度发布到 `/cmd_vel`。原碰撞参数和启动代码保留在注释中，恢复时需同步打开对应模块。
