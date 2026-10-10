# 速度链日志

`regulated_modules.launch.py` 默认启动独立 `velocity_diagnostics_node`，只订阅话题，不发布运动指令。

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py operation_mode:=fixed_path log_dir:=/tmp/nav2_logs velocity_file_log_frequency:=100.0 velocity_console_log_frequency:=1.0
```

仿真 `ros2 launch myworld_bringup entry.launch.py operation_mode:=fixed_path` 同样支持这些参数。`enable_velocity_diagnostics:=false` 关闭独立诊断，保留原有模块日志。

## 输出与字段

- 文件：`<log_dir>/nav2_regulated_modules/velocity_diagnostics.log`，DEBUG 快照默认 100 Hz；INFO 终端摘要默认 1 Hz，也进入文件。
- 每文件 10 MiB，保留 5 个滚动备份，每秒刷新，正常退出排空异步队列；日志采用追加写入。建议每次实验指定独立 `log_dir`。
- `ros_time` 为 ROS 时钟；`age` 使用单调时钟，单位秒。`vx` 单位 m/s、`wz` 单位 rad/s；`count` 是该订阅累计收到的消息数。
- `status` 分为 `missing`、`invalid`、`stale`、`valid`；默认超过 0.2 秒未更新为 stale。missing 的零值只是占位值，不代表停稳。
- 导航额外记录 `ChassisControl` 支路输入（`vx/wz` 是原始请求字段，`op` 表示方向，未收到时 `op=-1`）；导航记录控制器输出／平滑器输入、平滑器输出、启用时的碰撞监控输出、控制器反馈、平滑器反馈、底盘反馈及串口出口；remote 记录 `/control_to_uart` 的 `ControlRes.v/w` 和 `/motion_state` 的实际反馈。遥控输入是键盘事件，没有输入速度话题。
- 控制器、平滑器的反馈话题取自同一参数 YAML；`/motion_state` 使用 `MotionState.v_car/w_car`，其他反馈话题使用 `Odometry.twist`。

## 边界

100 Hz 是最新值的周期快照，不是每条消息的逐帧归档；重复 count 表示未收到新数据。各阶段来自独立订阅，不能当作同一控制周期的严格同步输入输出。话题观测不是平滑器内部校正参考值。

日志使用独立进程、单线程 ROS 执行器和异步滚动文件写入；8192 项队列拥塞时覆盖最旧记录，不能保证绝对无丢失。文件不可写时明确报错并仅保留终端摘要，不影响原控制链。

相关主 launch 子进程使用 `output='both'`，终端输出另由 ROS launch 保存到其启动时提示的 ROS 日志目录。该路径与 `log_dir` 独立；全局 ROS 日志目录仍由 `ROS_LOG_DIR` 管理。

## 验证

```bash
ROS_DOMAIN_ID=167 ROS_LOG_DIR=/tmp/velocity-diagnostics-ros python3 nav2_ws/src/navigation2/nav2_regulated_modules/test/velocity_diagnostics_runtime.py
```

先加载 ROS 与工作空间环境。该测试不启动底盘，验证三模式、碰撞监控分支、正负及零速度、非法／缺失／过期数据、双输出、采样频率、目录失败与文件滚动。

## 误差与终点距离的低频保存

固定路径控制器 `goal_error_log_frequency: 1.0` 按 ROS 时间约每秒记录一次终点诊断，独立于速度文件的 100 Hz。文件为 `<log_dir>/nav2_controller/controller_server.log`，包括位置误差（到目标点的直线距离）、路径剩余长度、航向误差和终点纵向投影。停车锁存等状态事件另即时记录。仿真入口可用 `fixed_path_goal_error_log_frequency:=1.0` 设置；直接规控入口使用控制器 YAML 中的参数。

已读取实际往返仿真的速度文件及滚动备份、控制器文件，确认两类数据均保存，终端终点误差记录也全部匹配文件；未重跑仿真。统计与样例见项目 `readme/test_reports/2026-10-10-velocity-logging/save_verification.json`。高频按墙钟采样、低频按 ROS 时钟限频，仿真速度会改变低频日志的墙钟间隔。

## S 曲线加速度与加加速度

规控及仿真 YAML 默认启用 `jerk_limited_smoothing: true`；通用速度平滑器默认 false。`max_accel`／`max_decel` 仍为硬限制，`max_accel_jerk`／`max_decel_jerk` 控制加速度变化速度。默认 `[6,0,4]`／`[8,0,6]`，新模式按两组的较小值保护目标突变及反向。新模式要求零死区、`scale_velocities=false`；限制参数需生命周期重新配置，不能激活中直接修改。

普通零指令和输入超时平滑制动，强制 `immediate_stop_on_zero_command=false`；下游碰撞监控急停保持现有处理。速度上限话题降低时先平滑制动，输出不能瞬时跳到新目标。

`<log_dir>/nav2_velocity_smoother/velocity_smoother.log` 在每个实际平滑周期记录 `S-curve axis`、单调时钟 dt、输入、反馈参考、校正目标、实际输出、由输出差分得到的 accel／jerk 及其限制。x/y/theta 轴编号为 0/1/2，线 jerk 单位 m/s³、角 jerk 单位 rad/s³。组件模式日志可能归入该容器首先初始化的共享 logger 文件，可按 `S-curve axis` 检索。原速度链文件 100 Hz 和终点误差 1 Hz 保留。

已完成一次短时消息与一次 0.3 m/s 停车测试，报告在项目 `readme/test_reports/2026-10-10-jerk-smoother`；没有全速度及实车验收。
