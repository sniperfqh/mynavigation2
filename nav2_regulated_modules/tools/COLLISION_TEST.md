# 实车／仿真周期 collision 状态与 1.5 m/s 直线 Action 示例

连接已有实车或仿真，不启动或复位机器人，不发布合成 TF／轮速，不直接发布底盘速度。
默认 `--runtime sim`，仿真默认 A=(9.16,-4.37)、B=(13.2,-0.47)；独立周期模式支持去程前进、返程倒车。
该坐标来自旧实验，使用前须确认它与当前地图、可通行区域相符，并手动将机器人放在 A 点 0.2 米内。
`--runtime real` 必须显式提供现场确认的 A／B 端点；缺少端点会在创建 ROS 节点和结果目录前拒绝。
当前 `myworld_bringup` 是外部定位入口，定位须另行启动；仅有 robot_state_publisher 不足以运行。

## 启用与运行

在项目的 `nav2_ws` 中构建并加载本工作空间（不要混入 learning 工作空间）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select nav2_regulated_modules myworld_bringup
source install/local_setup.bash
```

固定路径仿真入口默认启用 collision 门控；`enable_collision_state:=true` 可显式启用，`false` 可关闭。保留现有地图、定位、时间设置等实参。
例如已有外部定位的环境可使用：

```bash
ros2 launch myworld_bringup entry.launch.py operation_mode:=fixed_path \
  enable_collision_state:=true headless:=true use_rviz:=true \
  use_collision_monitor:=false
```

此处关闭独立 Collision Monitor 以单独观察注入状态的作用；它与 collision 状态门控是两个功能。
首次尚未收到 collision 消息时按状态 4 运行；收到过消息后断流达到 0.5 秒仍平滑停车。
显式发布 0／1 的停车含义不变。默认 4 只解除 collision 限制，不产生任务或速度指令。
不要在现有仿真仍运行时重复启动。已有进程不会自动加载新的默认值；若门控未启用，工具拒绝发送任务，需由用户重新启动加载。

另一个已加载相同环境和 ROS_DOMAIN_ID 的终端运行：

```bash
ros2 run nav2_regulated_modules collision_action_test.py \
  --schedule periodic --return-direction backward \
  --state-durations 4 3 3 3 3 \
  --output-dir "$PWD/collision-runs/$(date +%Y%m%d-%H%M%S)"
```

`Ctrl+C` 结束；`--max-legs 4` 可运行一轮；`--a X Y --b X Y` 覆盖端点，
`--odom-topic /odometry`、`--base-frame base_link` 可匹配实际反馈接口。
输出目录必须尚不存在。工具请求速度固定为 1.5 m/s，不调整现有控制上限。
开始前要求新鲜轮速连续停稳 1 秒。底盘发布者在空闲时可能存在但不发消息，不能仅因其存在就要求周期零指令；已收到的非零或非法指令仍拒绝起步。运动阶段的输出缺帧检查保持生效。

### 实车入口与反馈

实车使用已有导航、定位和底盘节点，要求三节点 collision 门控配置一致、控制器和平滑器 active、现有速度上限不低于 1.5 m/s，并拒绝 `use_sim_time=true`。已有导航任务或其他 `/collision_state` 发布者存在时也会拒绝发送任务。

先在当前终端设置已确认安全路线的 `A_X`、`A_Y`、`B_X`、`B_Y`，在已加载环境的 `nav2_ws` 执行：

```bash
ros2 run nav2_regulated_modules collision_action_test.py \
  --runtime real \
  --a "${A_X:?请设置起点X}" "${A_Y:?请设置起点Y}" \
  --b "${B_X:?请设置终点X}" "${B_Y:?请设置终点Y}" \
  --output-dir "$PWD/collision-runs/real-$(date +%Y%m%d-%H%M%S)"
```

| 参数 | `--runtime sim` 默认 | `--runtime real` 默认 |
|---|---|---|
| 反馈话题／类型 | `/odometry`／`nav_msgs/msg/Odometry` | `/motion_state`／`byd_custom_msgs/msg/MotionState` |
| 调度 | `cruise` | `periodic` |
| 返程方向 | `forward` | `backward` |
| 程数 | 0，持续运行 | 4 |
| 周期状态保持秒数：4／3／2／1／0 | 4／3／3／3／3 | 12／4／4／3／3 |

两种场景均可用 `--feedback-topic` 覆盖话题，消息类型由 `--runtime` 决定。实车读取 `v_car/w_car`，保留倒车负号；`--odom-topic` 仅作为仿真兼容参数，不能与 `--feedback-topic` 同时提供。实车不接受 `--odom-topic`。

可显式覆盖调度、返程方向、程数和周期时长；`--max-legs 0` 表示持续运行。路线过短、任务提前完成或观察窗不足时，未覆盖的状态与恢复项为 UNAVAILABLE，不能仅凭默认四程推断完整碰撞验收通过。

## 状态、判定与输出

### 独立周期模式（推荐）

`--schedule periodic` 使用单调时钟，固定 **50 Hz（20 ms）** 发布 `/collision_state`（std_msgs/msg/UInt8）。
首个 Action 接受后启动 `4→3→2→1→0` 循环；仿真默认分别保持 4／3／3／3／3 秒，16 秒一轮；实车默认 12／4／4／3／3 秒，26 秒一轮。
状态切换不等待达到 1.5 m/s；换程、到点和等待停稳不重置状态周期。退出冻结周期后仍以 50 Hz 发布停止。
`--state-durations S4 S3 S2 S1 S0` 可调整五个状态的保持时间，值须为有限正数。

去程 `motion_direction=1`，`--return-direction backward` 时返程为 2，max_speed 始终为正数 1.5。
倒车 UART 线速度为负数，对应限速平台为 −0.975／−0.525 m/s；不需要原地转向再返程。
仿真默认不传 `--return-direction` 仍为两程前进，以兼容已有命令；实车默认返程倒车。

每个“状态区间＋任务”单独记录在 results.json 的 intervals 中，换程后不拼接两程证据。
减速平台容差 ±0.05 m/s、连续 0.3 秒，要求原始请求幅值高于上限 0.05 m/s、方向一致、
离终点至少 0.5 米且输出角速度幅值≤0.05 rad/s。未满足评估条件时为 UNAVAILABLE，不因观察失败停止循环。
停止指令和轮速停稳均要求连续 1 秒；进入状态时已停稳标注“保持停稳”，不重复声称产生新的制动。
状态 4 的速度平台不足只记 UNAVAILABLE，不再阻止后续状态切换。

Action 成功与 collision 响应分开记录：腿的 ACTION_SUCCEEDED 不代表该腿全部 collision 响应通过。
发布时序在 trace.jsonl 的 publish 事件逐条保存，results.json 的 publish_timing 包含实测发布调用频率、
间隔最小／最大／均值及分布；configured_hz=50 不等于实测一定精确为 50。
CSV 另含方向、周期号、Goal UUID 和是否处于活动任务，曲线显示正负速度及对应上限。

四程有限验收使用 `--max-legs 4`；仿真默认持续运行，实车默认四程。两种场景均可显式传 `--max-legs 0` 持续运行，由 Ctrl+C 结束。
定位／轮速失效、输出缺帧、Action 拒绝／中止或超时仍触发停止与自身任务取消。

### 原巡航触发模式（保留兼容）

不传 `--schedule` 或传 `--schedule cruise` 保留原有逻辑，状态发布频率也统一为 50 Hz。
每程先建立正常状态 4 的巡航基准：
`ControlRes.v` 在 1.5±0.05 m/s、角速度在 ±0.05 rad/s 内连续 0.2 秒。
每程依次选择 3、2、1、0，四程一轮，持续重复。

| 状态 | 目标上限 | 观察要求 |
|---|---:|---|
| 3 | 0.975 m/s | ±0.05 m/s 平台连续 0.3 秒后恢复 4，最长等待 3 秒 |
| 2 | 0.525 m/s | ±0.05 m/s 平台连续 0.3 秒后恢复 4，最长等待 3 秒 |
| 1、0 | 0 m/s | 保持状态至少 3 秒；输出线速度≤0.01 m/s、角速度≤0.05 rad/s 连续 1 秒；另验轮速停稳 |

减速平台同时要求新鲜 `/cmd_vel_nav` 仍高于上限 0.05 m/s，且离终点至少 0.5 米，排除明显的到点制动。
状态 4 恢复后，要求观察到原 Action 的输出线速度大于 0.1 m/s 连续 0.2 秒，再等待到点与停稳。
S 曲线期间允许输出逐步变化，不要求瞬间达到上限。

每程结果为 PASS、FAIL 或 UNAVAILABLE；路径过短、巡航不足、提前到点或未观察到恢复均不得通过。
工具遇到未通过、数据异常、Action 拒绝／中止即停止自动发下一程。
`/control_to_uart` 缺失超过 0.25 秒、定位／轮速连续异常 1 秒或单程超过 120 秒时退出。
命令输出与轮速分开记录；不评价定位精度或实车制动距离。

### 证据与退出

两种模式的结果目录均包含 `trace.jsonl`、`results.json`、`velocity.csv`、`velocity.svg` 和 `velocity.html`。
曲线蓝线为底盘输出，橙线为状态对应的目标上限。CSV 保留所有速度样本，图形预览按需降采样。
结果在每程结束和退出时落盘，图形在有底盘样本时生成。

周期模式的 `response_summary` 列出预期方向的五种状态及同任务停车后恢复项；前进／倒车共 12 项。任一项 FAIL 时 `collision_status=FAIL`；全部必需项 PASS 才为 PASS，其余为 UNAVAILABLE。恢复要求同一 Goal、同一程、状态 0／1 后进入状态 4 的有效窗口；不能用换程后的运动替代原任务恢复。

退出时持续发布停止状态，按本工具 Goal UUID 取消任务，等待输出和轮速连续停稳 1 秒。
自身 Action 已确认终止后，允许 UART 在最后一次零指令后保持静默，但必须同时取得新鲜轮速连续停稳 1 秒；
结果用 shutdown_stop_evidence 区分这种证据与“新鲜输出＋新鲜轮速”。任务未确认终止、最后指令非零或轮速过期时不允许此判定。
无法确认接受或停稳时报告 shutdown_error 并返回非零，不宣称停车成功。
不停止已有仿真节点，不取消其他客户端任务。工具结束后状态流停止，现有 0.5 秒超时保护继续生效。

## 开发者隔离验证

`test/test_collision_action_tool.py` 验证连续证据窗、缺帧、终点制动排除及仿真开关路由。
`test/collision_action_protocol_test.py` 要求显式非零 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY=1，
通过 mock Action／遥测服务器验证四程循环、拒绝、Ctrl+C 取消、提前结束和输出缺失；不代表控制器真实响应。
新增周期模式验证未达巡航速度仍切换、跨程不重置、倒车负速度、周期退出冻结，以及空闲／任务结束后的发布者静默。
协议回归另覆盖 MotionState 和自定义反馈话题、反馈断流、重复状态发布者、拒绝、中断及实车错误使用仿真时钟，共 15 个场景；这些场景使用 mock 合成反馈，不代表实车运行或真实制动响应。
现有 `test/collision_state_runtime_test.py` 增加 `--speed 1.5 --distance 30`，可在另一隔离域内
用真实规控节点与合成 TF／轮速回归 1.5／0.975／0.525／0 m/s，底盘输出仍重映射至测试命名空间，
不接入现有仿真或实车。两类隔离验证均不能替代本工具在实际地图短路线上的运行验收。

该隔离回归追加 `--initial-default-check` 可先完全不发送 collision 消息，验证正常运行并持续超过原 0.5 秒超时，
再开启发布状态 0，验证显式停车及后续收到消息后的断流停车；不会将消息默认值当作实际接收记录。

## 当前证据边界

本例子支持的状态不等于当前地图已全部取得有效平台：历史周期仿真取得部分减速／停止证据，其他窗口仍有 UNAVAILABLE。独立状态发布源启停验收的首个任务因起步航向误差 2.928 rad、5 秒基准窗内只转向而结束，已取消并确认停稳；该轮没有启动状态源，不能作为断流停车或同一任务恢复通过的证据。

本文描述当前源码的使用方式；构建、隔离协议回归、真实控制器响应和实车运行的证据分别记录。动态结果由外层项目 ROADMAP 与原始报告维护。返回[工具索引](README.md)或[规控包说明](../README.md)。
