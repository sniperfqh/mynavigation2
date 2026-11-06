# 固定路径 TF 诊断

`ros2 launch nav2_regulated_modules regulated_modules.launch.py operation_mode:=fixed_path` 在下次使用新构建启动后，默认启用主程序内的 TF 诊断：map→base_link 每 10 ms 采样打印一次，直接边每收到一条就打印一次，控制器实际消费记录默认 100 Hz。无需额外启动诊断节点，不受速度日志开关或 ChassisControl 首帧门控影响；原有 TF 查询、超时、起停和转向算法保持。

## 现有日志为什么不足以定位来源

`controller_server.log` 原有的 `pose_age` 是终点复核时观察到的合成位姿时间戳差，Costmap 也会打印位姿、TF 失败或消息过滤失败。这些记录没有逐边的源时间戳、接收时刻和发布者。`regulated_navigator::lookupCurrentPose()` 的 fixed_path 分支只以查询成功刷新本机成功时间，不把源时间戳年龄作为新的控制门槛。

因此，合成 `map→base_link` 落后可能来自直接发布该边，也可能来自组合链上的某条边、上游队列、发送端时间戳／时钟或接收端调度；仅凭 `pose_age` 无法拆分。`transform_tolerance` 是查询等待容差，不能当作已经完成了定位延迟补偿。

## 文件与字段

日志目录由既有 `log_dir` 决定，默认 `/tmp/nav2_logs`。节点配置后，即使没有运动任务也会以 100 Hz 打印 `TF_MAP_BASE`；插件仅在实际控制调用期间记录。高频行用 DEBUG 写文件，原 Launch 默认文件级别 trace 可见；全边和时钟摘要仍为 1 Hz INFO。

| 文件／标记 | 内容 |
|---|---|
| `nav2_regulated_modules/regulated_navigator.log`：`TF_MAP_BASE` | 100 Hz 最新 map→base_link Buffer 采样，支持直接或组合链；包含序号、源时间戳、三种时钟、年龄、采样间隔、位置、四元数、航向及 `source_changed` |
| 同文件：`TF_RX_MAP_BASE` | 直接 map→base_link 动态边逐条接收记录，包含 GID、接收序号、源／接收时间及完整位姿；频率由实际发布源决定 |
| `nav2_regulated_modules/regulated_navigator.log`：`TF_CLOCK` | 同一进程 ROS／系统／单调时钟的纳秒值、各自步长、`use_sim_time`、ROS 与系统时间差 |
| 同文件：`TF_EDGE` | 实际收到的每条父子边，区分动态／静态、发布者节点与 GID、源时间戳、回调接收的三种时钟、接收频率／最大间隔／静默时长、年龄范围、重复／倒退／零时间戳计数、位置和航向 |
| 同文件：`TF_CHAIN` | 导航器自身 TF Buffer 的最新 `global_frame→robot_base_frame` 合成结果或查询异常；查询零等待 |
| `nav2_controller/controller_server.log`：`TF_CONTROL` | 固定路径插件真正收到并使用的位姿，默认随 100 Hz 控制调用打印；包含输入／使用坐标系及时间戳、年龄窗口最小／最大值、控制采样最大间隔、三种时钟、带符号的实测线／角速度 |

`TF_MAP_BASE` 的 100 Hz 是采样打印频率，不是对源更新率的承诺；`source_changed=false` 表示再次读到同一时间戳。源只发布 30 Hz 时，直接接收行也只能约 30 Hz，不能伪造为 100 个新源帧；组合链没有直接 map→base_link 边时，使用 `TF_MAP_BASE` 配合 `TF_EDGE` 分析。

`TF_EDGE` 的 `arrival_age_s` 是回调接收时 ROS 时间减源时间戳，`age_now_s` 是摘要时年龄。`silence_s` 与 `rx_gap_max_s` 使用单调时钟，避免系统时间校准混淆接收间隔。`window_samples`、`rx_hz` 和年龄范围是本窗口统计；`repeats_total`、`backwards_total`、`zero_total` 为生命周期配置以来累计值。插件的重复时间戳计数是控制周期重复使用同帧，不等于发布者重复发布。

静态 TF 也记录时间戳，**不能按动态 TF 的年龄门槛判为过期**。未收到新样本的窗口 `window_samples=0`，年龄范围回显最后一次到达年龄，需结合静默时长判断。最多保存 256 组“话题＋边＋GID”，超出时 `TF_RX_STATUS` 打印未保存变换数。

GID 与 ROS 图中发布端精确匹配后才填写节点名；图尚未发现或发布者已退出时打印 `unknown`，保留 GID，不能把话题上第一个发布者当作整棵 TF 树的来源。相同子帧出现不同 GID／父帧时可以发现潜在重复发布或父帧竞争。

中间件支持时还记录 `dds_source_ns`、`dds_received_ns`、`dds_delta_s` 和 `callback_after_dds_s`。`dds_valid=false` 表示发送／接收时间元数据不完整，不能用零值推断网络零延迟。跨机 DDS 时间差同样依赖时钟同步，不能直接当作网络延迟。

## 参数

下列为 ROS 节点参数，可写入自定义参数 YAML；不是新增的 Launch 实参。修改后重新配置／启动生效。

| 节点参数 | 默认值 | 作用 |
|---|---|---|
| `regulated_navigator.tf_diagnostics_enabled` | `true` | 仅 fixed_path 创建 TF 接收诊断与摘要定时器 |
| `regulated_navigator.tf_diagnostics_log_frequency` | `1.0` | 主程序摘要频率，范围大于 0 且不超过 10 Hz |
| `regulated_navigator.tf_diagnostics_map_base_frequency` | `100.0` | map→base_link 采样打印频率，范围大于 0 且不超过 100 Hz |
| `controller_server.FixedPathController.tf_diagnostics_enabled` | `true` | 固定路径插件的实际消费位姿日志；其他插件名默认关闭 |
| `controller_server.FixedPathController.tf_diagnostics_log_frequency` | `100.0` | 插件消费打印频率，上限 100 Hz；实际不超过控制调用频率 |

map→base_link 采样、直接边接收和控制消费行高频打印，其余全边统计仍按摘要周期输出。多传感器 TF 树日志量较大时可降低频率，或分别关闭两个开关。TF 订阅遵守现有 `tf`／`tf_static` 命名空间及重映射。

## 下次实车日志如何判断

1. 从 `TF_EDGE` 的父子关系重建实际链，确认是直接 `map→base_link`，还是 `map→odom→base_link` 等组合，不预设一定存在 `odom`。
2. 某条动态边到达时已经明显落后，问题范围在该边上游或其时间戳／跨机时钟；结合精确发布者继续查输入源和发送日志。
3. 单调接收间隔／静默时长变大，说明观察端接收有间断；它可能是源停止发布、传输丢包或本机回调阻塞，不能只凭观察频率判定发布频率。
4. 原始边到达及时，但 `TF_CHAIN`／`TF_CONTROL` 落后，应检查组合链限制、两个进程各自 Buffer、订阅队列和消费调度。导航器观察结果不等于控制器 Buffer 内容。
5. `TF_CLOCK` 的 ROS／系统步长与单调步长发生分歧，检查本机时钟跳变、ROS 仿真时间及 `/clock`。稳定的 ROS／系统时间差不能证明不同机器已经同步。
6. 完整定因仍可能需要发布端的原始测量时间、计算结束／发送时刻，以及两端时钟同步证据。本次增加的观察日志能定位可疑边和接收／消费阶段，不能替代远端内部日志。

2026-11-06 已完成单包 Release 构建，并在独立 ROS Domain 201、localhost 范围启动真实 fixed_path Launch，输入为静止的 100 Hz map→base_link 测试 TF。生命周期日志显示受管节点激活；20 秒日志窗口内，`TF_MAP_BASE` 和 `TF_RX_MAP_BASE` 各 2,001 条，实测约 100.03 Hz，间隔中位数约 10 ms，发布者 GID 匹配 `/map2baseTF`。原始日志与频率统计保存在 `/tmp/nav2-tf100hz-20261106/`。

本轮未发送运动任务，未运行实车定位／底盘反馈，也未触发需要 FollowPath 调用的 `TF_CONTROL` 运行验证。外部隔离服务探针超时，节点激活结论取自启动与生命周期日志，不记为服务查询通过。该验证证明高频 TF 文件打印生效，不证明实车定位延迟或跟踪误差已经改善。
