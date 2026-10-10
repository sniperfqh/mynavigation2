# 官方 main AMCL 借鉴与本轮边界

核对日期：2026-10-09；查询到的官方 main 引用为 `36161d4aa53f3c012dee606615a9dc92719004d4`。
参考 [官方 AMCL](https://github.com/ros-navigation/navigation2/tree/main/nav2_amcl)、
[参数与初始化](https://github.com/ros-navigation/navigation2/blob/main/nav2_amcl/src/amcl_node.cpp)、
[运动模型](https://github.com/ros-navigation/navigation2/blob/main/nav2_amcl/src/motion_model/differential_motion_model.cpp)。

| 能力 | 当前选择 |
| --- | --- |
| random_seed 固定粒子滤波随机种子 | 回移启动参数，实验明确记录种子；同时防止本项目 Gaussian PDF 初始化覆盖显式种子 |
| 距离场堆键缓存与重复计算避免 | 回移优化，保留 double 堆键精度；302 万单元修复前后快照逐字节一致，单次计算约 0.715→0.434 秒 |
| 小位移的差分运动预测 | 上游仍有相同的 <1 cm 分支，本轮保留已用零噪声测试和倒车仿真验证的纵向符号修复 |
| 概率激光模型与 beamskip | 本项目已具备相关模型；0.2 m/s 原实验配置出现定位偏差后，独立试验 likelihood_field_prob 与 sigma_hit=0.03，beamskip 保持关闭；若采用须重新建立同配置制动 A/B，不能与原基线混算 |
| 参数更新预校验与新 ROS 公共节点接口 | 不整包覆盖；本次新增种子限定启动配置，其他接口迁移不属于停车效率验证 |

`random_seed >= 0` 使用显式种子；负值保持原先未指定种子的行为。参数在节点声明后只读，
避免测试中途改变随机序列。Gaussian PDF 显式种子模式不再自行重新播种；未指定种子仍保留原 PDF
播种方式。本项目 PF 使用进程内 RNG，这些实验采用每进程单个 AMCL，不宣称多 AMCL 同进程随机流隔离。

测试覆盖同种子相同样本、不同种子不同样本、重新创建 Gaussian PDF 不重启显式序列、默认模式仍有效。
正式 A/B 的对应基线／候选使用相同种子；第二批更换种子。固定种子提高可重复性，不代表精度保证。

距离场优化只改变计算与缓存方式，不改变匹配距离定义；它的耗时收益属于初始化，不能算成到点效率改善。

地图几何、里程计方向与完整停车窗口是独立验收项；上游版本更新不替代本轮真实激光／定位验证。

## 概率模型独立验证（2026-10-09）

原实验 likelihood_field、sigma_hit=0.2 在正式 0.2 m/s 基线第 2 次倒车时，定位误差达到 16.63 mm，真值诊断约 5.04 mm；已取消并确认停稳，保留失败。

使用本项目已有 likelihood_field_prob、sigma_hit=0.03、beamskip=false，在相同原场景、真实激光与实验地图下验证 0.2 m/s 前后向各 5 次：10/10 通过。停后连续 1 秒定位 XY 最大误差为 5.026 mm，真值独立诊断最大为 7.209 mm；实际速度达到请求档位，停车后输出零速且无终点旋转。原始数据独立复算与采集结果一致。证据：`/tmp/nav2-speed-stop-20261008/prob_probe_02/results.json`、`raw_audit.json` 及压缩轨迹。

这是定位专项试验，尚非全速度段精度保证或制动效率验收，也不是上游新算法带来的精度证明。生产 AMCL YAML 未改变。新正式 A/B 位于 `/tmp/nav2-speed-stop-20261008/accept_v2/`，从 0.1 m/s 重建同一定位配置的基线与候选；状态以 `progress.json` 和原始数据为准，不能混入原配置结果。
