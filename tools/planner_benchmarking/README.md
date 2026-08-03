# Planning Benchmark

This experiment runs a set of planners over randomly generated maps, with randomly generated goals for objective benchmarking.

To use, modify the Nav2 bringup parameters to include the planners of interest:

```
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["SmacHybrid", "Smac2d", "SmacLattice", "Navfn", "ThetaStar"]
    SmacHybrid:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
    Smac2d:
      plugin: "nav2_smac_planner/SmacPlanner2D"
    SmacLattice:
      plugin: "nav2_smac_planner/SmacPlannerLattice"
    Navfn:
      plugin: "nav2_navfn_planner/NavfnPlanner"
    ThetaStar:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner"
```

Set global costmap settings to those desired for benchmarking. The global map will be automatically set in the script. Inside of `metrics.py`, you can modify the map or set of planners to use.

Launch the benchmark via `ros2 launch ./planning_benchmark_bringup.py` to launch the planner and map servers, then run each script in this directory:

- `metrics.py` to capture data in `.pickle` files.
- `process_data.py` to take the metric files and process them into key results (and plots)

## 中文翻译

# 规划器基准测试

该实验在随机生成的地图和随机目标上运行多种全局规划器，用于客观比较规划性能。需要在 Nav2 Bringup 参数中配置待比较的插件，例如 Navfn、ThetaStar、Smac 2D、Hybrid 和 Lattice，并按原文示例设置插件类型。

全局 Costmap 参数应设置为基准测试所需值；脚本会自动设置全局地图。metrics.py 负责采集指标并保存为 .pickle，process_data.py 负责处理指标并生成关键结果和图表。先运行 planning_benchmark_bringup.py 启动规划器和地图服务器，再执行目录中的脚本。
