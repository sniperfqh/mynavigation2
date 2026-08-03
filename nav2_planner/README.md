# Nav2 Planner

The Nav2 planner is a Task Server in Nav2 that implements the `nav2_behavior_tree::ComputePathToPose` interface.

A planning module implementing the `nav2_behavior_tree::ComputePathToPose` interface is responsible for generating a feasible path given start and end robot poses. It loads a map of potential planner plugins to do the path generation in different user-defined situations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-planner-server.html) for additional parameter descriptions and a [tutorial about writing planner plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html).

## 中文翻译

# Nav2 Planner

Nav2 Planner 是全局规划任务服务器，实现路径规划 Action 接口。它负责接收目标 Pose 或 Pose 序列、获取机器人起点、处理 TF 和 Costmap、选择 Global Planner 插件、校验返回路径并发布规划结果。

规划器实现 nav2_core::GlobalPlanner 的 createPlan() 接口。一个 Planner Server 可以同时加载多个规划器，通过插件 ID 选择实际算法。参数说明和新建 Planner 插件教程见原文链接。
