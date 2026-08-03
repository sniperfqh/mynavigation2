# Nav2 Core

This package hosts the abstract interface (virtual base classes) for plugins to be used with the following:
- global planner (e.g., `nav2_navfn_planner`)
- controller (e.g., path execution controller, e.g `nav2_dwb_controller`)
- smoother (e.g., `nav2_ceres_costaware_smoother`)
- goal checker (e.g. `simple_goal_checker`)
- behaviors (e.g. `drive_on_heading`)
- progress checker (e.g. `simple_progress_checker`)
- waypoint task executor (e.g. `take_pictures`)
- exceptions in planning and control

The purposes of these plugin interfaces are to create a separation of concern from the system software engineers and the researcher / algorithm designers. Each plugin type is hosted in a "task server" (e.g. planner, recovery, control servers) which handles requests and multiple algorithm plugin instances. The plugins are used to compute a value back to the server without having to worry about ROS 2 actions, topics, or other software utilities. A plugin designer can simply use the tools provided in the API to do their work, or create new ones if they like internally to gain additional information or capabilities.

## 中文翻译

# Nav2 Core

该包定义 Nav2 插件使用的抽象接口（虚基类），覆盖全局规划器、局部控制器、路径平滑器、Goal Checker、行为插件、Progress Checker、航点任务执行器以及规划和控制异常。

这些接口把系统软件工程师负责的任务服务器与算法研究人员负责的插件实现分离。每类插件由 Planner、Recovery 或 Controller 等 Task Server 承载，服务器处理请求并管理多个算法实例；插件只需根据 API 输入计算路径、速度、平滑结果或检查结果，不需要直接处理 ROS 2 Action、Topic 及其他宿主工具。
