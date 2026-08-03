# Nav2 Controller

The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available controller plugins. 

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

## 中文翻译

# Nav2 Controller

Nav2 Controller 是 Nav2 中实现 nav2_msgs::action::FollowPath Action Server 的任务服务器。它接收 Planner 生成的路径，调用配置的控制器插件计算机器人速度，并负责控制周期、里程计、进度检查、目标检查和最终速度发布。

插件必须实现 nav2_core 中 controller 头文件定义的虚基类。一个 Controller Server 可以加载多个路径执行插件，并通过 Controller ID 选择。Progress Checker 和 Goal Checker 也以插件形式提供，使卡住判定和到达判定与具体控制器解耦。
