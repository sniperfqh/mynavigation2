# Nav2 Util

The `nav2_util` package contains utilities abstracted from individual packages which may find use in other uses. Some examples of things you'll find here:

- Geometry utilities for computing distances and values in paths
- A Nav2 specific lifecycle node wrapper for boilerplate code and useful common utilities like `declare_parameter_if_not_declared()`
- Simplified service clients
- Simplified action servers
- Transformation and robot pose helpers

The long-term aim is for these utilities to find more permanent homes in other packages (within and outside of Nav2) or migrate to the raw tools made available in ROS 2.

## 中文翻译

# Nav2 Util

nav2_util 包提供从各个 Nav2 包中抽取的通用工具，包括路径距离和几何计算、Lifecycle 节点封装、declare_parameter_if_not_declared、简化 Service Client、简化 Action Server，以及 TF 和机器人 Pose 辅助函数。

长期目标是把这些工具迁移到更合适的包（包括 Nav2 之外的包），或迁移到 ROS 2 原生提供的底层工具中。
