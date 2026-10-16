# costmap_queue

## 当前项目用法

提供代价地图栅格按距离展开的队列工具，服务于 DWB 距离场计算。

包类型：算法库。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`nav2_common`、`nav2_costmap_2d`、`rclcpp`。

使用方式：由依赖方通过构建依赖、头文件或 Python 导入使用，不提供通用的独立导航启动命令。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
