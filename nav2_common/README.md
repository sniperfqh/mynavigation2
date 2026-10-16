# nav2_common

## 当前项目用法

提供 Nav2 通用 CMake 宏与 Python Launch 参数重写工具，包括 RewrittenYaml。

包类型：构建与启动工具库。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[cmake/](cmake/) 保存构建宏，[nav2_common/](nav2_common/) 保存 Python 包与 Launch 工具。

直接依赖（清单节选，完整约束见 manifest）：`ament_cmake_python`、`launch`、`launch_ros`、`osrf_pycommon`、`python3-yaml`、`rclpy`。

使用方式：由依赖方通过构建依赖、头文件或 Python 导入使用，不提供通用的独立导航启动命令。

例如 Launch 中使用 `from nav2_common.launch import RewrittenYaml`，按启动实参生成临时参数配置；源码见 [nav2_common/launch/](nav2_common/launch/)。这不会自动修改源 YAML，也不代表运行中节点参数已热更新。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
