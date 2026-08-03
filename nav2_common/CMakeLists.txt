cmake_minimum_required(VERSION 3.5)

project(nav2_common NONE)
# 中文：本包本身不编译 C++ 目标，工程类型使用 NONE；它主要安装 Python Launch 工具和公共 CMake 宏。

find_package(ament_cmake_core REQUIRED)
# 中文：提供 ament_package() 及配置导出能力，负责把本包注册到 ROS 2 的 ament 索引。
find_package(ament_cmake_python REQUIRED)
# 中文：提供 ament_python_install_package()，把 nav2_common Python 模块安装到运行时搜索路径。

ament_python_install_package(nav2_common)
# 中文：安装 nav2_common/ 目录下的 Python 包；下游 Launch 文件通过该安装包导入四个 Substitution。

ament_package(
  CONFIG_EXTRAS "nav2_common-extras.cmake"
)
# 中文：导出额外 CMake 配置，使下游 find_package(nav2_common) 后能够使用 nav2_package() 宏。

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
# 中文：把公开的 cmake/ 目录安装到 share/nav2_common，供下游构建系统读取 nav2_package.cmake。
