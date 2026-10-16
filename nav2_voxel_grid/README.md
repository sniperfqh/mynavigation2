# Nav2 Voxel Grid

## 当前项目用法

提供按位压缩的三维体素标记、清除与射线追踪结构，服务于 VoxelLayer。

包类型：体素算法库。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`nav2_common`、`rclcpp`。

使用方式：由依赖方通过构建依赖、头文件或 Python 导入使用，不提供通用的独立导航启动命令。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The `nav2_voxel_grid` package contains the VoxelGrid used by the `Voxel Layer` inside of `nav2_costmap_2d`. The voxel grid itself is simply a 2D char pointer array of the map size with bit locations corresponding to voxel values (free, unknown, occupied , etc).

It is branched out as a separate package for use in other applications where a dense voxel grid representation may be useful. It also contains implementations of 3D raycasting.

## ROS1 Comparison

This package is a direct port to ROS2 for use in the voxel layer.

## 中文翻译

### Nav2 Voxel Grid

nav2_voxel_grid 包实现 nav2_costmap_2d 中 Voxel Layer 使用的 VoxelGrid。网格本质上是与地图尺寸对应的二维字符指针数组，每个位表示体素状态，例如空闲、未知和占用；包中还提供三维射线投射实现。

该包从 Costmap 中独立出来，便于其他需要稠密体素表示的应用复用，是 ROS 1 版本面向 ROS 2 的直接移植。
