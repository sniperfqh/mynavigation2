# Nav2 Voxel Grid

The `nav2_voxel_grid` package contains the VoxelGrid used by the `Voxel Layer` inside of `nav2_costmap_2d`. The voxel grid itself is simply a 2D char pointer array of the map size with bit locations corresponding to voxel values (free, unknown, occupied , etc). 

It is branched out as a separate package for use in other applications where a dense voxel grid representation may be useful. It also contains implementations of 3D raycasting. 

## ROS1 Comparison

This package is a direct port to ROS2 for use in the voxel layer. 

## 中文翻译

# Nav2 Voxel Grid

nav2_voxel_grid 包实现 nav2_costmap_2d 中 Voxel Layer 使用的 VoxelGrid。网格本质上是与地图尺寸对应的二维字符指针数组，每个位表示体素状态，例如空闲、未知和占用；包中还提供三维射线投射实现。

该包从 Costmap 中独立出来，便于其他需要稠密体素表示的应用复用，是 ROS 1 版本面向 ROS 2 的直接移植。
