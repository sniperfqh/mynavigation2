# laserpub

## 用途

合成 LaserScan 测试节点，可配置视场、量程、采样数及左右障碍束。不连接硬件，也不提供真实环境检测；当前规控生产入口不自动启动它。

| 参数 | 内置默认值 |
|---|---|
| `topic`／`frame_id` | `/c200_lidar_node/scan`／`base_link` |
| `publish_rate`／`sample_count` | 10 Hz／360 |
| `angle_min`／`angle_max` | −π／π rad |
| `range_min`／`range_max` | 0.12／3.5 m |
| `default_range` | `range_max` |
| `publish_test_obstacles` | `true` |
| `obstacle_range`／`obstacle_width` | 1.5 m／中心左右各 6 个采样点 |

发布使用 SensorDataQoS。测试障碍位于 ±π/2 方向；默认距离填满其余采样。发布频率必须为正、采样数至少 2、角度及量程上下界必须有效。

## 隔离运行

先在 `nav2_ws` 构建本包并加载环境，然后使用独立测试域：

```bash
ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 ros2 run laserpub laserpub \
  --ros-args -p topic:=/test_scan -p frame_id:=test_base
```

实现与依赖见 [src/laserpub_node.cpp](src/laserpub_node.cpp)、[package.xml](package.xml)。环境加载方法见[仓库说明](../README.md)。
