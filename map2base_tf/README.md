# map2base_tf

## 用途与默认值

可执行文件名为 `map2baseTF`，周期发送固定位置的动态 TF，用于隔离验证。不是定位算法，不读取激光、地图或轮速。

| 参数 | 内置默认值 |
|---|---|
| `parent_frame`／`child_frame` | `map`／`base_link` |
| `x`／`y`／`z` | 9.3／−12.0／0.0 m |
| `yaw` | 3.094010164837 rad |
| `publish_rate` | 30 Hz |
| `tf_time_offset` | 0.0 s；正值将发送时间戳向过去偏移 |

发布频率必须为正，时间偏移不得为负。固定变换使用当前 ROS 时间逐次发送，不代表机器人运动或有效定位。

## 隔离运行

先在 `nav2_ws` 构建并加载环境，再运行：

```bash
ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 ros2 run map2base_tf map2baseTF \
  --ros-args -p parent_frame:=test_map -p child_frame:=test_base
```

当前外部定位入口不自动启动本节点。不要与实际定位向同一子坐标系重复发布 TF。实现见 [src/map2base_tf_node.cpp](src/map2base_tf_node.cpp)，依赖见 [package.xml](package.xml)，构建约定见[仓库说明](../README.md)。
