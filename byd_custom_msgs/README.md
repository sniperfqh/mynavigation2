# byd_custom_msgs

## 接口目录

本包通过 rosidl 生成 C++／Python 接口，没有独立节点。构建来源见 [CMakeLists.txt](CMakeLists.txt) 与 [package.xml](package.xml)。

| 类型 | 定义 | 用途 |
|---|---|---|
| `msg/ControlRes` | [ControlRes.msg](msg/ControlRes.msg) | `v/w` 为底盘线／角速度；另含 `v_lift/w_rotation` |
| `msg/MotionState` | [MotionState.msg](msg/MotionState.msg) | 带 Header 的 `v_car/w_car`、升降与货架反馈 |
| `msg/ChassisControl` | [ChassisControl.msg](msg/ChassisControl.msg) | Header、方向 `op`、线／角速度与加速度请求 |
| `msg/NaviSegment` | [NaviSegment.msg](msg/NaviSegment.msg) | 直线／三次贝塞尔路径段、端点、控制点、限速及方向 |
| `action/NavigationService` | [NavigationService.action](action/NavigationService.action) | Goal 为 `task_id`＋`NaviSegment[]`；结果 `finish`；反馈当前任务、段编号和 progress |
| `action/FollowFixedPath` | [FollowFixedPath.action](action/FollowFixedPath.action) | Path 请求、success 结果与 distance_remaining 反馈；定义存在不代表默认启动同名服务器 |

## 当前使用契约

- `/navigation_service` 由 fixed_path 模式的 `regulated_navigator` 提供。直线 `segment_type=1`，贝塞尔 `segment_type=2`；常量拼写保留接口原样 `SEGMENT_TYPR_LINE/SEGMENT_TYPR_BEZIER`。
- `motion_direction=1` 前进、`2` 倒车；同一 Action 所有段须同方向。`max_speed` 为有限正数，倒车通过方向字段表达；整条任务取各段最小速度，并受 `fixed_path_max_speed` 限制。
- 相邻段端点须连续，路径坐标使用导航全局坐标系。`max_load_speed` 和 `dwell_time` 字段被接口保留，当前 fixed_path 路径生成／执行分支未据此实现载重速度切换或分段驻留。
- `ChassisControl.op`：0 前进、1 后退、2 左转、3 右转。一次请求表达直行或原地旋转；实际限幅、超时及输出由消费者负责。
- `/control_to_uart` 是 ROS `ControlRes` 话题，名称不代表本包包含串口驱动；消息定义不规定硬件已执行命令。

从项目根目录进入 `nav2_ws` 构建并加载环境后，可只读检查接口：

```bash
ros2 interface show byd_custom_msgs/action/NavigationService
ros2 interface show byd_custom_msgs/msg/NaviSegment
```

调用方法见[规控包](../nav2_regulated_modules/README.md)，构建方法见[仓库说明](../README.md)。
