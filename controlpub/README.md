# controlpub

## 职责与接口

将 `geometry_msgs/msg/Twist` 的 `linear.x/angular.z` 转为 `byd_custom_msgs/msg/ControlRes` 的 `v/w`；`v_lift`、`w_rotation` 固定为 0。这里只发布 ROS 消息，实际串口发送由外部通信程序负责。

| 参数 | 节点内置默认值 | 说明 |
|---|---|---|
| `input_topic` | `/cmd_vel` | Reliable／Volatile、KeepLast(10) 输入 |
| `output_topic` | `/control_to_uart` | Reliable／Volatile、KeepLast(10) 输出 |
| `legacy_velocity_logging` | `true` | 旧版运动摘要；统一规控入口可关闭 |

只有收到首条有限的有效速度后才创建发布者；之后每条有效输入转发一次，没有定时零速心跳或输入断流看门狗。空闲时话题可能尚无发布者，或保留发布者但不发消息。非有限速度被忽略，不应把话题静默自行解释为已停稳。

## 构建与运行

从项目根目录执行：

```bash
cd nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to controlpub
source install/local_setup.bash
ros2 run controlpub controlpub_node
```

规控和相应仿真入口已管理本节点时，不重复启动。源码见 [controlpub_node.cpp](src/controlpub_node.cpp)，构建与依赖见 [CMakeLists.txt](CMakeLists.txt)、[package.xml](package.xml)。速度链与单一输出来源说明见[规控包](../nav2_regulated_modules/README.md)。
