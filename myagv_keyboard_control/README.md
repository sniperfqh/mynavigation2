# myagv_keyboard_control

`myagv_keyboard_control` 是 MYAGV 底盘终端键盘直控包。节点按照固定周期直接发布
`byd_custom_msgs/msg/ControlRes` 到 `/control_to_uart`，接口与 `controlpub` 的最终输出一致。

## 一、目录结构

```text
myagv_keyboard_control/
├── config/keyboard_control.yaml          # 默认控制参数
├── launch/keyboard_control.launch.py     # 节点启动入口
├── src/myagv_keyboard_control_node.cpp   # 键盘读取与周期发布实现
├── CMakeLists.txt
└── package.xml
```

## 二、控制接口

默认输出：

```text
Topic: /control_to_uart
Type:  byd_custom_msgs/msg/ControlRes
Rate:  50Hz
```

字段定义：

```text
v           底盘线速度，单位 m/s
w           底盘角速度，单位 rad/s
v_lift      固定为 0.0
w_rotation  固定为 0.0
```

按键映射：

| 按键 | 功能 | v | w |
| --- | --- | ---: | ---: |
| `W` 或 `↑` | 前进 | `+linear_speed` | `0.0` |
| `S` 或 `↓` | 后退 | `-linear_speed` | `0.0` |
| `A` 或 `←` | 原地左转 | `0.0` | `+angular_speed` |
| `D` 或 `→` | 原地右转 | `0.0` | `-angular_speed` |
| `Space` 或 `X` | 停车 | `0.0` | `0.0` |
| `Q` | 发布停车指令并退出 | `0.0` | `0.0` |

方向按键超过 `command_timeout` 没有再次输入时，节点自动恢复为周期发布停车指令。

## 三、运行方法

节点必须在可交互终端中运行：

```bash
source /opt/ros/humble/setup.bash
source /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws/install/setup.bash
ros2 launch myagv_keyboard_control keyboard_control.launch.py
```

也可以直接运行并覆盖参数：

```bash
ros2 run myagv_keyboard_control myagv_keyboard_control_node --ros-args \
  -p linear_speed:=0.1 \
  -p angular_speed:=0.3 \
  -p command_timeout:=0.8
```

## 四、参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `output_topic` | `/control_to_uart` | 底盘控制输出 Topic |
| `publish_rate` | `50.0` | 周期发布频率，单位 Hz |
| `linear_speed` | `0.2` | 前进和后退速度绝对值，单位 m/s |
| `angular_speed` | `0.5` | 左右转角速度绝对值，单位 rad/s |
| `command_timeout` | `0.5` | 最后一次方向键输入后的停车超时，单位 s；设为 `0.0` 表示关闭超时 |

## 五、安全约束

不要同时运行 `controlpub` 和本节点。两者都会发布 `/control_to_uart`，同时运行会造成底盘控制指令竞争。

首次联调应架空驱动轮或断开动力执行机构，先通过以下命令确认字段、符号和频率：

```bash
ros2 topic echo /control_to_uart
ros2 topic hz /control_to_uart
ros2 topic info /control_to_uart --verbose
```
