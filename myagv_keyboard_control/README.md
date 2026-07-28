<!-- 中文说明：本文档记录键盘遥控包的接口、平滑控制逻辑、参数、运行方式和安全约束。 -->
# myagv_keyboard_control

`myagv_keyboard_control` 是 MYAGV 底盘终端键盘直控包。节点按照固定周期直接发布
`byd_custom_msgs/msg/ControlRes` 到 `/control_to_uart`，接口与 `controlpub` 的最终输出一致。
按键只改变目标速度，节点按照线速度和角速度加减速限制生成连续的周期输出。

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
| `W` 或 `↑` | 平滑加速前进 | `→ +linear_speed` | `0.0` |
| `S` 或 `↓` | 平滑加速后退 | `→ -linear_speed` | `0.0` |
| `A` 或 `←` | 平滑加速原地左转 | `0.0` | `→ +angular_speed` |
| `D` 或 `→` | 平滑加速原地右转 | `0.0` | `→ -angular_speed` |
| `Space` 或 `X` | 按减速度限制平滑停车 | `→ 0.0` | `→ 0.0` |
| `Q` | 立即发布停车指令并退出 | `0.0` | `0.0` |

前进／后退反向、左转／右转反向以及直行／原地转向切换都会先减速到零，再向新方向平滑加速，
不会跨过零点跳变，也不会在直行与原地转向切换期间同时输出明显的线速度和角速度。

终端不能直接报告按键松开事件。方向按键超过 `command_timeout` 没有再次输入时，节点将其视为
按键已经松开：目标速度切换为零，当前速度继续按减速度限制平滑下降，直至周期发布停车指令。
`Q` 和节点安全收口仍立即清零，不经过平滑减速。

## 三、运行方法

节点必须在可交互终端中运行。Launch 会把启动 Shell 的实际 `/dev/pts/*` 设备传给节点，
因此既支持直接运行，也支持从 `nav2_regulated_modules` 的 `remote` 分支启动：

```bash
source /opt/ros/humble/setup.bash
source /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws/install/setup.bash
ros2 launch myagv_keyboard_control keyboard_control.launch.py
```

也可以从规控主流程进入遥控模式：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py operation_mode:=remote
```

也可以直接运行并覆盖参数：

```bash
ros2 run myagv_keyboard_control myagv_keyboard_control_node --ros-args \
  -p linear_speed:=0.1 \
  -p angular_speed:=0.3 \
  -p linear_accel_limit:=0.3 \
  -p linear_decel_limit:=0.6 \
  -p command_timeout:=0.8
```

## 四、参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `input_device` | `/dev/tty` | 键盘输入终端；Launch 会自动覆盖为启动 Shell 的 `/dev/pts/*` |
| `output_topic` | `/control_to_uart` | 底盘控制输出 Topic |
| `publish_rate` | `50.0` | 周期发布频率，单位 Hz |
| `linear_speed` | `0.2` | 前进和后退速度绝对值，单位 m/s |
| `angular_speed` | `0.5` | 左右转角速度绝对值，单位 rad/s |
| `linear_accel_limit` | `0.4` | 线速度加速限制，单位 m/s²，必须大于零 |
| `linear_decel_limit` | `0.8` | 线速度减速限制绝对值，单位 m/s²，必须大于零 |
| `angular_accel_limit` | `1.0` | 角速度加速限制，单位 rad/s²，必须大于零 |
| `angular_decel_limit` | `2.0` | 角速度减速限制绝对值，单位 rad/s²，必须大于零 |
| `command_timeout` | `0.5` | 最后一次方向键输入后的松键判定超时，超时后平滑减速，单位 s；设为 `0.0` 表示关闭超时 |

默认 `50 Hz` 下，线速度从零提升到 `0.2 m/s` 约需 `0.5 s`，平滑停车约需 `0.25 s`；
角速度从零提升到 `0.5 rad/s` 约需 `0.5 s`，平滑停车约需 `0.25 s`。

## 五、安全约束

不要同时运行 `controlpub` 和本节点。两者都会发布 `/control_to_uart`，同时运行会造成底盘控制指令竞争。

`Space`／`X` 和键盘输入超时用于正常平滑停车；需要立即停车并退出时使用 `Q`。节点安全收口仍会
绕过速度斜坡直接发布零速度。

首次联调应架空驱动轮或断开动力执行机构，先通过以下命令确认字段、符号和频率：

```bash
ros2 topic echo /control_to_uart
ros2 topic hz /control_to_uart
ros2 topic info /control_to_uart --verbose
```
