#!/usr/bin/env python3
# 模拟充电电池状态发布器：10Hz 持续发 /agv_battery_state（byd_custom_msgs/BatteryState），
# 供 charging_goal_checker 的充电电流短路完成检测消费。
# 时间戳用节点时钟实时打——这是 ros2 topic pub 做不到的（它构造一次消息后 stamp 不再刷新，
# 会被 checker 的 charging_current_detect_window=0.5s 新鲜度窗口判过期）。
# 时钟源自动探测：有 /clock（Gazebo 模拟时钟）就跟 checker 一样用 sim time，
# 没有则退回系统时钟，保证单独调试时也能发出消息。
#
# 关键语义：充电电流不是一上来就有的——delay 秒内发 charging_current=0（未上桩放电状态），
# delay 秒后才达到设定电流（模拟对接上电）。这是因为 checker 一旦在 0.5s 窗口内看到
# charging_current >= 0.05A 就短路判定充电完成、FollowPath 立即 SUCCEEDED；
# 若从任务一开始就发大电流，车还没动车就"充电成功"了。delay 请按对接耗时估计
# （0.1m/s 跑 1.78m ≈ 20s，可设 25~30s），或手动在对接完成前再启动本脚本。
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from byd_custom_msgs.msg import BatteryState

# 用法: python3 fake_battery_state.py [charging_current_A] [delay_s] [voltage_V]
#       python3 fake_battery_state.py 2.0 30.0 48.0
# 默认 30s 后出现 2.0A 充电电流（阈值 0.05A），电池电压 48V。
CHARGING_CURRENT_A = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
DELAY_S = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
VOLTAGE_V = float(sys.argv[3]) if len(sys.argv) > 3 else 48.0

rclpy.init()

# 1) 探测 /clock 是否在发布（最多等 1.5s）。
probe = Node('fake_battery_clock_probe')
clock_seen = []
probe.create_subscription(Clock, '/clock',
                          lambda msg: clock_seen.append(msg), 10)
for _ in range(15):
    rclpy.spin_once(probe, timeout_sec=0.1)
    if clock_seen:
        break
use_sim_time = bool(clock_seen)
probe.destroy_node()
print(f'[fake_battery_state] use_sim_time={use_sim_time} '
      f'({"检测到 /clock，跟随模拟时钟" if use_sim_time else "未检测到 /clock，使用系统时钟"})')

# 2) use_sim_time 必须在构造时经 parameter_overrides 传入：
#    rclpy 节点自动声明该参数，事后 declare_parameter 会抛
#    ParameterAlreadyDeclaredException。
node = Node('fake_battery_state',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
pub = node.create_publisher(BatteryState, '/agv_battery_state',
                            qos_profile_sensor_data)  # 与 checker 订阅 QoS（SensorData）匹配

start_time = None


def tick():
    global start_time
    now = node.get_clock().now()
    if start_time is None:
        start_time = now  # 首拍记录基准，delay 按节点时钟计（sim/系统时钟都成立）
    docked = (now - start_time).nanoseconds * 1e-9 >= DELAY_S
    msg = BatteryState()
    msg.header.stamp = now.to_msg()  # 关键：每帧实时时间戳
    msg.header.frame_id = 'battery'
    msg.voltage = VOLTAGE_V
    msg.discharging_current = 0.0 if docked else 1.5  # 上桩前放电 1.5A，上桩后停止放电
    msg.charging_current = CHARGING_CURRENT_A if docked else 0.0
    msg.capacity = 80.0
    msg.temperature = 30.0
    msg.cycle_count = 120.0  # float32 字段，必须给浮点
    pub.publish(msg)


node.create_timer(0.1, tick)  # 10Hz 持续发，任何时刻帧龄 ≤0.1s（窗口 0.5s）
print(f'[fake_battery_state] 以 10Hz 发布 /agv_battery_state: '
      f'{DELAY_S:.1f}s 后充电电流={CHARGING_CURRENT_A:.2f}A（阈值 0.05A），'
      f'电压={VOLTAGE_V:.1f}V，Ctrl+C 退出')
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
