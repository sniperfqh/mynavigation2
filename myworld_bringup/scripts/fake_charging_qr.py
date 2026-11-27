#!/usr/bin/env python3
# 模拟充电二维码发布器：10Hz 持续发 /qrcode，坐标与上次充电起点一致 (9.16, -4.37) m。
# 时间戳用节点时钟实时打——这是 ros2 topic pub 做不到的（它构造一次消息后 stamp 不再刷新，
# 会被 navigator 的 charging_qr_max_age=0.5s 新鲜度闸门判过期）。
# 时钟源自动探测：有 /clock（Gazebo 模拟时钟）就跟 navigator 一样用 sim time，
# 没有则退回系统时钟，保证单独调试时也能发出消息。
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from byd_custom_msgs.msg import QrCode

# 用法: python3 fake_charging_qr.py [x_mm] [y_mm] [theta_deg]
#       python3 fake_charging_qr.py 9160.0 -4370.0 0.0
# 坐标为二维码在 map 帧的位姿（mm/度），默认与充电测试起点一致 (9.16, -4.37) m。
QR_X_MM = float(sys.argv[1]) if len(sys.argv) > 1 else 9160.0
QR_Y_MM = float(sys.argv[2]) if len(sys.argv) > 2 else -4370.0
QR_THETA_DEG = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

rclpy.init()

# 1) 探测 /clock 是否在发布（最多等 1.5s）。
probe = Node('fake_qr_clock_probe')
clock_seen = []
probe.create_subscription(Clock, '/clock',
                          lambda msg: clock_seen.append(msg), 10)
for _ in range(15):
    rclpy.spin_once(probe, timeout_sec=0.1)
    if clock_seen:
        break
use_sim_time = bool(clock_seen)
probe.destroy_node()
print(f'[fake_charging_qr] use_sim_time={use_sim_time} '
      f'({"检测到 /clock，跟随模拟时钟" if use_sim_time else "未检测到 /clock，使用系统时钟"})')

# 2) use_sim_time 必须在构造时经 parameter_overrides 传入：
#    rclpy 节点自动声明该参数，事后 declare_parameter 会抛
#    ParameterAlreadyDeclaredException。
node = Node('fake_charging_qr',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
pub = node.create_publisher(QrCode, '/qrcode', qos_profile_sensor_data)  # 与 navigator 订阅 QoS 匹配


def tick():
    msg = QrCode()
    msg.header.stamp = node.get_clock().now().to_msg()  # 关键：每帧实时时间戳
    msg.header.frame_id = 'map'
    msg.tag = 1
    msg.x = QR_X_MM
    msg.y = QR_Y_MM
    msg.theta = QR_THETA_DEG  # 度，回调内转弧度
    pub.publish(msg)


node.create_timer(0.1, tick)  # 10Hz 持续发，任何时刻帧龄 ≤0.1s
print(f'[fake_charging_qr] 以 10Hz 发布 /qrcode: tag=1, '
      f'({QR_X_MM / 1000.0:.3f}, {QR_Y_MM / 1000.0:.3f}) m, theta={QR_THETA_DEG}°，Ctrl+C 退出')
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
