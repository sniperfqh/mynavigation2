"""一次短时闭环消息测试，不启动底盘；数据与 S 曲线日志均留在 /tmp。"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import rclpy
from verify_jerk_log import verify_log
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

root = Path('/tmp/jerk-smoother-runtime-20261010')
root.mkdir(exist_ok=True)
env = os.environ.copy()
env['SPDLOG_WRAPPER_LOG_DIR'] = str(root / 'spdlog')
env['SPDLOG_WRAPPER_FILE_LEVEL'] = 'debug'
env['SPDLOG_WRAPPER_CONSOLE_LEVEL'] = 'info'
env['SPDLOG_WRAPPER_FLUSH_INTERVAL_SECONDS'] = '1'
executable = get_package_prefix('nav2_velocity_smoother') + '/lib/nav2_velocity_smoother/velocity_smoother'
rclpy.init()
node = rclpy.create_node('jerk_smoother_test')
publisher = node.create_publisher(Twist, '/jerk_input', 10)
odom = node.create_publisher(Odometry, '/jerk_feedback', 10)
samples = []
sub = node.create_subscription(Twist, '/jerk_output', lambda msg: samples.append([time.monotonic(), msg.linear.x, msg.angular.z]), 100)
process = None

def await_result(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    assert future.done(), 'service response timeout'
    return future.result()

try:
    with (root / 'console.log').open('w') as output:
        process = subprocess.Popen([executable, '--ros-args', '-p', 'smoothing_frequency:=100.0',
                                    '-p', 'jerk_limited_smoothing:=true', '-p', 'feedback:=CLOSED_LOOP',
                                    '-p', 'odom_topic:=/jerk_feedback', '-p', 'immediate_stop_on_zero_command:=true',
                                    '-r', 'cmd_vel:=/jerk_input', '-r', 'cmd_vel_smoothed:=/jerk_output'],
                                   env=env, stdout=output, stderr=subprocess.STDOUT)
        client = node.create_client(ChangeState, '/velocity_smoother/change_state')
        assert client.wait_for_service(timeout_sec=10)
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert await_result(client.call_async(request)).success
        setter = node.create_client(SetParameters, '/velocity_smoother/set_parameters')
        assert setter.wait_for_service(timeout_sec=3)
        request = SetParameters.Request()
        request.parameters = [Parameter('max_accel', value=[2.0, 0.0, 3.2]).to_parameter_msg()]
        assert not await_result(setter.call_async(request)).results[0].successful
        start = time.monotonic()
        while time.monotonic() - start < 7.5:
            elapsed = time.monotonic() - start
            message = Twist()
            message.linear.x = .3 if elapsed < 2 else 0.0 if elapsed < 3.5 else -.3 if elapsed < 5.5 else 0.0
            message.angular.z = .2 if elapsed < 2 else 0.0
            publisher.publish(message)
            feedback = Odometry()
            feedback.header.stamp = node.get_clock().now().to_msg()
            feedback.twist.twist.linear.x = samples[-1][1] * .9 if samples else 0.0
            feedback.twist.twist.angular.z = samples[-1][2] * .9 if samples else 0.0
            odom.publish(feedback)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(.005)
        assert samples and abs(samples[-1][1]) < 1e-6 and abs(samples[-1][2]) < 1e-6
finally:
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
    node.destroy_node()
    rclpy.shutdown()
    (root / 'output.json').write_text(json.dumps(samples))

assert process.returncode == 0
log = (root / 'spdlog/nav2_velocity_smoother/velocity_smoother.log').read_text()
assert 'S-curve enabled' in log
records = []
pattern = r'S-curve axis=(\d+) dt=([\d.]+) input=([\d.-]+) feedback_reference=([\d.-]+) target=([\d.-]+) output=([\d.-]+) accel=([\d.-]+) jerk=([\d.-]+)'
for match in re.finditer(pattern, log):
    axis, dt, requested, feedback, target, output, accel, jerk = map(float, match.groups())
    records.append([axis, dt, requested, feedback, target, output, accel, jerk])
validation = verify_log(root / 'spdlog/nav2_velocity_smoother/velocity_smoother.log')
assert len(records) > 900
assert any(x[2] > 0 for x in records) and any(x[2] < 0 for x in records)
summary = dict(status='PASS', samples=len(samples), log_records=len(records), peak_accel_x=max(abs(x[6]) for x in records if x[0] == 0), peak_jerk_x=max(abs(x[7]) for x in records if x[0] == 0), final_speed=samples[-1][1], log_file=str(root / 'spdlog/nav2_velocity_smoother/velocity_smoother.log'))
summary['constraint_validation'] = validation
(root / 'results.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
