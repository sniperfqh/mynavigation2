# 隔离启动诊断节点并合成输入，验证来源去重、数据状态、日志频率和滚动。
# 它会启动自己的测试进程，但不发布实际底盘运动目标；结果仅证明诊断协议。
"""可控消息集成验证；独立 ROS_DOMAIN_ID 下运行，不启动底盘。"""
import os
import pathlib
import re
import signal
import subprocess
import tempfile
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from byd_custom_msgs.msg import MotionState, ControlRes, ChassisControl


# 在独立日志目录启动诊断节点，合成各来源并检查日志字段、频率与缺失状态，结束后只清理自己的进程。
def run_case(node, mode, collision=False, feedback='/motion_state', directory=None, shared_controller=False):
    root = pathlib.Path(directory or tempfile.mkdtemp(prefix='velocity-diag-'))
    capture = root / 'console.log' if directory is None else pathlib.Path(tempfile.mktemp(prefix='velocity-console-'))
    command = [get_package_prefix('nav2_regulated_modules') + '/lib/nav2_regulated_modules/velocity_diagnostics_node', '--ros-args',
               '-p', 'operation_mode:=' + mode, '-p', 'use_collision_monitor:=' + str(collision).lower(),
               '-p', 'smoother_output_topic:=' + ('cmd_vel_collision_in' if collision else 'cmd_vel'),
               '-p', 'smoother_feedback_topic:=' + feedback, '-p', 'log_dir:=' + str(root)]
    if shared_controller:
        command += ['-p', 'controller_feedback_topic:=motion_state']
    publishers = [(node.create_publisher(Twist, '/cmd_vel_nav', 10), Twist()),
                  (node.create_publisher(Twist, '/cmd_vel', 10), Twist()),
                  (node.create_publisher(Twist, '/cmd_vel_collision_in', 10), Twist()),
                  (node.create_publisher(Odometry, '/odom', 10), Odometry()),
                  (node.create_publisher(MotionState, '/motion_state', 10), MotionState()),
                  (node.create_publisher(ControlRes, '/control_to_uart', 10), ControlRes()),
                  (node.create_publisher(ChassisControl, '/downstream/chassis_control', 10), ChassisControl())]
    with capture.open('w') as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        try:
            time.sleep(1.2)
            discovery_deadline = time.monotonic() + 10
            selected = [publishers[4][0], publishers[5][0]] if mode == 'remote' else [publishers[i][0] for i in (0, 1, 4, 5)]
            while any(pub.get_subscription_count() == 0 for pub in selected):
                if time.monotonic() > discovery_deadline:
                    raise RuntimeError('DDS discovery timeout: ' + str(capture))
                rclpy.spin_once(node, timeout_sec=0.05)
            start = time.monotonic()
            assert publishers[4][0].get_subscription_count() == 1, 'MotionState source subscribed more than once'
            while time.monotonic() - start < 3.2:
                elapsed = time.monotonic() - start
                speed = 0.3 if elapsed < 1 else -0.3 if elapsed < 2 else float('nan') if 2.8 < elapsed < 3.0 else 0.0
                for pub, msg in publishers:
                    if isinstance(msg, Twist):
                        msg.linear.x = speed
                    elif isinstance(msg, Odometry):
                        msg.twist.twist.linear.x = speed
                    elif isinstance(msg, MotionState):
                        msg.v_car = speed
                        msg.v_lift, msg.lift_height, msg.w_shelf, msg.yaw_shelf = .1, .2, .3, .4
                    elif isinstance(msg, ChassisControl):
                        msg.linear_velocity = speed
                        msg.op = 1
                    else:
                        msg.v = speed
                    pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0)
                time.sleep(0.01)
            time.sleep(1.1)
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
            for pub, _ in publishers:
                node.destroy_publisher(pub)
    assert process.returncode == 0, capture.read_text()
    console = capture.read_text()
    if directory is not None:
        assert 'File diagnostics disabled' in console
        print('PASS unwritable directory', capture)
        return
    text = (root / 'nav2_regulated_modules/velocity_diagnostics.log').read_text()
    assert 'status=missing' in text and 'status=stale' in text and 'status=valid' in text and 'status=invalid' in text
    for value in ('vx=0.300000', 'vx=-0.300000', 'vx=0.000000'):
        assert value in text, value
    stage = 'remote_out' if mode == 'remote' else 'controller_out/smoother_in'
    samples = [float(x) for x in re.findall(r'\[debug\] velocity ros_time=([\d.]+) mode=' + mode + ' schema=2', text)]
    hz = (len(samples)-1)/(samples[-1]-samples[0])
    assert 95 < hz < 105, hz
    summaries = [line.split('[info] ', 1)[1] for line in console.splitlines() if '[info] velocity ros_time=' in line]
    assert all(line in text for line in summaries)
    assert 4 <= sum('stage=' + stage + ' ' in line for line in summaries) <= 20
    assert ('stage=collision_out/final_cmd' in text) == collision
    assert ('stage=chassis_control_in' in text) == (mode != 'remote')
    assert ('stage=controller_out/smoother_in' in text) == (mode != 'remote')
    frames = [line for line in text.splitlines() if '[debug] velocity ros_time=' in line]
    assert all(line.count('topic=/motion_state ') == 1 for line in frames)
    assert 'v_lift=0.100000 lift_height=0.200000 w_shelf=0.300000 yaw_shelf=0.400000' in text
    if mode != 'remote' and feedback.lstrip('/') == 'motion_state':
        assert 'smoother_feedback,chassis_feedback' in text
    if shared_controller:
        assert 'stage=controller_feedback,smoother_feedback,chassis_feedback' in text
    print('PASS', mode, collision, feedback, 'file_hz=', round(hz, 2), root)


# 通过提高诊断采样频率触发滚动文件，验证大小／备份和跨文件来源标识，不测试车辆旋转。
def run_rotation():
    root = pathlib.Path(tempfile.mkdtemp(prefix='velocity-rotation-'))
    executable = get_package_prefix('nav2_regulated_modules') + '/lib/nav2_regulated_modules/velocity_diagnostics_node'
    with (root / 'console.log').open('w') as output:
        process = subprocess.Popen([executable, '--ros-args', '-p', 'velocity_file_log_frequency:=1000.0', '-p', 'log_dir:=' + str(root)], stdout=output, stderr=subprocess.STDOUT)
        try:
            time.sleep(14)
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
    files = list((root / 'nav2_regulated_modules').glob('velocity_diagnostics*.log'))
    assert process.returncode == 0 and len(files) >= 2, files
    assert all(p.stat().st_size <= 10 * 1024 * 1024 for p in files)
    text = (root / 'nav2_regulated_modules/velocity_diagnostics.log').read_text()
    assert text.endswith('\n')
    print('PASS rotation and shutdown', root)


if __name__ == '__main__':
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) != 0, 'Use an isolated nonzero ROS_DOMAIN_ID'
    rclpy.init()
    node = rclpy.create_node('velocity_diagnostics_test')
    try:
        run_case(node, 'fixed_path')
        run_case(node, 'autonomous', True, '/odom')
        run_case(node, 'remote')
        run_case(node, 'fixed_path', feedback='motion_state', shared_controller=True)
        run_case(node, 'fixed_path', directory='/proc/velocity-diagnostics-test')
        run_rotation()
    finally:
        node.destroy_node()
        rclpy.shutdown()
