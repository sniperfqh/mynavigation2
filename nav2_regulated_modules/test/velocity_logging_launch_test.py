"""检查三模式和碰撞监控参数路由，不启动节点。"""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters

path = Path(__file__).resolve().parents[1] / 'launch/regulated_modules.launch.py'
spec = importlib.util.spec_from_file_location('velocity_logging_launch', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
for mode in ('fixed_path', 'autonomous', 'remote'):
    for collision in ('true', 'false'):
        context = LaunchContext()
        context.launch_configurations.update({
            'params_file': str(path.parent.parent / 'params/regulated_modules.yaml'),
            'namespace': '', 'use_collision_monitor': collision, 'use_sim_time': 'false',
            'operation_mode': mode, 'log_dir': '/tmp/velocity-launch-test',
            'enable_velocity_diagnostics': 'true', 'velocity_file_log_frequency': '100.0',
            'velocity_console_log_frequency': '1.0'})
        actions = module.start_velocity_diagnostics(context)
        assert len(actions) == 1
        values = evaluate_parameters(context, actions[0]._Node__parameters)[0]
        assert values['operation_mode'] == mode
        assert values['smoother_feedback_topic'] == '/motion_state'
        assert values['controller_feedback_topic'] == '/odometry'
        assert values['smoother_output_topic'] == ('cmd_vel_collision_in' if collision == 'true' else 'cmd_vel')
        assert values['velocity_file_log_frequency'] == 100.0
        assert values['velocity_console_log_frequency'] == 1.0
        context.launch_configurations['enable_velocity_diagnostics'] = 'false'
        assert module.start_velocity_diagnostics(context) == []
print('PASS 6 enabled routes and 6 disabled routes')
