# 通过实际 Launch 解析验证 collision 开关、默认启用、显式关闭、命名空间和组合节点路由。
# 捕获生成参数文件检查三节点一致性，不运行生命周期或底盘。
"""验证实际 launch 生成的参数文件，不启动 ROS 节点。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


# 组织本文件的回归场景；各 test_ 方法构造输入并断言预期，不把模拟样本当作现场证据。
class CollisionLaunchTest(unittest.TestCase):
    # 构造临时参数文件与 LaunchContext 并执行解析，返回已生成的节点参数以便断言。
    def resolve(self, mode, enabled, namespace='', composition=False):
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('collision_launch', root / 'launch/regulated_modules.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        captured = []
        original = module.ParameterFile

        # 暂时包装 ParameterFile 构造器收集结果，仍调用原构造器；不执行 Node 进程。
        def capture(*args, **kwargs):
            value = original(*args, **kwargs)
            captured.append(value)
            return value

        module.ParameterFile = capture
        config = yaml.safe_load((root / 'params/regulated_modules.yaml').read_text())
        config['velocity_smoother']['ros__parameters']['collision_state_topic'] = '/test/collision'
        config['velocity_smoother']['ros__parameters']['collision_state_timeout'] = 0.7
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'params.yaml'
            path.write_text(yaml.safe_dump(config))
            context = LaunchContext()
            context.launch_configurations.update(params_file=str(path), operation_mode=mode,
                                                  namespace=namespace,
                                                  use_composition=str(composition).lower())
            if enabled is not None:
                context.launch_configurations['enable_collision_state'] = str(enabled).lower()
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            for resolver in [module.resolve_collision_state_arguments, module.resolve_braking_margin_arguments]:
                for action in resolver(context):
                    action.execute(context)
            result = yaml.safe_load(Path(captured[0].evaluate(context)).read_text())
            return result[namespace] if namespace else result

    # 在各模式、命名空间组合中核对三个节点统一的状态话题、超时和开关。
    def test_all_modes_and_namespace(self):
        for mode, enabled, namespace in [('fixed_path', True, ''), ('fixed_path', False, ''),
                                          ('autonomous', True, ''), ('remote', True, ''),
                                          ('fixed_path', True, 'robot')]:
            with self.subTest(mode=mode, enabled=enabled, namespace=namespace):
                for composition in [False, True]:
                    result = self.resolve(mode, enabled, namespace, composition)
                    for node in ['controller_server', 'velocity_smoother', 'regulated_navigator']:
                        parameters = result[node]['ros__parameters']
                        self.assertEqual(parameters['collision_state_enabled'], mode == 'fixed_path' and enabled)
                        self.assertEqual(parameters['collision_state_topic'], '/test/collision')
                        self.assertEqual(parameters['collision_state_timeout'], 0.7)

    # 核对默认开启仅 fixed_path 生效，显式关闭覆盖默认，组件模式同样适用。
    def test_default_and_explicit_disable(self):
        for mode in ['fixed_path', 'autonomous', 'remote']:
            for composition in [False, True]:
                for enabled in [None, False]:
                    with self.subTest(mode=mode, composition=composition, enabled=enabled):
                        result = self.resolve(mode, enabled, composition=composition)
                        for node in ['controller_server', 'velocity_smoother', 'regulated_navigator']:
                            self.assertEqual(result[node]['ros__parameters']['collision_state_enabled'],
                                             mode == 'fixed_path' and enabled is None)


if __name__ == '__main__':
    unittest.main()
