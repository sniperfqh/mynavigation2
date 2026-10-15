"""验证实际 launch 生成的参数文件，不启动 ROS 节点。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


class CollisionLaunchTest(unittest.TestCase):
    def resolve(self, mode, enabled, namespace='', composition=False):
        root = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location('collision_launch', root / 'launch/regulated_modules.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        captured = []
        original = module.ParameterFile

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
                                                  enable_collision_state=str(enabled).lower(), namespace=namespace,
                                                  use_composition=str(composition).lower())
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            for resolver in [module.resolve_collision_state_arguments, module.resolve_braking_margin_arguments]:
                for action in resolver(context):
                    action.execute(context)
            result = yaml.safe_load(Path(captured[0].evaluate(context)).read_text())
            return result[namespace] if namespace else result

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


if __name__ == '__main__':
    unittest.main()
