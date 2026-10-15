"""Resolve logging switches on the actual standalone/composed launch graph."""
import importlib.util
from pathlib import Path
import unittest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters


class LoggingRoutingTest(unittest.TestCase):
    def test_default_fallback_and_explicit_detail(self):
        root = Path(__file__).resolve().parents[1]
        for mode in ['fixed_path', 'autonomous', 'remote']:
            for namespace in ['', 'robot']:
                for composition in ['false', 'true']:
                    for diagnostics, detail, expected in [('true', 'false', False), ('false', 'false', True), ('true', 'true', True)]:
                        with self.subTest(mode=mode, namespace=namespace, composition=composition, diagnostics=diagnostics, detail=detail):
                            spec = importlib.util.spec_from_file_location('logging_launch', root / 'launch/regulated_modules.launch.py')
                            module = importlib.util.module_from_spec(spec)
                            spec.loader.exec_module(module)
                            files, nodes = [], []
                            original_file, original_node = module.ParameterFile, module.Node
                            def capture_file(*args, **kwargs):
                                result = original_file(*args, **kwargs)
                                files.append(result)
                                return result
                            def capture_node(**kwargs):
                                result = original_node(**kwargs)
                                nodes.append((kwargs, result))
                                return result
                            module.ParameterFile, module.Node = capture_file, capture_node
                            context = LaunchContext()
                            context.launch_configurations.update(operation_mode=mode, namespace=namespace, use_composition=composition,
                                                                 enable_velocity_diagnostics=diagnostics, enable_legacy_velocity_logs=detail)
                            description = module.generate_launch_description()
                            for action in description.entities:
                                if isinstance(action, DeclareLaunchArgument):
                                    action.execute(context)
                            for resolver in [module.resolve_collision_state_arguments, module.resolve_braking_margin_arguments]:
                                for action in resolver(context):
                                    action.execute(context)
                            parameters = yaml.safe_load(Path(files[0].evaluate(context)).read_text())
                            if namespace:
                                parameters = parameters[namespace]
                            for name in ['regulated_navigator', 'velocity_smoother']:
                                self.assertIs(parameters[name]['ros__parameters']['legacy_velocity_logging'], expected)
                            controls = [node for kwargs, node in nodes if kwargs.get('package') == 'controlpub']
                            self.assertEqual(len(controls), 2)
                            for node in controls:
                                merged = {}
                                for values in evaluate_parameters(context, node._Node__parameters):
                                    merged.update(values)
                                self.assertIs(merged['legacy_velocity_logging'], expected)


if __name__ == '__main__':
    unittest.main()
