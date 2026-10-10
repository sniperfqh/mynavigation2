import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml
from launch import LaunchContext
from nav2_common.launch import RewrittenYaml

spec = importlib.util.spec_from_file_location(
    'margin_launch', Path(__file__).parents[1] / 'launch/regulated_modules.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
KEYS = ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
        'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']


class MarginLaunchTest(unittest.TestCase):
    def resolve(self, config, overrides):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'params.yaml'
            source.write_text(yaml.safe_dump(config))
            context = LaunchContext()
            context.launch_configurations['params_file'] = str(source)
            for key in KEYS:
                context.launch_configurations['fixed_path_' + key] = overrides.get(key, '')
            for action in module.resolve_braking_margin_arguments(context):
                action.execute(context)
            rewrites = {'controller_server.ros__parameters.FixedPathController.' + key:
                        context.launch_configurations['resolved_fixed_path_' + key] for key in KEYS}
            output = RewrittenYaml(source_file=str(source), param_rewrites=rewrites,
                                   convert_types=True).perform(context)
            return yaml.safe_load(Path(output).read_text())['controller_server']['ros__parameters']['FixedPathController']

    def test_yaml_values_preserved_and_typed(self):
        values = dict(zip(KEYS, [.12, False, .04, .4]))
        result = self.resolve({'controller_server': {'ros__parameters': {'FixedPathController': values}}}, {})
        self.assertEqual(result, values)
        self.assertIs(type(result[KEYS[1]]), bool)

    def test_explicit_override_wins(self):
        values = dict(zip(KEYS, [.12, False, .04, .4]))
        result = self.resolve({'controller_server': {'ros__parameters': {'FixedPathController': values}}},
                              {KEYS[0]: '.1', KEYS[1]: 'true', KEYS[2]: '.05'})
        self.assertEqual(result[KEYS[0]], .1)
        self.assertIs(result[KEYS[1]], True)
        self.assertEqual(result[KEYS[2]], .05)
        self.assertEqual(result[KEYS[3]], .4)

    def test_simulation_empty_override_keeps_yaml(self):
        self.check_simulation_route('', .12)

    def test_simulation_explicit_override_forwarded(self):
        self.check_simulation_route('.2', .2)

    def check_simulation_route(self, override, expected):
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
        path = Path(__file__).parents[2] / 'myworld_bringup/launch/navigation_launch.py'
        spec = importlib.util.spec_from_file_location('simulation_margin_launch', path)
        simulation = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(simulation)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'params.yaml'
            source.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': {'FixedPathController': {'goal_braking_distance_margin': .12}}}}))
            context = LaunchContext()
            context.launch_configurations.update({'params_file': str(source), 'fixed_path_goal_braking_distance_margin': override})
            description = simulation.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            include = next(action for action in description.entities if isinstance(action, IncludeLaunchDescription))
            forwarded = {key: perform_substitutions(context, normalize_to_list_of_substitutions(value)) for key, value in include.launch_arguments}
            config = yaml.safe_load(Path(forwarded['params_file']).read_text())
            self.assertEqual(config['controller_server']['ros__parameters']['FixedPathController']['goal_braking_distance_margin'], .12)
            result = self.resolve(config, {KEYS[0]: forwarded['fixed_path_goal_braking_distance_margin']})
            self.assertEqual(result[KEYS[0]], expected)

    def test_invalid_boolean_rejected(self):
        with self.assertRaises(ValueError):
            self.resolve({}, {KEYS[1]: 'invalid'})


if __name__ == '__main__':
    unittest.main()
