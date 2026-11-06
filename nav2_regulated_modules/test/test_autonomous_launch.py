"""Check mode-specific parameter changes without starting ROS nodes."""
import importlib.util
import ast
import copy
from pathlib import Path
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('autonomous_launch', ROOT / 'launch/regulated_modules.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AutonomousLaunchTest(unittest.TestCase):
    def expected_fixed_path_limits(self, document, namespace=''):
        expected = copy.deepcopy(document)
        config = expected[namespace] if namespace else expected
        smoother = config['velocity_smoother']['ros__parameters']
        for name, value in [('max_accel', 0.3), ('max_decel', -0.3),
                            ('max_accel_jerk', 1.0), ('max_decel_jerk', 1.0)]:
            smoother[name][0] = value
        return expected

    def resolve(self, mode, profile, namespace='', planner_id=''):
        source = ROOT.parent / 'myworld_bringup/params/myworld2.yaml'
        context = LaunchContext()
        context.launch_configurations.update(operation_mode=mode, autonomous_mppi_profile=str(profile).lower(), autonomous_planner_id=planner_id)
        common = {'controller_server.ros__parameters.FixedPathController.goal_linear_deceleration': '0.25'}
        kwargs = dict(source_file=str(source), root_key=namespace, param_rewrites=common, convert_types=True)
        resolved = module.NavigationRewrittenYaml(**kwargs).perform(context)
        baseline = RewrittenYaml(**kwargs).perform(context)
        return yaml.safe_load(Path(resolved).read_text()), yaml.safe_load(Path(baseline).read_text())

    def test_fixed_motion_limits_and_remote_compatibility(self):
        for mode in ['fixed_path', 'remote']:
            for profile in [False, True]:
                for namespace in ['', 'robot']:
                    with self.subTest(mode=mode, profile=profile, namespace=namespace):
                        resolved, baseline = self.resolve(mode, profile, namespace)
                        if mode == 'fixed_path':
                            baseline = self.expected_fixed_path_limits(baseline, namespace)
                        self.assertEqual(resolved, baseline)

    def test_simulation_planner_override_is_autonomous_only(self):
        for mode in ['autonomous', 'fixed_path', 'remote']:
            for profile in [False, True]:
                for namespace in ['', 'robot']:
                    with self.subTest(mode=mode, profile=profile, namespace=namespace):
                        resolved, baseline = self.resolve(mode, profile, namespace, planner_id='SmacHybrid')
                        if namespace:
                            resolved, baseline = resolved[namespace], baseline[namespace]
                        if mode == 'autonomous':
                            planner = resolved['regulated_navigator']['ros__parameters']['planner_id']
                            server = resolved['planner_server']['ros__parameters']
                            self.assertEqual(planner, 'SmacHybrid')
                            self.assertEqual(server['selected_planner'], planner)
                            self.assertEqual(server[server['selected_planner']]['plugin'],
                                             'nav2_smac_planner/SmacPlannerHybrid')
                            self.assertEqual(server[server['selected_planner']]['motion_model_for_search'],
                                             'REEDS_SHEPP')
                        else:
                            if mode == 'fixed_path':
                                baseline = self.expected_fixed_path_limits(baseline)
                            self.assertEqual(resolved, baseline)

    def test_autonomous_mppi_wrapper_and_settle(self):
        for namespace in ['', 'robot']:
            resolved, baseline = self.resolve('autonomous', True, namespace)
            if namespace:
                resolved, baseline = resolved[namespace], baseline[namespace]
            self.assertEqual(resolved['planner_server']['ros__parameters']['SmacHybrid']['motion_model_for_search'],
                             'REEDS_SHEPP')
            controller = resolved['controller_server']['ros__parameters']
            self.assertEqual(controller['controller_frequency'], 50.0)
            self.assertEqual(controller['MPPI']['plugin'], 'nav2_regulated_modules::AutonomousMPPIController')
            self.assertEqual(controller['MPPI']['primary_controller'], 'nav2_mppi_controller::MPPIController')
            self.assertEqual(controller['MPPI']['batch_size'], 300)
            self.assertEqual(controller['MPPI']['vx_max'], 0.5)
            self.assertTrue(controller['MPPI']['visualize'])
            self.assertEqual(controller['MPPI']['TrajectoryVisualizer'], {'trajectory_step': 15, 'time_step': 5})
            self.assertEqual(controller['MPPI']['model_dt'], 0.02)
            self.assertEqual(controller['MPPI']['time_steps'], 75)
            self.assertAlmostEqual(controller['MPPI']['time_steps'] * controller['MPPI']['model_dt'], 1.5)
            expected_smoother = dict(baseline['velocity_smoother']['ros__parameters'])
            expected_smoother['smoothing_frequency'] = 50.0
            self.assertEqual(resolved['velocity_smoother']['ros__parameters'], expected_smoother)
            self.assertEqual(controller['MPPI']['autonomous_terminal_distance'], 0.15)
            self.assertEqual(controller['MPPI']['autonomous_terminal_max_linear_velocity'], 0.05)
            self.assertEqual(controller['MPPI']['autonomous_terminal_max_angular_velocity'],
                             baseline['controller_server']['ros__parameters']['MPPI'].get('autonomous_terminal_max_angular_velocity', 0.1))
            navigator = resolved['regulated_navigator']['ros__parameters']
            self.assertTrue(navigator['autonomous_terminal_progress_enabled'])
            self.assertEqual(navigator['autonomous_terminal_xy_tolerance'], controller['stopped_goal_checker']['xy_goal_tolerance'])
            self.assertFalse(controller['MPPI']['GoalAngleCritic']['enabled'])
            self.assertEqual(controller['progress_checker']['plugin'], 'nav2_controller::PoseProgressChecker')
            self.assertEqual(controller['stopped_goal_checker']['settle_duration'], 0.5)
            self.assertEqual(controller['stopped_goal_checker']['xy_goal_tolerance'], 0.03)
            self.assertFalse(controller['stopped_goal_checker']['stateful'])
            self.assertEqual(controller['FixedPathController'], baseline['controller_server']['ros__parameters']['FixedPathController'])

    def test_scene_planner_selection_and_motion_model_follow_yaml(self):
        config = yaml.safe_load((ROOT.parent / 'myworld_bringup/params/myworld2.yaml').read_text())
        self.assertEqual(config['regulated_navigator']['ros__parameters']['planner_id'], 'SmacHybrid')
        self.assertEqual(config['planner_server']['ros__parameters']['selected_planner'], 'SmacHybrid')
        self.assertEqual(config['planner_server']['ros__parameters']['SmacHybrid']['motion_model_for_search'], 'REEDS_SHEPP')
        # A later YAML edit must not be silently overridden by the autonomous profile.
        config['regulated_navigator']['ros__parameters']['planner_id'] = 'GridBasedAstar'
        config['planner_server']['ros__parameters']['selected_planner'] = 'GridBasedAstar'
        config['planner_server']['ros__parameters']['SmacHybrid']['motion_model_for_search'] = 'DUBIN'
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'params.yaml'
            source.write_text(yaml.safe_dump(config))
            for profile in [False, True]:
                context = LaunchContext()
                context.launch_configurations.update(operation_mode='autonomous', autonomous_mppi_profile=str(profile).lower(), autonomous_planner_id='')
                output = module.NavigationRewrittenYaml(source_file=str(source), param_rewrites={}, convert_types=True).perform(context)
                resolved = yaml.safe_load(Path(output).read_text())
                self.assertEqual(resolved['regulated_navigator']['ros__parameters']['planner_id'], 'GridBasedAstar')
                self.assertEqual(resolved['planner_server'], config['planner_server'])

        entry = ast.parse((ROOT.parent / 'myworld_bringup/launch/entry.launch.py').read_text())
        planner_arguments = [value.value for item in ast.walk(entry) if isinstance(item, ast.Dict)
                             for key, value in zip(item.keys, item.values)
                             if isinstance(key, ast.Constant) and key.value == 'autonomous_planner_id'
                             and isinstance(value, ast.Constant)]
        self.assertEqual(planner_arguments, [''])

    def test_direct_launch_defaults_to_hybrid(self):
        description = module.generate_launch_description()
        argument = next(action for action in description.entities
                        if isinstance(action, DeclareLaunchArgument) and action.name == 'autonomous_planner_id')
        context = LaunchContext()
        context.launch_configurations['operation_mode'] = 'autonomous'
        argument.execute(context)
        self.assertEqual(context.launch_configurations['autonomous_planner_id'], 'SmacHybrid')
        source = ROOT.parent / 'myworld_bringup/params/myworld2.yaml'
        output = module.NavigationRewrittenYaml(source_file=str(source), param_rewrites={}, convert_types=True).perform(context)
        resolved = yaml.safe_load(Path(output).read_text())
        self.assertEqual(resolved['regulated_navigator']['ros__parameters']['planner_id'], 'SmacHybrid')
        self.assertEqual(resolved['planner_server']['ros__parameters']['selected_planner'], 'SmacHybrid')

    def test_direct_autonomous_does_not_replace_selected_controller(self):
        resolved, baseline = self.resolve('autonomous', False)
        controller = resolved['controller_server']['ros__parameters']
        self.assertEqual(controller['controller_frequency'], 50.0)
        expected_mppi = dict(baseline['controller_server']['ros__parameters']['MPPI'])
        expected_mppi.update(model_dt=0.02, time_steps=75, vx_max=0.5, visualize=True,
                             TrajectoryVisualizer={'trajectory_step': 15, 'time_step': 5})
        self.assertEqual(controller['MPPI'], expected_mppi)
        self.assertEqual(resolved['velocity_smoother']['ros__parameters']['smoothing_frequency'], 50.0)
        self.assertEqual(resolved['controller_server']['ros__parameters']['stopped_goal_checker'], baseline['controller_server']['ros__parameters']['stopped_goal_checker'])

    def test_diagnostic_defaults_and_explicit_override(self):
        entry_spec = importlib.util.spec_from_file_location('entry_autonomous_rates', ROOT.parent / 'myworld_bringup/launch/entry.launch.py')
        entry = importlib.util.module_from_spec(entry_spec)
        entry_spec.loader.exec_module(entry)
        for launch_module in [module, entry]:
            description = launch_module.generate_launch_description()
            argument = next(action for action in description.entities if isinstance(action, DeclareLaunchArgument) and action.name == 'velocity_file_log_frequency')
            for mode, expected in [('autonomous', '50.0'), ('fixed_path', '100.0'), ('remote', '100.0')]:
                context = LaunchContext()
                context.launch_configurations['operation_mode'] = mode
                argument.execute(context)
                self.assertEqual(context.launch_configurations['velocity_file_log_frequency'], expected)
                context.launch_configurations['velocity_file_log_frequency'] = '25.0'
                argument.execute(context)
                self.assertEqual(context.launch_configurations['velocity_file_log_frequency'], '25.0')

    def test_custom_terminal_parameters_and_goal_tolerance_are_forwarded(self):
        config = yaml.safe_load((ROOT.parent / 'myworld_bringup/params/myworld2.yaml').read_text())
        controller = config['controller_server']['ros__parameters']
        controller['stopped_goal_checker']['xy_goal_tolerance'] = 0.025
        controller['MPPI'].update(autonomous_terminal_distance=0.2, autonomous_terminal_max_linear_velocity=0.04, autonomous_terminal_max_angular_velocity=0.25)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'params.yaml'
            source.write_text(yaml.safe_dump({'robot': config}))
            context = LaunchContext()
            context.launch_configurations.update(operation_mode='autonomous', autonomous_mppi_profile='true')
            output = module.NavigationRewrittenYaml(source_file=str(source), param_rewrites={}, root_key='robot', convert_types=True).perform(context)
            resolved = yaml.safe_load(Path(output).read_text())['robot']
            self.assertEqual(resolved['controller_server']['ros__parameters']['MPPI']['autonomous_terminal_max_linear_velocity'], 0.04)
            self.assertEqual(resolved['controller_server']['ros__parameters']['MPPI']['autonomous_terminal_max_angular_velocity'], 0.25)
            self.assertEqual(resolved['regulated_navigator']['ros__parameters']['autonomous_terminal_distance'], 0.2)
            self.assertEqual(resolved['regulated_navigator']['ros__parameters']['autonomous_terminal_xy_tolerance'], 0.025)


if __name__ == '__main__':
    unittest.main()
