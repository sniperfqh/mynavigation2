"""Check autonomous-only parameter changes without starting ROS nodes."""
import importlib.util
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
    def resolve(self, mode, profile, namespace='', planner_id=''):
        source = ROOT.parent / 'myworld_bringup/params/myworld2.yaml'
        context = LaunchContext()
        context.launch_configurations.update(operation_mode=mode, autonomous_mppi_profile=str(profile).lower(), autonomous_planner_id=planner_id)
        common = {'controller_server.ros__parameters.FixedPathController.goal_linear_deceleration': '0.25'}
        kwargs = dict(source_file=str(source), root_key=namespace, param_rewrites=common, convert_types=True)
        resolved = module.NavigationRewrittenYaml(**kwargs).perform(context)
        baseline = RewrittenYaml(**kwargs).perform(context)
        return yaml.safe_load(Path(resolved).read_text()), yaml.safe_load(Path(baseline).read_text())

    def test_fixed_and_remote_are_byte_equivalent_parameters(self):
        for mode in ['fixed_path', 'remote']:
            for profile in [False, True]:
                for namespace in ['', 'robot']:
                    with self.subTest(mode=mode, profile=profile, namespace=namespace):
                        resolved, baseline = self.resolve(mode, profile, namespace)
                        self.assertEqual(resolved, baseline)

    def test_simulation_planner_override_is_autonomous_only(self):
        for mode in ['autonomous', 'fixed_path', 'remote']:
            resolved, baseline = self.resolve(mode, True, planner_id='SmacHybrid')
            planner = resolved['regulated_navigator']['ros__parameters']['planner_id']
            if mode == 'autonomous':
                self.assertEqual(planner, 'SmacHybrid')
                self.assertEqual(resolved['planner_server']['ros__parameters']['SmacHybrid']['plugin'],
                                 'nav2_smac_planner/SmacPlannerHybrid')
            else:
                self.assertEqual(resolved, baseline)

    def test_autonomous_mppi_wrapper_and_settle(self):
        for namespace in ['', 'robot']:
            resolved, baseline = self.resolve('autonomous', True, namespace)
            if namespace:
                resolved, baseline = resolved[namespace], baseline[namespace]
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
