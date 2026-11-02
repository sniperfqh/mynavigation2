"""Fourth-mode profile and compatibility checks, without starting navigation."""
import importlib.util
from pathlib import Path
import unittest
import yaml
from launch import LaunchContext

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location('fixed_launch', ROOT / 'launch/regulated_modules.launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AutonomousFixedLaunchTest(unittest.TestCase):
    def test_collision_routing_for_standalone_and_composition(self):
        from test_collision_launch import CollisionLaunchTest
        resolver = CollisionLaunchTest()
        for composition in [False, True]:
            for namespace in ['', 'robot']:
                for enabled in [True, False]:
                    with self.subTest(composition=composition, namespace=namespace, enabled=enabled):
                        config = resolver.resolve('autonomous_fixed', enabled, namespace, composition)
                        for name in ['controller_server', 'regulated_navigator', 'velocity_smoother']:
                            self.assertEqual(config[name]['ros__parameters']['collision_state_enabled'], enabled)
                        self.assertEqual(config['controller_server']['ros__parameters']['controller_plugins'], ['AutonomousFixedPathController'])

    def test_fourth_mode_profiles_and_namespace(self):
        for source in [ROOT / 'params/regulated_modules.yaml', ROOT.parent / 'myworld_bringup/params/myworld2.yaml']:
            for namespace in ['', 'robot']:
                with self.subTest(source=source, namespace=namespace):
                    context = LaunchContext()
                    context.launch_configurations.update(operation_mode='autonomous_fixed', autonomous_mppi_profile='true')
                    output = MODULE.NavigationRewrittenYaml(source_file=str(source), root_key=namespace, param_rewrites={}, convert_types=True).perform(context)
                    config = yaml.safe_load(Path(output).read_text())
                    if namespace:
                        config = config[namespace]
                    navigator = config['regulated_navigator']['ros__parameters']
                    planner = config['planner_server']['ros__parameters']
                    controller = config['controller_server']['ros__parameters']
                    smoother = config['velocity_smoother']['ros__parameters']
                    self.assertEqual(navigator['planner_id'], planner['selected_planner'])
                    self.assertEqual(planner['selected_planner'], 'SmacHybrid')
                    self.assertEqual(planner['SmacHybrid']['motion_model_for_search'], 'REEDS_SHEPP')
                    self.assertEqual(planner['SmacHybrid']['plugin'], 'nav2_regulated_modules::AutonomousFixedHybridPlanner')
                    self.assertEqual(planner['SmacHybrid']['tolerance'], 0.003)
                    self.assertEqual(navigator['autonomous_fixed_xy_tolerance'], 0.01)
                    self.assertFalse(navigator['use_smoother'])
                    self.assertEqual(controller['controller_plugins'], ['AutonomousFixedPathController'])
                    self.assertEqual(controller['controller_frequency'], 100.0)
                    self.assertEqual(smoother['smoothing_frequency'], 100.0)
                    self.assertEqual(smoother['min_velocity'][0], -0.5)
                    self.assertEqual(controller['AutonomousFixedPathController']['tracker']['desired_linear_vel'], 0.5)
                    self.assertEqual(controller['AutonomousFixedPathController']['brake_stable_duration'], 0.2)
                    self.assertEqual(controller['AutonomousFixedPathController']['terminal_review_timeout'], 2.0)
                    self.assertEqual(controller['AutonomousFixedPathController']['terminal_capture_margin'], 0.02)
                    self.assertEqual(controller['autonomous_fixed_cusp_checker']['xy_goal_tolerance'], 0.01)
                    self.assertFalse(controller['autonomous_fixed_goal_checker']['stateful'])
                    self.assertAlmostEqual(controller['autonomous_fixed_goal_checker']['xy_goal_tolerance'], 0.007)
                    tracker = controller['AutonomousFixedPathController']['tracker']
                    self.assertEqual(tracker['goal_stop_entry_tolerance'], 0.005)
                    self.assertEqual(tracker['goal_final_approach_velocity'], 0.01)
                    self.assertEqual(tracker['goal_terminal_lookahead_min_dist'], 0.05)


if __name__ == '__main__':
    unittest.main()
