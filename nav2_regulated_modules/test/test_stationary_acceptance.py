import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'stationary_acceptance', Path(__file__).parents[1] / 'tools/stationary_acceptance.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StationaryAcceptanceTest(unittest.TestCase):
    def test_short_pose_loss_does_not_reset_stationary(self):
        window = module.StationaryAcceptance()
        for i in range(101):
            window.observe(i / 100, True, None if 40 <= i < 50 else [i / 100])
        result = window.evaluate(1.0)
        self.assertTrue(result['stationary'])
        self.assertTrue(result['pose_evaluable'])
        self.assertAlmostEqual(result['pose_coverage'], 0.9)

    def test_missing_pose_retains_stop_but_rejects_precision(self):
        window = module.StationaryAcceptance()
        for i in range(101):
            window.observe(i / 100, True)
        self.assertTrue(window.evaluate(1.0)['stationary'])
        self.assertFalse(window.evaluate(1.0)['pose_evaluable'])

    def test_large_pose_gap_rejects_precision(self):
        window = module.StationaryAcceptance()
        for i in range(101):
            window.observe(i / 100, True, None if 30 <= i < 60 else [i / 100])
        self.assertFalse(window.evaluate(1.0)['pose_evaluable'])

    def test_motion_or_velocity_sampling_gap_restarts_stop(self):
        for stopped, next_time in [(False, 0.9), (True, 1.2)]:
            window = module.StationaryAcceptance()
            for i in range(90):
                window.observe(i / 100, True, [i / 100])
            window.observe(next_time, stopped, [next_time])
            self.assertFalse(window.evaluate(next_time)['stationary'])


if __name__ == '__main__':
    unittest.main()
