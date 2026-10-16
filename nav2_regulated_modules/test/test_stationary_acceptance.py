# 分开验收窗口回归：短定位缺失可保留停稳，定位大缺口必须拒绝精度评价。
# 运动、时间回跳或速度采样大间隔重启停稳计时，定位缺失单独影响 pose_evaluable。
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'stationary_acceptance', Path(__file__).parents[1] / 'tools/stationary_acceptance.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


# 组织本文件的回归场景；各 test_ 方法构造输入并断言预期，不把模拟样本当作现场证据。
class StationaryAcceptanceTest(unittest.TestCase):
    # 短定位缺失不撤销独立速度停稳，但定位仍按覆盖率检查。
    def test_short_pose_loss_does_not_reset_stationary(self):
        window = module.StationaryAcceptance()
        for i in range(101):
            window.observe(i / 100, True, None if 40 <= i < 50 else [i / 100])
        result = window.evaluate(1.0)
        self.assertTrue(result['stationary'])
        self.assertTrue(result['pose_evaluable'])
        self.assertAlmostEqual(result['pose_coverage'], 0.9)

    # 没有位姿时可以确认速度停止，精度必须不可评价。
    def test_missing_pose_retains_stop_but_rejects_precision(self):
        window = module.StationaryAcceptance()
        for i in range(101):
            window.observe(i / 100, True)
        self.assertTrue(window.evaluate(1.0)['stationary'])
        self.assertFalse(window.evaluate(1.0)['pose_evaluable'])

    # 即使轮速连续零值，过大的定位缺口也必须拒绝精度评价。
    def test_large_pose_gap_rejects_precision(self):
        window = module.StationaryAcceptance()
        for i in range(101):
            window.observe(i / 100, True, None if 30 <= i < 60 else [i / 100])
        self.assertFalse(window.evaluate(1.0)['pose_evaluable'])

    # 运动或速度观察间隔过大都需重新开始停稳计时。
    def test_motion_or_velocity_sampling_gap_restarts_stop(self):
        for stopped, next_time in [(False, 0.9), (True, 1.2)]:
            window = module.StationaryAcceptance()
            for i in range(90):
                window.observe(i / 100, True, [i / 100])
            window.observe(next_time, stopped, [next_time])
            self.assertFalse(window.evaluate(next_time)['stationary'])


if __name__ == '__main__':
    unittest.main()
