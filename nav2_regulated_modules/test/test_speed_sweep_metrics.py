# 纯指标回归：完整停后窗、缺帧、微量运动、未达速、时钟冻结及失败批次不得被误判通过。
# 边界输入对应采集协议的时间／位置／速度排列，修改注释不能放宽断言阈值。
"""Acceptance must not hide missing samples, failed attempts or speed derating."""
import importlib.util
import pathlib
import unittest

TOOLS = pathlib.Path(__file__).resolve().parents[1] / 'tools'
SPEC = importlib.util.spec_from_file_location('metrics', TOOLS / 'speed_sweep_metrics.py')
metrics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(metrics)


# 组织本文件的回归场景；各 test_ 方法构造输入并断言预期，不把模拟样本当作现场证据。
class AcceptanceTests(unittest.TestCase):
    # 构造固定时间步长的数值样本，x 为位置偏差 m、v 为线速度 m/s；不是 ROS 实测数据。
    def samples(self, x=.009, v=0.):
        return [(i * .02, x, 0., v, 0., .01) for i in range(61)]

    # 区分完整合格窗口与位置超过 10 mm 的样本，内部任务成功不能替代外部阈值。
    def test_full_window_and_threshold(self):
        window = metrics.first_stopped_window(self.samples())
        self.assertTrue(metrics.evaluate(window, (0., 0.), (1., 0.))['passed'])
        self.assertFalse(metrics.evaluate(self.samples(.011), (0., 0.), (1., 0.))['passed'])

    # 故意移除中间样本，验证时间覆盖不足不能算连续停稳。
    def test_sampling_gap_invalidates_window(self):
        samples = self.samples()[:20] + self.samples()[40:]
        self.assertFalse(metrics.first_stopped_window(samples))

    # 反馈线速度超过停止阈值时不得通过停后窗口。
    def test_motion_during_observation_is_not_stopped(self):
        self.assertFalse(metrics.first_stopped_window(self.samples(v=.011)))

    # 实际只到低于请求的速度时拒绝达速，避免比较不同实际速度的效率。
    def test_derated_speed_is_not_requested_speed(self):
        self.assertFalse(metrics.motion_metrics(self.samples(v=.08), 0., 1.2, (1., 0.), .1, .01)['speed_reached'])
        self.assertTrue(metrics.motion_metrics(self.samples(v=.1), 0., 1.2, (1., 0.), .1, .01)['speed_reached'])

    # 短暂触达请求速度不足以形成至少半秒巡航证据。
    def test_brief_speed_peak_is_not_cruise(self):
        samples = self.samples(v=.1)[:10] + self.samples(v=.08)[10:]
        self.assertFalse(metrics.motion_metrics(samples, 0., 1.2, (1., 0.), .1, .01)['speed_reached'])

    # 位置含 NaN 时拒绝精度通过，不能在统计中静默忽略坏样本。
    def test_nonfinite_position_is_rejected(self):
        samples = self.samples()
        samples[10] = (samples[10][0], float('nan'), 0., 0., 0., .01)
        self.assertFalse(metrics.evaluate(samples, (0., 0.), (1., 0.))['passed'])

    # 仿真时钟冻结时，墙钟样本数量增长也不能冒充完整仿真时间窗。
    def test_frozen_simulation_clock_is_not_full_window(self):
        samples = [(*s, 0., 0., 10.) for s in self.samples()]
        self.assertFalse(metrics.first_stopped_window(samples))

    # 相隔过久的两个达速点不组成连续巡航平台。
    def test_missing_cruise_samples_do_not_count_as_held_speed(self):
        samples = [(0., 0., 0., .1, 0., .01), (.6, 0., 0., .1, 0., .01), (.62, 0., 0., 0., 0., .01)]
        self.assertFalse(metrics.motion_metrics(samples, 0., 1., (1., 0.), .1, .01)['speed_reached'])

    # 停后初始采样缺口不能被后续更好的窗口替换掩盖。
    def test_later_good_window_cannot_hide_initial_stopped_gap(self):
        early = self.samples()[:10]
        later = [(s[0] + .5, *s[1:]) for s in self.samples()]
        self.assertFalse(metrics.first_stopped_window(early + later))

    # 停止阈值附近的越界样本必须重置窗口，再从后续稳定段计时。
    def test_stopped_window_starts_after_threshold_transient(self):
        early = [(0., .005, 0., .0099, 0., .01), (.02, .005, 0., .01001, 0., .01)]
        stable = [(s[0] + .04, *s[1:]) for s in self.samples()]
        self.assertTrue(metrics.first_stopped_window(early + stable))

    # 先前已观察到的超容差位置不能被后续合格样本覆盖掉。
    def test_earlier_bad_position_is_not_hidden(self):
        samples = [(0., .011, 0., .01001, 0., .01)] + self.samples()
        self.assertFalse(metrics.observed_position_valid(samples, (0., 0.), 1.2))

    # 候选或基线不足／失败时不应产出效率改善通过结论。
    def test_failed_attempt_cannot_produce_efficiency_pass(self):
        spec = importlib.util.spec_from_file_location('sweep', TOOLS / 'sweep_stop_efficiency.py')
        sweep = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sweep)
        baseline = []
        candidate = []
        for direction in ('forward', 'reverse'):
            for index in range(10):
                baseline.append({'direction': direction, 'accepted': True, 'terminal_duration_s': 20., 'total_duration_s': 30.})
                candidate.append({'direction': direction, 'accepted': index != 0, 'terminal_duration_s': 10., 'total_duration_s': 20.})
        self.assertFalse(sweep.compare(baseline, candidate)['forward']['passed'])


if __name__ == '__main__':
    unittest.main()
