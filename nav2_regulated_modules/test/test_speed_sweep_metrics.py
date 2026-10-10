"""Acceptance must not hide missing samples, failed attempts or speed derating."""
import importlib.util
import pathlib
import unittest

TOOLS = pathlib.Path(__file__).resolve().parents[1] / 'tools'
SPEC = importlib.util.spec_from_file_location('metrics', TOOLS / 'speed_sweep_metrics.py')
metrics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(metrics)


class AcceptanceTests(unittest.TestCase):
    def samples(self, x=.009, v=0.):
        return [(i * .02, x, 0., v, 0., .01) for i in range(61)]

    def test_full_window_and_threshold(self):
        window = metrics.first_stopped_window(self.samples())
        self.assertTrue(metrics.evaluate(window, (0., 0.), (1., 0.))['passed'])
        self.assertFalse(metrics.evaluate(self.samples(.011), (0., 0.), (1., 0.))['passed'])

    def test_sampling_gap_invalidates_window(self):
        samples = self.samples()[:20] + self.samples()[40:]
        self.assertFalse(metrics.first_stopped_window(samples))

    def test_motion_during_observation_is_not_stopped(self):
        self.assertFalse(metrics.first_stopped_window(self.samples(v=.011)))

    def test_derated_speed_is_not_requested_speed(self):
        self.assertFalse(metrics.motion_metrics(self.samples(v=.08), 0., 1.2, (1., 0.), .1, .01)['speed_reached'])
        self.assertTrue(metrics.motion_metrics(self.samples(v=.1), 0., 1.2, (1., 0.), .1, .01)['speed_reached'])

    def test_brief_speed_peak_is_not_cruise(self):
        samples = self.samples(v=.1)[:10] + self.samples(v=.08)[10:]
        self.assertFalse(metrics.motion_metrics(samples, 0., 1.2, (1., 0.), .1, .01)['speed_reached'])

    def test_nonfinite_position_is_rejected(self):
        samples = self.samples()
        samples[10] = (samples[10][0], float('nan'), 0., 0., 0., .01)
        self.assertFalse(metrics.evaluate(samples, (0., 0.), (1., 0.))['passed'])

    def test_frozen_simulation_clock_is_not_full_window(self):
        samples = [(*s, 0., 0., 10.) for s in self.samples()]
        self.assertFalse(metrics.first_stopped_window(samples))

    def test_missing_cruise_samples_do_not_count_as_held_speed(self):
        samples = [(0., 0., 0., .1, 0., .01), (.6, 0., 0., .1, 0., .01), (.62, 0., 0., 0., 0., .01)]
        self.assertFalse(metrics.motion_metrics(samples, 0., 1., (1., 0.), .1, .01)['speed_reached'])

    def test_later_good_window_cannot_hide_initial_stopped_gap(self):
        early = self.samples()[:10]
        later = [(s[0] + .5, *s[1:]) for s in self.samples()]
        self.assertFalse(metrics.first_stopped_window(early + later))

    def test_stopped_window_starts_after_threshold_transient(self):
        early = [(0., .005, 0., .0099, 0., .01), (.02, .005, 0., .01001, 0., .01)]
        stable = [(s[0] + .04, *s[1:]) for s in self.samples()]
        self.assertTrue(metrics.first_stopped_window(early + stable))

    def test_earlier_bad_position_is_not_hidden(self):
        samples = [(0., .011, 0., .01001, 0., .01)] + self.samples()
        self.assertFalse(metrics.observed_position_valid(samples, (0., 0.), 1.2))

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
