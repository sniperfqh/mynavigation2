"""Separate stationary velocity evidence from sampled localization coverage."""
import math


class StationaryAcceptance:
    def __init__(self, duration=1.0):
        self.duration = duration
        self.stationary_since = None
        self.last_time = None
        self.samples = []

    def observe(self, now, stopped, pose=None):
        if (not stopped or not math.isfinite(now) or
                (self.last_time is not None and
                 (now < self.last_time or now - self.last_time > 0.2))):
            self.stationary_since = None
            self.samples = []
        self.last_time = now
        if not stopped or not math.isfinite(now):
            return
        if self.stationary_since is None:
            self.stationary_since = now
        if pose is not None and not all(math.isfinite(v) for v in pose):
            pose = None
        self.samples.append((now, pose))
        # Keep one boundary sample to measure coverage by time, not query count.
        while len(self.samples) > 1 and self.samples[1][0] <= now - self.duration:
            self.samples.pop(0)

    def evaluate(self, now):
        stationary = (self.stationary_since is not None and
                      now - self.stationary_since >= self.duration)
        begin = now - self.duration
        points = [p for t, p in self.samples if t >= begin and p is not None]
        covered = 0.0
        for (t, p), (end, _) in zip(self.samples, self.samples[1:]):
            if p is not None and end - t <= 0.2:
                covered += max(0.0, min(end, now) - max(t, begin))
        times = [begin] + [t for t, p in self.samples
                           if begin <= t <= now and p is not None] + [now]
        gap = max((b - a for a, b in zip(times, times[1:])), default=self.duration)
        coverage = min(1.0, covered / self.duration)
        evaluable = stationary and len(points) >= 10 and coverage >= 0.8 and gap <= 0.2
        return {
            'stationary': stationary,
            'stationary_since': self.stationary_since,
            'pose_evaluable': evaluable,
            'pose_coverage': coverage,
            'pose_max_gap_s': gap,
            'pose_sample_count': len(points),
            'window_begin': begin,
            'window_end': now,
            'poses': points,
            'protocol': 'separated_velocity_pose_v2',
        }
