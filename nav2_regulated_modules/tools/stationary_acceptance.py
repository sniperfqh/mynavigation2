# 速度停稳与定位质量分开的纯窗口计算器，不依赖 ROS，也不发送命令。
# 速度连续条件先独立成立，再检查定位覆盖、最大缺帧与样本数；短暂定位缺失不会伪装成完整连续定位。
"""Separate stationary velocity evidence from sampled localization coverage."""
import math


# 保存 duration（s）和空的停稳／定位历史，类本身不读取 ROS 时钟。
class StationaryAcceptance:
    # 保存 duration（s）和空的停稳／定位历史，类本身不读取 ROS 时钟。
    def __init__(self, duration=1.0):
        self.duration = duration
        self.stationary_since = None
        self.last_time = None
        self.samples = []

    # 运动、非有限时间、时间回跳或速度观察 gap>0.2 s 会重置停稳。
    # pose 缺失只作为定位无效样本记录，保留一条窗边界样本以按时间算覆盖率。
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
        # 保留窗边界前一条样本，以有效时间占比计算覆盖率，而非按有效查询次数占比。
        while len(self.samples) > 1 and self.samples[1][0] <= now - self.duration:
            self.samples.pop(0)

    # 独立输出 stationary 与 pose_evaluable；定位还要求≥10 样本、覆盖≥80%、最大缺口≤0.2 s。
    # 边界参与缺口计算，不在缺失区间插值；本函数只给可评价性，不直接判 XY 精度通过。
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
