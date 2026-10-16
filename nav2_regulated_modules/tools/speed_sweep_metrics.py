# 离线速度与停车指标。样本索引、时间类型和窗口质量须与采集器一致，不能把短峰值当巡航。
# 到点 Action、命令零速、反馈停稳、定位采样精度和蠕动耗时分别评价，避免一个通过项掩盖另一个缺失项。
"""Arrival acceptance uses a complete post-result window, never cached AMCL."""
import math


# 用初始 map 位姿、里程计位置和 yaw 建立二维刚性对照，limit 单位 m。
# 该变换只用于检测后续定位修正幅度，不是新的定位输入。
class LocalizationCorrectionGuard:
    # 用初始 map 位姿、里程计位置和 yaw 建立二维刚性对照，limit 单位 m。
    # 该变换只用于检测后续定位修正幅度，不是新的定位输入。
    def __init__(self, sample, yaw, limit=.20):
        self.cosine, self.sine = math.cos(yaw), math.sin(yaw)
        self.tx = sample[1] - self.cosine * sample[6] + self.sine * sample[7]
        self.ty = sample[2] - self.sine * sample[6] - self.cosine * sample[7]
        self.limit = limit
        self.consecutive = 0
        self.maximum = 0.

    # 以初始刚性对照预测地图位置，记录与当前定位的偏差及连续超限次数。
    # 连续两次超过 limit 才报告异常，单次波动不直接终止实验。
    def observe(self, sample):
        x = self.tx + self.cosine * sample[6] - self.sine * sample[7]
        y = self.ty + self.sine * sample[6] + self.cosine * sample[7]
        correction = math.hypot(sample[1] - x, sample[2] - y)
        self.maximum = max(self.maximum, correction)
        self.consecutive = self.consecutive + 1 if correction > self.limit else 0
        return self.consecutive >= 2


# 截取从首样本起达到 duration 的最早前缀；不足时返回原样本。
# 该函数只裁剪时间，不保证样本连续、有效或停稳。
def first_window(samples, duration=1.0):
    if not samples:
        return samples
    for index, sample in enumerate(samples):
        if sample[0] - samples[0][0] >= duration:
            return samples[:index + 1]
    return samples


# 连续检查速度、TF 年龄与采样间隔，遇到不合格样本重置窗口。
# 返回的是是否存在足够长合格窗，不会补齐缺帧。
def stopped_window(samples, duration=1.0):
    window = []
    for sample in samples:
        if (not all(math.isfinite(v) for v in sample[3:6]) or abs(sample[3]) > .01
                or abs(sample[4]) > .05 or sample[5] > .2
                or (window and sample[0] - window[-1][0] > .1)):
            window = []
            continue
        window.append(sample)
        if sample[0] - window[0][0] >= duration:
            return True
    return False


# 要求至少两点、精确业务终点在容差内且相邻点非重合。
# 它只验证采样几何，不做地图障碍碰撞检查。
def valid_path(points, target):
    return (len(points) >= 2 and math.dist(points[-1], target) <= 1e-9
            and all(math.dist(a, b) > 1e-9 for a, b in zip(points, points[1:])))


# 检查窗口结束前最后一条命令为零、近期命令无非零尾巴，并报告停车观察段旋转峰值。
# 允许已发布零命令后输出静默，但缺少任何命令时不能通过。
def command_evidence(trace, goal_begin, window_begin, window_end):
    commands = [s for s in trace if goal_begin <= s[0] <= window_end and s[2] == '/cmd_vel']
    if not commands:
        return False, None
    # Collision Monitor 可能在发出零命令后静默，因此检查最后有效命令，而非伪造周期零帧。
    # 最后命令须为零，观察段内的终点旋转另行作为不合格证据。
    zero = abs(commands[-1][3]) < 1e-6 and abs(commands[-1][4]) < 1e-6
    tail = [s for s in commands if s[0] >= window_end - .1]
    zero = zero and all(abs(s[3]) < 1e-6 and abs(s[4]) < 1e-6 for s in tail)
    peak = max((abs(s[4]) for s in commands if s[0] >= window_begin), default=0.)
    return zero, peak


# 按样本六元前缀（墙钟、x、y、实测 v、实测 w、TF 年龄）评估完整停后窗口。
# 无穷／NaN、时段不足、采样缺口、定位误差或运动任一不满足都不能通过。
def evaluate(samples, target, tangent, duration=1.0):
    """samples: wall time, x, y, measured v, measured w, TF age seconds."""
    if any(not all(math.isfinite(v) for v in s[:6]) for s in samples):
        return {'passed': False, 'reason': 'nonfinite_sample'}
    if len(samples) < 2:
        return {"passed": False, "reason": "insufficient_samples"}
    errors = [math.hypot(s[1] - target[0], s[2] - target[1]) for s in samples]
    drift = max(math.hypot(s[1] - samples[0][1], s[2] - samples[0][2]) for s in samples)
    coverage = samples[-1][0] - samples[0][0]
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    fresh = all(0 <= s[5] <= 0.2 for s in samples)
    stopped = all(abs(s[3]) <= 0.01 and abs(s[4]) <= 0.05 for s in samples)
    dx, dy = samples[-1][1] - target[0], samples[-1][2] - target[1]
    return {"passed": coverage >= duration and max(gaps) <= 0.1 and fresh
            and stopped and max(errors) <= 0.010,
            "coverage_s": coverage, "max_gap_s": max(gaps), "tf_fresh": fresh,
            "stopped": stopped, "max_xy_error_m": max(errors), "final_xy_error_m": errors[-1],
            "drift_m": drift, "longitudinal_error_m": dx * tangent[0] + dy * tangent[1],
            "lateral_error_m": -dx * tangent[1] + dy * tangent[0]}


# 从首个进入速度门限的样本开始累积最早停后窗口。
# 初段出现缺帧不会通过挑选后面更好的窗口来遮蔽；时钟冻结也不能凑够完整观察时间。
def first_stopped_window(samples, duration=1.):
    window = []
    for s in samples:
        if abs(s[3]) > .01 or abs(s[4]) > .05:
            window = []
            continue
        if not all(math.isfinite(v) for v in s[:6]) or s[5] > .2 or (window and s[0] - window[-1][0] > .1):
            if window:
                return []
            window = []
            continue
        window.append(s)
        simulation_complete = len(s) < 9 or window[-1][8] - window[0][8] >= duration
        if window[-1][0] - window[0][0] >= duration and simulation_complete:
            return window
    return []


# 检查截止 end 前所有已观察位置是否有限且在 10 mm 半径内。
# 只约束可见样本，不证明未采样间隙内也准确。
def observed_position_valid(samples, target, end):
    observed = [s for s in samples if s[0] <= end]
    return bool(observed) and all(all(math.isfinite(v) for v in s[1:3]) and math.dist(s[1:3], target) <= .010 for s in observed)


# 统计达速持续时间、末端区进入时刻与低速蠕动积分。
# 相邻采样 gap>0.1 s 不计作连续巡航，实际速度在请求 ±5% 内保持至少 0.5 s 才算达速。
def motion_metrics(samples, begin, end, target, speed, creep):
    points = [s for s in samples if begin <= s[0] <= end]
    zone = max(.8, speed * speed / .5 + .1 * speed + .1)
    reached, cruise_start, cruise_max = False, None, 0.
    entry = None
    entry_sim = None
    crawl = 0.
    for a, b in zip(points, points[1:]):
        gap = b[0] - a[0]
        if entry is None and math.dist(a[1:3], target) <= zone:
            entry = a[0]
            entry_sim = a[8] if len(a) > 8 else None
        if gap > .1 or gap <= 0:
            cruise_start = None
            continue
        if .95 * speed <= abs(a[3]) <= 1.05 * speed and .95 * speed <= abs(b[3]) <= 1.05 * speed:
            if cruise_start is None:
                cruise_start = a[0]
            cruise_max = max(cruise_max, b[0] - cruise_start)
        else:
            cruise_start = None
        if .005 <= abs(a[3]) <= creep * 1.05 and gap <= .1:
            crawl += gap
    reached = cruise_max >= .5
    return {'speed_reached': reached, 'cruise_duration_s': cruise_max,
            'terminal_zone_m': zone, 'terminal_entry_wall': entry, 'terminal_entry_sim': entry_sim,
            'terminal_to_result_s': end - entry if entry is not None else None,
            'measured_crawl_s': crawl}
