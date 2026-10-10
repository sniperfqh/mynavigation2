"""只读核查已有 S 曲线日志，按每帧的实际限制判定，不启动 ROS。"""
import json
import math
from pathlib import Path
import sys


def verify_log(path):
    axes = {}
    history = {}
    for line in Path(path).read_text().splitlines():
        if 'S-curve enabled:' in line:
            history.clear()
        if 'S-curve axis=' not in line:
            continue
        row = dict(token.split('=', 1) for token in line.split('S-curve ', 1)[1].split())
        row = {key: float(value) for key, value in row.items()}
        assert all(math.isfinite(value) for value in row.values()), 'Non-finite curve record'
        axis = int(row['axis'])
        dt = row['dt']
        assert dt > 0
        acceleration_limit = min(row['max_accel'], -row['max_decel'])
        jerk_limit = min(row['max_accel_jerk'], row['max_decel_jerk'])
        assert abs(row['accel']) <= acceleration_limit + 1e-6, row
        assert abs(row['jerk']) <= jerk_limit + 1e-5, row
        state = axes.setdefault(axis, dict(records=0, peak_accel=0.0, peak_jerk=0.0, limits=[]))
        state['records'] += 1
        state['peak_accel'] = max(state['peak_accel'], abs(row['accel']))
        state['peak_jerk'] = max(state['peak_jerk'], abs(row['jerk']))
        limits = [row[key] for key in ('max_accel', 'max_decel', 'max_accel_jerk', 'max_decel_jerk')]
        if limits not in state['limits']:
            state['limits'].append(limits)
        if axis in history:
            previous = history[axis]
            difference = (row['output'] - previous['output']) / dt
            # 速度、dt 以9位小数保存，差分校验须容许其可计算的舍入误差。
            tolerance = 1e-6 + 5e-9 / dt
            assert abs(difference) <= acceleration_limit + tolerance, row
            if 'difference' in previous:
                jerk_difference = (difference - previous['difference']) / dt
                tolerance = 1e-5 + 5e-9 / (dt * dt) + 5e-9 / (dt * previous['dt'])
                assert abs(jerk_difference) <= jerk_limit + tolerance, row
            row['difference'] = difference
        history[axis] = row
    assert axes, 'No S-curve records found'
    return dict(status='PASS', log_file=str(path), axes=axes, final_outputs={axis: row['output'] for axis, row in history.items()})


if __name__ == '__main__':
    print(json.dumps(verify_log(sys.argv[1]), indent=2))
