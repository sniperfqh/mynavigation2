#!/usr/bin/env python3
"""Recompute acceptance from raw traces and render the complete speed matrix."""
import argparse
import bisect
import gzip
import hashlib
import json
import math
from pathlib import Path

from speed_sweep_metrics import command_evidence, evaluate, first_stopped_window, motion_metrics, observed_position_valid


def load(root, name):
    compressed = root / (name + '.json.gz')
    if compressed.exists():
        with gzip.open(compressed, 'rt') as stream:
            return json.load(stream)
    return json.loads((root / (name + '.json')).read_text())


def nearest(data, times, when):
    i = bisect.bisect_left(times, when)
    candidates = data[max(0, i - 1):min(len(data), i + 1)]
    if not candidates:
        return None
    sample = min(candidates, key=lambda s: abs(s[0] - when))
    return sample if abs(sample[0] - when) <= .05 else None


def audit_run(root):
    report = json.loads((root / 'results.json').read_text())
    poses = load(root, 'pose_trace')
    velocities = load(root, 'velocity_trace')
    truth = load(root, 'ground_truth')
    times = [s[0] for s in truth]
    cases = []
    for goal in report['goals']:
        post_file = root / f'post_{goal["index"]}.json'
        if not post_file.exists():
            cases.append({'index': goal['index'], 'passed': False, 'outcome': goal.get('outcome'), 'reason': 'no_complete_post_window'})
            continue
        post = json.loads(post_file.read_text())
        window = first_stopped_window(post)
        tangent = (0., 1. if goal['direction'] == 'forward' else -1.)
        precision = evaluate(window, goal['target'], tangent)
        position_valid = observed_position_valid(post, goal['target'], window[-1][0]) if window else False
        precision['passed'] = precision['passed'] and position_valid
        movement = motion_metrics(poses, goal['begin_wall'], goal['result_wall'], goal['target'], report['configuration']['speed'], report['configuration']['creep'])
        zero, peak = command_evidence(velocities, goal['begin_wall'], window[0][0], window[-1][0]) if window else (False, None)
        truth_errors = []
        for sample in window:
            physical = nearest(truth, times, sample[0])
            if physical is not None:
                truth_errors.append(math.dist(physical[4:6], goal['target']))
        passed = (goal.get('status') == 4 and goal.get('finish') is True and precision['passed']
                  and movement['speed_reached'] and zero and peak is not None and peak <= .05
                  and goal.get('path_geometry_valid') and not goal.get('unexpected_reversal'))
        cases.append({'index': goal['index'], 'direction': goal['direction'], 'passed': bool(passed),
                      'reported_passed': goal.get('accepted'), 'precision': precision, 'motion': movement,
                      'zero_output': zero, 'post_command_peak_w': peak,
                      'truth_samples': len(truth_errors),
                      'truth_max_error_m': max(truth_errors) if truth_errors else None})
    return {'root': str(root), 'cases': cases, 'matches_report': all(c['passed'] == c.get('reported_passed', False) for c in cases),
            'source_commit': report.get('source_commit'), 'deployed_library_sha256': report.get('deployed_library_sha256'),
            'amcl_parameters': report.get('amcl_parameters'), 'controller_parameters': report.get('active_parameters'),
            'smoother_parameters': report.get('smoother_parameters')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('progress', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    progress = json.loads(args.progress.read_text())
    audits = []
    for run in progress['runs']:
        root = Path(run['output'])
        if run['status'] != 'RUNNING' and (root / 'velocity_trace.json.gz').exists():
            audits.append(audit_run(root))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'raw_audit.json').write_text(json.dumps(audits, indent=2, allow_nan=False))
    lines = ['# fixed_path 停车效率正式验收', '', f'调度状态：{progress["status"]}。以完整原始停后窗口独立复算；精度为 map 定位，不等同物理精度。', '',
             '| 速度 m/s | 方向 | 基线/候选通过数 | 基线末端 P95 s | 候选末端 P95 s | 改善 | 定位最大误差 mm | 真值最大误差 mm | 结论 |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for index in range(1, 16):
        speed = f'{index / 10:.1f}'
        results = progress['speeds'].get(speed)
        if results is None:
            lines.append(f'| {speed} | 前/后 | — | — | — | — | — | — | 未完成 |')
            continue
        relevant = [a for a in audits if f'_{speed}_attempt' in Path(a['root']).name]
        for direction, title in [('forward', '前进'), ('reverse', '后退')]:
            result = results[direction]
            cases = [c for a in relevant for c in a['cases'] if c.get('direction') == direction]
            errors = [c['precision']['max_xy_error_m'] for c in cases if 'max_xy_error_m' in c.get('precision', {})]
            physical = [c['truth_max_error_m'] for c in cases if c.get('truth_max_error_m') is not None]
            audited = bool(cases) and all(c['passed'] for c in cases)
            number = lambda value, scale=1.: f'{value * scale:.2f}' if value is not None else '—'
            verdict = '通过' if result['passed'] and audited else '未通过'
            lines.append(f'| {speed} | {title} | {result["baseline_passed"]}/{result["candidate_passed"]} | {number(result["baseline_terminal_p95_s"])} | {number(result["candidate_terminal_p95_s"])} | {number(result["improvement_fraction"], 100)}% | {number(max(errors) if errors else None, 1000)} | {number(max(physical) if physical else None, 1000)} | {verdict} |')
    lines += ['', '真值按接收墙钟最近 50 ms 配对，只作诊断；原始桥接时间戳可能为零。未执行或中断的速度不标记通过。',
              '', f'原始调度：[progress.json]({args.progress.resolve()})；复算细节：[raw_audit.json](./raw_audit.json)。', '']
    (args.output / 'RESULTS.md').write_text('\n'.join(lines))
    print(json.dumps({'runs_audited': len(audits), 'audit_agreement': all(a['matches_report'] for a in audits), 'matrix': str(args.output / 'RESULTS.md')}))


if __name__ == '__main__':
    main()
