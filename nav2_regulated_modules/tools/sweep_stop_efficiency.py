#!/usr/bin/env python3
"""Progressive 0.1 m/s sweep; fail closed and retain every attempt."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time


def percentile95(values):
    ordered = sorted(values)
    return ordered[math.ceil(.95 * len(ordered)) - 1] if ordered else None


def compare(baseline, candidate, count=10):
    result = {}
    for direction in ('forward', 'reverse'):
        b = [g for g in baseline if g['direction'] == direction]
        c = [g for g in candidate if g['direction'] == direction]
        complete = len(b) == count and len(c) == count
        accepted = complete and all(g.get('accepted') for g in b + c)
        bt = percentile95([g['terminal_duration_s'] for g in b if g.get('terminal_duration_s') is not None])
        ct = percentile95([g['terminal_duration_s'] for g in c if g.get('terminal_duration_s') is not None])
        btotal = percentile95([g['total_duration_s'] for g in b if g.get('total_duration_s') is not None])
        ctotal = percentile95([g['total_duration_s'] for g in c if g.get('total_duration_s') is not None])
        gain = 1 - ct / bt if accepted and bt and ct is not None else None
        result[direction] = {'baseline_attempts': len(b), 'candidate_attempts': len(c),
                             'baseline_passed': sum(bool(g.get('accepted')) for g in b),
                             'candidate_passed': sum(bool(g.get('accepted')) for g in c),
                             'baseline_terminal_p95_s': bt, 'candidate_terminal_p95_s': ct,
                             'improvement_fraction': gain, 'baseline_total_p95_s': btotal,
                             'candidate_total_p95_s': ctotal,
                             'passed': bool(accepted and gain >= .20 and ctotal <= btotal)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--params-file', type=Path, required=True)
    parser.add_argument('--map-file', type=Path, required=True)
    parser.add_argument('--max-speed', type=float, default=1.5)
    parser.add_argument('--phase', choices=['screen', 'accept'], default='screen')
    parser.add_argument('--margin', type=float, default=.02)
    parser.add_argument('--creep', type=float, default=.03)
    parser.add_argument('--deceleration', type=float, default=.25)
    args = parser.parse_args()
    if not .1 <= args.max_speed <= 1.5 or abs(args.max_speed * 10 - round(args.max_speed * 10)) > 1e-9:
        parser.error('max-speed must be on the 0.1 m/s grid, up to 1.5')
    args.output.mkdir(parents=True, exist_ok=False)
    runner = Path(__file__).with_name('run_speed_case.py')
    progress = {'phase': args.phase, 'status': 'RUNNING', 'runs': [], 'speeds': {},
                'parameters': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}}

    def save():
        temporary = args.output / 'progress.tmp'
        temporary.write_text(json.dumps(progress, indent=2, allow_nan=False))
        temporary.replace(args.output / 'progress.json')

    def run(label, speed, repeats, margin, creep, deceleration, reject_ok=False, seed=3407):
        # Baseline stopping distance + acceleration and 0.5 s cruise reserves.
        length = max(2., math.ceil((speed * speed / .5 + .1 * speed + .1 + .5 * speed + .5) * 10) / 10)
        for retry in range(3):
            output = args.output / f'{label}_{speed:.1f}_attempt{retry}'
            command = [sys.executable, str(runner), '--output', str(output), '--params-file', str(args.params_file.resolve()),
                       '--map-file', str(args.map_file.resolve()), '--speed', str(speed), '--length', str(length),
                       '--repeats', str(repeats), '--stationary-seconds', '5', '--goal-timeout', '180',
                       '--tolerance', '0.01', '--stop-entry', '0.005', '--terminal-lookahead', '0.03', '--margin', str(margin),
                       '--creep', str(creep), '--deceleration', str(deceleration), '--seed', str(seed)]
            entry = {'label': label, 'speed': speed, 'length': length, 'output': str(output), 'command': command, 'status': 'RUNNING'}
            progress['runs'].append(entry)
            save()
            print('START', label, speed, 'repeat/direction', repeats, flush=True)
            with (args.output / f'{output.name}.stdout.log').open('w') as log:
                process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            report_file = output / 'results.json'
            report = json.loads(report_file.read_text()) if report_file.exists() else {'error': 'runner exited without report', 'goals': []}
            entry.update(status='PASSED' if process.returncode == 0 else 'FAILED', exit_code=process.returncode,
                         error=report.get('error'), attempts=len(report['goals']), passed=sum(bool(g.get('accepted')) for g in report['goals']))
            save()
            print('END', label, speed, entry['status'], entry['error'], flush=True)
            if process.returncode == 0:
                return report['goals']
            # Retry startup only; never erase an action/precision failure.
            if report['goals'] or retry == 2:
                if reject_ok and report['goals']:
                    return report['goals']
                raise RuntimeError(f'{label} {speed}: {report.get("error")}')
            time.sleep(2.)
        raise RuntimeError('unreachable')

    save()
    try:
        if args.phase == 'screen':
            baseline = run('baseline', .1, 5, .1, .01, .25)
            progress['screening'] = {'baseline': baseline, 'profiles': []}
            profiles = [(.05, .01, .25), (.02, .01, .25), (.1, .02, .25), (.1, .03, .25),
                        (.1, .01, .5), (.1, .01, .75), (.1, .01, 1.),
                        (args.margin, args.creep, args.deceleration)]
            for index, profile in enumerate(profiles):
                cases = run(f'candidate{index}', .1, 3, *profile, reject_ok=True)
                progress['screening']['profiles'].append({'profile': profile, 'cases': cases})
                save()
        else:
            for index in range(1, round(args.max_speed * 10) + 1):
                speed = index / 10.
                b, c = [], []
                # Two alternating blocks keep startup count bounded while avoiding
                # all-baseline-then-all-candidate order bias.
                for block in range(2):
                    order = ['baseline', 'candidate'] if block == 0 else ['candidate', 'baseline']
                    for name in order:
                        profile = (.1, .01, .25) if name == 'baseline' else (args.margin, args.creep, args.deceleration)
                        cases = run(f'{name}_block{block}', speed, 5, *profile, seed=3407 + block)
                        (b if name == 'baseline' else c).extend(cases)
                result = compare(b, c)
                progress['speeds'][f'{speed:.1f}'] = result
                save()
                if not all(r['passed'] for r in result.values()):
                    raise RuntimeError(f'{speed:.1f} m/s did not meet 20% efficiency/precision criteria')
        progress['status'] = 'COMPLETE'
    except (Exception, KeyboardInterrupt) as error:
        progress['status'] = 'STOPPED'
        progress['error'] = str(error)
        print('STOPPED', error, flush=True)
    finally:
        save()
    return 0 if progress['status'] == 'COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
