#!/usr/bin/env python3
# ooii: costmap timing statistics tool (companion to the "costmap_timing" /
# "obstacle_timing" debug logs added by the ooii instrumentation).
"""Summarise Nav2 costmap timing logs for before/after comparisons.

The costmap stack writes one structured line per update cycle (spdlog DEBUG, so
it lands in the log file):

  [..] costmap_timing name=local_costmap update=0.001817 publish=0.003910
        cpu=0.001001 cycle=0.200103 lock=0.000002 origin=0.000032 reset=0.000044
        layer.voxel_layer.bounds=0.000900 layer.voxel_layer.costs=0.000120 ...

and one line per obstacle layer update, split per observation source:

  [..] obstacle_timing name=voxel_layer fetch_mark=0.000012 fetch_clear=0.000010
        scan.mark=0.000400 scan.clear=0.000300 scan.age=0.083000 ...

Usage:
  costmap_timing_stats.py planner_server.log
  costmap_timing_stats.py controller_server.log --name local_costmap --period 0.2
  costmap_timing_stats.py before.log after.log --compare --name local_costmap

All durations are printed in milliseconds.  Only cycles after the costmap is
initialised are reported; the first cycle of a run is usually much slower.
"""

import argparse
import csv
import math
import re
import sys
from collections import defaultdict
from datetime import datetime

TIMESTAMP_RE = re.compile(r'^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\]')
TOKEN_RE = re.compile(r'([A-Za-z0-9_.]+)=([^\s]+)')

# Durations that are already seconds in the log.
SECONDS_KEYS = {'update', 'publish', 'cpu', 'cycle', 'lock', 'origin', 'reset'}


def _is_duration_key(key):
    if key in SECONDS_KEYS:
        return True
    return (key.startswith('layer.') and (key.endswith('.bounds') or key.endswith('.costs'))) \
        or key.endswith('.mark') or key.endswith('.clear') or key.endswith('.age') \
        or key.startswith('fetch_')


def parse_time(text):
    """Accept 'HH:MM:SS[.fff]' or a full ISO timestamp."""
    text = text.strip()
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        pass
    for fmt in ('%H:%M:%S.%f', '%H:%M:%S', '%H:%M'):
        try:
            return datetime.strptime(text, fmt)
        except ValueError:
            continue
    raise argparse.ArgumentTypeError(f'cannot parse time: {text!r}')


def in_window(ts, start, end):
    if ts is None:
        return start is None and end is None
    # Compare only the time of day when the filter has no date component.
    if start is not None:
        if start.year == 1900:
            if ts.time() < start.time():
                return False
        elif ts < start:
            return False
    if end is not None:
        if end.year == 1900:
            if ts.time() > end.time():
                return False
        elif ts > end:
            return False
    return True


def parse_logs(paths):
    costmap_rows = []
    obstacle_rows = []
    for path in paths:
        with open(path, 'r', errors='replace') as handle:
            for line in handle:
                if 'costmap_timing ' in line:
                    kind = 'costmap'
                elif 'obstacle_timing ' in line:
                    kind = 'obstacle'
                else:
                    continue

                ts_match = TIMESTAMP_RE.match(line)
                ts = None
                if ts_match:
                    try:
                        ts = datetime.fromisoformat(ts_match.group(1))
                    except ValueError:
                        ts = None

                metrics = {}
                name = ''
                for key, value in TOKEN_RE.findall(line):
                    if key == 'name':
                        name = value
                        continue
                    try:
                        number = float(value)
                    except ValueError:
                        continue
                    metrics[key] = number

                row = {'ts': ts, 'name': name, 'metrics': metrics, 'file': path}
                if kind == 'costmap':
                    costmap_rows.append(row)
                else:
                    obstacle_rows.append(row)
    return costmap_rows, obstacle_rows


def percentile(sorted_values, q):
    if not sorted_values:
        return float('nan')
    # Nearest-rank percentile, index 1..n.
    rank = max(1, math.ceil(q * len(sorted_values)))
    return sorted_values[min(rank, len(sorted_values)) - 1]


def summarise(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return None
    mean = sum(ordered) / n
    variance = sum((value - mean) ** 2 for value in ordered) / n
    return {
        'n': n,
        'mean': mean,
        'std': math.sqrt(variance),
        'p50': percentile(ordered, 0.50),
        'p90': percentile(ordered, 0.90),
        'p95': percentile(ordered, 0.95),
        'p99': percentile(ordered, 0.99),
        'max': ordered[-1],
    }


def collect(rows, prefix_skip=()):
    """Return {metric_key: [values in ms]} for one costmap/obstacle group."""
    series = defaultdict(list)
    for row in rows:
        for key, value in row['metrics'].items():
            if key in prefix_skip:
                continue
            if not _is_duration_key(key):
                continue
            series[key].append(value * 1000.0)
    return series


def print_table(title, series, rows, period, show_share):
    if not series:
        print(f'{title}: no samples')
        return
    print(f'\n== {title}  (cycles={len(rows)})')
    header = f"{'metric':38s} {'n':>6s} {'mean':>9s} {'std':>8s} {'p50':>9s} {'p90':>9s} {'p95':>9s} {'p99':>9s} {'max':>9s}  (ms)"
    print(header)
    print('-' * len(header))
    for key in sorted(series, key=lambda k: -summarise(series[k])['mean']):
        stats = summarise(series[key])
        print(f"{key:38s} {stats['n']:6d} {stats['mean']:9.3f} {stats['std']:8.3f} "
              f"{stats['p50']:9.3f} {stats['p90']:9.3f} {stats['p95']:9.3f} "
              f"{stats['p99']:9.3f} {stats['max']:9.3f}")

    update = summarise(series.get('update', []))
    if update is not None and period is not None:
        over = [row['metrics']['update'] for row in rows
                if 'update' in row['metrics'] and row['metrics']['update'] > period]
        publish_values = series.get('publish', [])
        publish_over = [row['metrics']['publish'] for row in rows
                        if 'publish' in row['metrics'] and row['metrics']['publish'] > period]
        cycle_values = [row['metrics']['cycle'] for row in rows
                        if 'cycle' in row['metrics'] and row['metrics']['cycle'] > period * 1.05]
        print(f"\nperiod={period * 1000.0:.1f}ms  "
              f"update overruns={len(over)}  publish overruns={len(publish_over)}  "
              f"cycle>1.05*period={len(cycle_values)}  "
              f"update budget used: mean={update['mean'] / (period * 1000.0) * 100:.2f}% "
              f"p95={update['p95'] / (period * 1000.0) * 100:.2f}% "
              f"max={update['max'] / (period * 1000.0) * 100:.2f}%")

    if show_share and update is not None and update['mean'] > 0:
        print('\nmean share of update:')
        parts = []
        for key in sorted(series):
            if not key.startswith('layer.'):
                continue
            stats = summarise(series[key])
            parts.append((stats['mean'], key))
        for mean, key in sorted(parts, reverse=True):
            print(f'  {key:38s} {mean:8.3f} ms  {mean / update["mean"] * 100:5.1f}%')
        accounted = sum(mean for mean, _ in parts)
        other = update['mean'] - accounted
        print(f'  {"(pose/footprint/overhead)":38s} {other:8.3f} ms  {other / update["mean"] * 100:5.1f}%')


def print_comparison(title, base_series, compare_series):
    print(f'\n== {title}  baseline vs compare (ms)')
    header = (f"{'metric':38s} {'p50 base':>9s} {'p50 new':>9s} {'d%':>7s} "
              f"{'p95 base':>9s} {'p95 new':>9s} {'d%':>7s} "
              f"{'max base':>9s} {'max new':>9s}")
    print(header)
    print('-' * len(header))
    keys = sorted(set(base_series) | set(compare_series))
    for key in keys:
        base = summarise(base_series.get(key, []))
        new = summarise(compare_series.get(key, []))
        if base is None or new is None:
            continue

        def delta(old, replacement):
            if old == 0:
                return float('inf') if replacement else 0.0
            return (replacement - old) / old * 100.0

        print(f"{key:38s} {base['p50']:9.3f} {new['p50']:9.3f} {delta(base['p50'], new['p50']):7.1f} "
              f"{base['p95']:9.3f} {new['p95']:9.3f} {delta(base['p95'], new['p95']):7.1f} "
              f"{base['max']:9.3f} {new['max']:9.3f}")


def write_csv(path, rows):
    keys = sorted({key for row in rows for key in row['metrics']})
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['timestamp', 'name'] + [f'{key}_ms' if _is_duration_key(key) else key
                                                 for key in keys])
        for row in rows:
            values = []
            for key in keys:
                value = row['metrics'].get(key)
                if value is None:
                    values.append('')
                elif _is_duration_key(key):
                    values.append(f'{value * 1000.0:.6f}')
                else:
                    values.append(value)
            writer.writerow([row['ts'].isoformat() if row['ts'] else '', row['name']] + values)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logs', nargs='+', help='planner_server.log / controller_server.log')
    parser.add_argument('--name', default=None,
                        help='costmap/obstacle layer name to select, e.g. local_costmap or voxel_layer')
    parser.add_argument('--period', type=float, default=None,
                        help='expected update period in seconds (0.2 local, 1.0 global); '
                             'default: median cycle time')
    parser.add_argument('--start', type=parse_time, default=None, help='HH:MM:SS or ISO timestamp')
    parser.add_argument('--end', type=parse_time, default=None, help='HH:MM:SS or ISO timestamp')
    parser.add_argument('--compare', action='store_true',
                        help='treat the first log as baseline and print deltas for the second')
    parser.add_argument('--csv', default=None, help='write the selected costmap cycles to CSV')
    args = parser.parse_args()

    costmap_rows, obstacle_rows = parse_logs(args.logs)
    costmap_rows = [row for row in costmap_rows if in_window(row['ts'], args.start, args.end)]
    obstacle_rows = [row for row in obstacle_rows if in_window(row['ts'], args.start, args.end)]

    names = sorted({row['name'] for row in costmap_rows})
    obstacle_names = sorted({row['name'] for row in obstacle_rows})

    def select(rows, wanted):
        if wanted is None:
            return rows
        return [row for row in rows if row['name'] == wanted]

    def resolve(wanted, available, label):
        if wanted is None:
            if len(available) > 1:
                print(f'multiple {label} names found ({", ".join(available)}); use --name',
                      file=sys.stderr)
                sys.exit(2)
            return available[0] if available else None
        return wanted if wanted in available else None

    # --name is shared: it may name a costmap (local_costmap) or an obstacle
    # layer (voxel_layer / obstacle_layer); only the matching group is reported.
    costmap_name = resolve(args.name, names, 'costmap')
    obstacle_name = resolve(args.name, obstacle_names, 'obstacle layer')
    if args.name is not None and costmap_name is None and obstacle_name is None:
        print(f'--name {args.name!r} not found; costmaps: {", ".join(names) or "none"}; '
              f'obstacle layers: {", ".join(obstacle_names) or "none"}', file=sys.stderr)
        sys.exit(2)

    if args.name is None:
        selected_costmap = costmap_rows
        selected_obstacle = obstacle_rows
    else:
        # Report only the group the requested name belongs to.
        selected_costmap = select(costmap_rows, costmap_name) if costmap_name else []
        selected_obstacle = select(obstacle_rows, obstacle_name) if obstacle_name else []

    if not selected_costmap and not selected_obstacle:
        print('no timing lines found; is the costmap file log level at debug/trace?', file=sys.stderr)
        sys.exit(1)

    period = args.period
    if period is None:
        cycles = [row['metrics']['cycle'] for row in selected_costmap
                  if row['metrics'].get('cycle', 0.0) > 0.0]
        if cycles:
            period = summarise(cycles)['p50']
            print(f'inferred period: {period:.4f}s (median cycle)')

    def report(costmap_rows_subset, obstacle_rows_subset, label):
        if costmap_rows_subset:
            series = collect(costmap_rows_subset)
            print_table(f'costmap_timing name={costmap_name} {label}', series,
                        costmap_rows_subset, period, show_share=True)
        if obstacle_rows_subset:
            series = collect(obstacle_rows_subset)
            print_table(f'obstacle_timing name={obstacle_name} {label}', series,
                        obstacle_rows_subset, None, show_share=False)

    if args.compare:
        if len(args.logs) != 2:
            parser.error('--compare expects exactly two log files')
        base_costmap = [row for row in selected_costmap if row['file'] == args.logs[0]]
        new_costmap = [row for row in selected_costmap if row['file'] == args.logs[1]]
        base_obstacle = [row for row in selected_obstacle if row['file'] == args.logs[0]]
        new_obstacle = [row for row in selected_obstacle if row['file'] == args.logs[1]]
        if base_costmap and new_costmap:
            print_comparison(f'costmap_timing name={costmap_name}',
                             collect(base_costmap), collect(new_costmap))
        if base_obstacle and new_obstacle:
            print_comparison(f'obstacle_timing name={obstacle_name}',
                             collect(base_obstacle), collect(new_obstacle))
    else:
        # Files may be rotated segments of the same run: merge them.
        report(selected_costmap, selected_obstacle, '')

    if args.csv:
        write_csv(args.csv, selected_costmap)
        print(f'\nwrote {len(selected_costmap)} costmap cycles to {args.csv}')


if __name__ == '__main__':
    main()
