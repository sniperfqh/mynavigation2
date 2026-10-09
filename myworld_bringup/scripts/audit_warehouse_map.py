#!/usr/bin/env python3
"""Audit a laser-built map using held-out scans and diagnostic simulator poses."""
import argparse
import bisect
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt
import yaml


def load(root, name):
    with gzip.open(root / (name + '.json.gz'), 'rt') as stream:
        return json.load(stream)


def nearest(records, times, when, limit=.05):
    index = bisect.bisect_left(times, when)
    choices = records[max(0, index - 1):index + 1]
    sample = min(choices, key=lambda item: abs(item[0] - when)) if choices else None
    return sample if sample is not None and abs(sample[0] - when) <= limit else None


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mapping-dir', type=Path, required=True)
    parser.add_argument('--validation-dir', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(args.map.read_text())
    pixels = np.asarray(Image.open(args.map.parent / config['image']))
    occupied = pixels < 255 * (1 - config['occupied_thresh'])
    endpoint_distance = distance_transform_edt(~occupied) * config['resolution']
    known = pixels != 205
    resolution = config['resolution']
    ox, oy, origin_yaw = config['origin']
    if abs(origin_yaw) > 1e-9:
        raise ValueError('rotated image origin is not supported by this audit')
    build = load(args.mapping_dir, 'trajectory')
    build_truth = load(args.mapping_dir, 'truth')
    first = build[0]
    physical = nearest(build_truth, [p[0] for p in build_truth], first[0])
    if physical is None:
        raise ValueError('no independent pose aligned with first SLAM observation')
    rotation = first[4] - yaw(physical[4:8])
    c, s = math.cos(rotation), math.sin(rotation)
    transform = np.array([[c, -s], [s, c]])
    offset = np.array(first[2:4]) - transform @ np.array(physical[1:3])
    # One fixed calibration at mapping start. No per-scan re-registration.
    validation = json.loads((args.validation_dir / 'progress.json').read_text())
    scans = load(args.validation_dir, 'scans')
    truth = load(args.validation_dir, 'truth')
    times = [p[0] for p in truth]
    headings = [event for event in validation['events'] if event['event'] == 'validation_heading']
    cases, all_errors = [], []
    for event in headings:
        selected = [scan for scan in scans if event['wall'] + .5 <= scan['wall'] <= event['wall'] + 2.9]
        errors, endpoint_errors, eligible, missing, outside = [], [], 0, 0, 0
        for scan in selected:
            physical = nearest(truth, times, scan['wall'])
            if physical is None:
                raise ValueError('missing diagnostic pose in held-out observation')
            theta_world = yaw(physical[4:8])
            laser_world = np.array(physical[1:3]) + .18 * np.array([math.cos(theta_world), math.sin(theta_world)])
            laser = transform @ laser_world + offset
            theta = theta_world + rotation
            for index in range(len(scan['ranges'])):
                measured = scan['ranges'][index]
                if measured is None or not scan['range_min'] < measured < scan['range_max'] - .05:
                    continue
                eligible += 1
                angle = theta + scan['angle_min'] + index * scan['angle_increment']
                endpoint_col = math.floor((laser[0] + measured * math.cos(angle) - ox) / resolution)
                endpoint_row = pixels.shape[0] - 1 - math.floor((laser[1] + measured * math.sin(angle) - oy) / resolution)
                if 0 <= endpoint_col < pixels.shape[1] and 0 <= endpoint_row < pixels.shape[0]:
                    endpoint_errors.append(float(endpoint_distance[endpoint_row, endpoint_col]))
                else:
                    outside += 1
                    endpoint_errors.append(2.0)
                distance = np.arange(scan['range_min'], scan['range_max'], resolution / 2)
                col = np.floor((laser[0] + distance * math.cos(angle) - ox) / resolution).astype(int)
                row = pixels.shape[0] - 1 - np.floor((laser[1] + distance * math.sin(angle) - oy) / resolution).astype(int)
                inside = (col >= 0) & (col < pixels.shape[1]) & (row >= 0) & (row < pixels.shape[0])
                hits = np.flatnonzero(inside & occupied[np.clip(row, 0, pixels.shape[0] - 1), np.clip(col, 0, pixels.shape[1] - 1)])
                if len(hits):
                    errors.append(abs(distance[hits[0]] - measured))
                else:
                    missing += 1
        median = float(np.median(errors)) if errors else None
        p95 = float(np.percentile(errors, 95)) if errors else None
        missed = missing / eligible if eligible else 1.
        passed = bool(errors and median <= .05 and p95 <= .15 and missed <= .05)
        cases.append({'point': event['point'], 'stage': event['stage'], 'scans': len(selected),
                      'eligible_beams': eligible, 'matched_beams': len(errors), 'no_hit_fraction': missed,
                      'endpoint_median_m': float(np.median(endpoint_errors)) if endpoint_errors else None,
                      'endpoint_p95_m': float(np.percentile(endpoint_errors, 95)) if endpoint_errors else None,
                      'endpoint_out_of_bounds': outside,
                      'median_m': median, 'p95_m': p95, 'passed': passed})
        all_errors.extend(errors)
    positions = {tuple(round(value, 2) for value in event['point']) for event in headings}
    result = {'passed': validation['status'] == 'VALIDATED' and len(positions) >= 6 and len(cases) >= 18 and all(case['passed'] for case in cases),
              'positions': len(positions), 'headings': len(cases), 'cases': cases,
              'median_m': float(np.median(all_errors)) if all_errors else None,
              'p95_m': float(np.percentile(all_errors, 95)) if all_errors else None,
              'known_area_m2': float(known.sum() * resolution ** 2),
              'fixed_world_to_map': {'yaw': rotation, 'translation': offset.tolist()},
              'map_yaml_sha256': hashlib.sha256(args.map.read_bytes()).hexdigest(),
              'map_image_sha256': hashlib.sha256((args.map.parent / config['image']).read_bytes()).hexdigest(),
              'mapping_dir': str(args.mapping_dir), 'validation_dir': str(args.validation_dir)}
    (args.output / 'audit.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    preview = Image.fromarray(pixels.astype('uint8')).convert('RGB')
    draw = ImageDraw.Draw(preview)
    points = [(round((pose[2] - ox) / resolution), pixels.shape[0] - 1 - round((pose[3] - oy) / resolution)) for pose in build]
    draw.line(points, fill=(0, 130, 230), width=3)
    for event in headings:
        x, y = event['point'];px, py = (x - ox) / resolution, pixels.shape[0] - 1 - (y - oy) / resolution
        draw.ellipse((px - 10, py - 10, px + 10, py + 10), fill=(255, 70, 30))
    preview.save(args.output / 'coverage.png')
    print(json.dumps({k: result[k] for k in ('passed', 'positions', 'headings', 'median_m', 'p95_m')}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
