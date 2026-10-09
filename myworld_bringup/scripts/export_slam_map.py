#!/usr/bin/env python3
"""Export one atomic SLAM grid with full origin precision."""
import argparse
import hashlib
import json
import os
import signal
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from slam_toolbox.srv import DeserializePoseGraph, Pause


def export_grid(msg, output):
    info = msg.info
    q = info.origin.orientation
    import math
    angle = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
    pixels = np.where(grid < 0, 205, np.where(grid >= 50, 0, 254)).astype('uint8')[::-1]
    Image.fromarray(pixels).save(output / 'myworld3.pgm')
    config = {'image': 'myworld3.pgm', 'mode': 'trinary', 'resolution': float(info.resolution),
              'origin': [info.origin.position.x, info.origin.position.y, angle],
              'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}
    (output / 'myworld3.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    saved = yaml.safe_load((output / 'myworld3.yaml').read_text())
    if not np.array_equal(np.asarray(Image.open(output / 'myworld3.pgm')), pixels):
        raise RuntimeError('map pixels changed during export')
    if max(abs(a - b) for a, b in zip(saved['origin'], config['origin'])) > 1e-9:
        raise RuntimeError('map origin precision lost')
    probability = 1. - pixels.astype(float) / 255.
    decoded = np.where(probability > saved['occupied_thresh'], 100,
                       np.where(probability < saved['free_thresh'], 0, -1))
    expected = np.where(grid < 0, -1, np.where(grid >= 50, 100, 0))[::-1]
    if not np.array_equal(decoded, expected):
        raise RuntimeError('map export changed unknown/free/occupied semantics')
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--posegraph', type=Path, required=True)
    parser.add_argument('--params-file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = '177'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    log = (args.output / 'slam.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'slam_toolbox', 'sync_slam_toolbox_node',
                                '--ros-args', '--params-file', str(args.params_file.resolve()),
                                '-p', 'use_sim_time:=false'], stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    rclpy.init()
    node = Node('precise_map_export')

    def call(service, type_, request):
        client = node.create_client(type_, service)
        if not client.wait_for_service(timeout_sec=30.):
            raise RuntimeError('service unavailable: ' + service)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60.)
        if not future.done() or future.result() is None:
            raise RuntimeError('service failed: ' + service)
        response = future.result()
        node.destroy_client(client)
        return response

    try:
        request = DeserializePoseGraph.Request()
        request.filename = str(args.posegraph.resolve())
        request.match_type = request.START_AT_FIRST_NODE
        call('/slam_toolbox/deserialize_map', DeserializePoseGraph, request)
        call('/slam_toolbox/pause_new_measurements', Pause, Pause.Request())
        msg = call('/slam_toolbox/dynamic_map', GetMap, GetMap.Request()).map
        if not msg.info.width or not msg.info.height:
            raise RuntimeError('loaded posegraph yielded an empty map')
        config = export_grid(msg, args.output)
        manifest = {'source_posegraph': str(args.posegraph), 'origin': config['origin'],
                    'width': msg.info.width, 'height': msg.info.height,
                    'pixel_roundtrip_exact': True, 'origin_roundtrip_error_m': 0.,
                    'posegraph_sha256': hashlib.sha256(Path(str(args.posegraph) + '.posegraph').read_bytes()).hexdigest()}
        (args.output / 'export.json').write_text(json.dumps(manifest, indent=2))
        print(json.dumps(manifest, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10.)
        log.close()


if __name__ == '__main__':
    main()
