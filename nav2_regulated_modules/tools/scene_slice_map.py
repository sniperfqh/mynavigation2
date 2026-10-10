#!/usr/bin/env python3
"""Experimental map from the unchanged scene's visual geometry at lidar height.

Uses installed numpy/Pillow, never simulator truth or synthetic scan inputs.
All inputs and transforms are recorded so ray residuals can validate the map.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import yaml

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def pose_matrix(text):
    x, y, z, roll, pitch, yaw = map(float, (text or '0 0 0 0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])
    result[:3, 3] = [x, y, z]
    return result


def mesh_triangles(path):
    root = ET.parse(path).getroot()
    unit = float(root.find('c:asset/c:unit', NS).get('meter', '1'))
    axis = root.findtext('c:asset/c:up_axis', default='Y_UP', namespaces=NS)
    axis_transform = np.eye(4)
    if axis == 'Y_UP':
        axis_transform[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    elif axis != 'Z_UP':
        raise ValueError('unsupported COLLADA up_axis ' + axis)
    geometries = {}
    for geometry in root.findall('c:library_geometries/c:geometry', NS):
        mesh = geometry.find('c:mesh', NS)
        sources = {}
        for source in mesh.findall('c:source', NS):
            accessor = source.find('c:technique_common/c:accessor', NS)
            stride = int(accessor.get('stride', '3'))
            values = np.fromstring(source.find('c:float_array', NS).text, sep=' ')
            sources[source.get('id')] = values.reshape(-1, stride)[:, :3]
        vertices = {v.get('id'): v.find("c:input[@semantic='POSITION']", NS).get('source')[1:] for v in mesh.findall('c:vertices', NS)}
        groups = []
        for primitive in mesh:
            tag = primitive.tag.rsplit('}', 1)[-1]
            if tag in ('source', 'vertices'):
                continue
            if tag not in ('triangles', 'polylist'):
                raise ValueError('unsupported mesh primitive ' + tag)
            if int(primitive.get('count', '0')) == 0:
                continue
            inputs = primitive.findall('c:input', NS)
            vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            stride = max(int(i.get('offset')) for i in inputs) + 1
            idx = np.fromstring(primitive.find('c:p', NS).text, dtype=int, sep=' ').reshape(-1, stride)[:, int(vertex.get('offset'))]
            positions = sources[vertices[vertex.get('source')[1:]]]
            if tag == 'triangles':
                groups.append(positions[idx.reshape(-1, 3)])
            else:
                offset = 0
                for count in map(int, primitive.find('c:vcount', NS).text.split()):
                    polygon = idx[offset:offset + count]
                    groups.append(positions[np.array([[polygon[0], polygon[i], polygon[i + 1]] for i in range(1, count - 1)])])
                    offset += count
        geometries[geometry.get('id')] = np.concatenate(groups)

    output = []

    def visit(node, parent):
        transform = parent.copy()
        for child in node:
            tag = child.tag.rsplit('}', 1)[-1]
            local = np.eye(4)
            if tag == 'matrix':
                local = np.fromstring(child.text, sep=' ').reshape(4, 4)
            elif tag == 'translate':
                local[:3, 3] = np.fromstring(child.text, sep=' ')
            elif tag == 'scale':
                local[:3, :3] = np.diag(np.fromstring(child.text, sep=' '))
            elif tag == 'rotate':
                values = np.fromstring(child.text, sep=' ')
                axis_vector = values[:3] / np.linalg.norm(values[:3])
                angle = math.radians(values[3])
                x, y, z = axis_vector
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                local[:3, :3] = np.eye(3) * math.cos(angle) + (1 - math.cos(angle)) * np.outer(axis_vector, axis_vector) + math.sin(angle) * skew
            else:
                continue
            transform = transform @ local
        for instance in node.findall('c:instance_geometry', NS):
            triangles = geometries[instance.get('url')[1:]]
            flat = triangles.reshape(-1, 3)
            points = np.c_[flat, np.ones(len(flat))] @ (axis_transform @ transform).T
            output.append((points[:, :3] * unit).reshape(-1, 3, 3))
        for nested in node.findall('c:node', NS):
            visit(nested, transform)

    for node in root.findall('c:library_visual_scenes/c:visual_scene/c:node', NS):
        visit(node, np.eye(4))
    if not output:
        raise ValueError('no mesh instances: ' + str(path))
    return np.concatenate(output)


def slice_edges(triangles, height):
    crossing = triangles[(triangles[:, :, 2].min(1) < height) & (triangles[:, :, 2].max(1) > height)]
    for triangle in crossing:
        points = []
        for a, b in ((triangle[0], triangle[1]), (triangle[1], triangle[2]), (triangle[2], triangle[0])):
            if (a[2] - height) * (b[2] - height) < 0:
                t = (height - a[2]) / (b[2] - a[2])
                points.append(a[:2] + t * (b[:2] - a[:2]))
        if len(points) == 2:
            yield points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--height', type=float, default=.335)
    parser.add_argument('--resolution', type=float, default=.01)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    world = ET.parse(args.world).getroot().find('world')
    models = args.world.parent / 'models'
    origin = (-7., -10.5)
    width, height = math.ceil(14.3 / args.resolution), math.ceil(21.15 / args.resolution)
    image = Image.new('L', (width, height), 254)
    draw = ImageDraw.Draw(image)
    evidence = {'world_sha256': hashlib.sha256(args.world.read_bytes()).hexdigest(), 'height_m': args.height, 'resolution_m': args.resolution, 'inputs': [], 'excluded_above_or_below_plane': []}
    cache = {}
    instances = [(include, np.eye(4)) for include in world.findall('include')]
    for wrapper in world.findall('model'):
        wrapper_pose = pose_matrix(wrapper.findtext('pose'))
        instances.extend((include, wrapper_pose) for include in wrapper.findall('include'))
        if wrapper.find('include') is None:
            for link in wrapper.findall('link'):
                for visual in link.findall('visual'):
                    transform = wrapper_pose @ pose_matrix(link.findtext('pose')) @ pose_matrix(visual.findtext('pose'))
                    geometry = visual.find('geometry')
                    box = geometry.find('box')
                    cylinder = geometry.find('cylinder')
                    if box is not None:
                        half = np.fromstring(box.findtext('size'), sep=' ') / 2
                        max_z = transform[2, 3] + np.sum(np.abs(transform[2, :3]) * half)
                    elif cylinder is not None and np.allclose(transform[2, :3], [0, 0, 1]):
                        max_z = transform[2, 3] + float(cylinder.findtext('length')) / 2
                    else:
                        raise ValueError('unsupported inline geometry: ' + wrapper.get('name'))
                    if max_z >= args.height:
                        raise ValueError('inline primitive intersects scan plane: ' + wrapper.get('name'))
                    evidence['excluded_above_or_below_plane'].append(wrapper.get('name') + '/' + visual.get('name'))
    for include, wrapper_pose in instances:
        model_name = include.findtext('uri').removeprefix('model://')
        if model_name == 'diffbot':
            continue
        model_path = models / model_name / 'model.sdf'
        model = ET.parse(model_path).getroot().find('model')
        base = wrapper_pose @ pose_matrix(include.findtext('pose')) @ pose_matrix(model.findtext('pose'))
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                geometry = visual.find('geometry')
                mesh = geometry.find('mesh')
                if mesh is None:
                    raise ValueError('primitive visual requires explicit support: ' + str(model_path))
                uri = mesh.findtext('uri')
                mesh_path = args.world.parent / uri.removeprefix('file://')
                if mesh_path not in cache:
                    cache[mesh_path] = mesh_triangles(mesh_path)
                    evidence['inputs'].append({'path': str(mesh_path), 'sha256': hashlib.sha256(mesh_path.read_bytes()).hexdigest()})
                triangles = cache[mesh_path]
                transform = base @ pose_matrix(link.findtext('pose')) @ pose_matrix(visual.findtext('pose'))
                scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
                flat = triangles.reshape(-1, 3) * scale
                transformed = (np.c_[flat, np.ones(len(flat))] @ transform.T)[:, :3].reshape(-1, 3, 3)
                if transformed[:, :, 2].min() >= args.height or transformed[:, :, 2].max() <= args.height:
                    evidence['excluded_above_or_below_plane'].append(include.findtext('name', model_name))
                for a, b in slice_edges(transformed, args.height):
                    pixel = lambda p: ((p[0] - origin[0]) / args.resolution, height - 1 - (p[1] - origin[1]) / args.resolution)
                    draw.line([pixel(a), pixel(b)], fill=0, width=1)
    if not cache or not any(v == 0 for v in image.getdata()):
        raise ValueError('empty geometry map; not suitable for localization')
    image.save(args.output / 'scene.pgm')
    metadata = {'image': 'scene.pgm', 'resolution': args.resolution, 'origin': [*origin, 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}
    (args.output / 'scene.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    (args.output / 'geometry_manifest.json').write_text(json.dumps(evidence, indent=2))
    print(json.dumps({'map': str(args.output / 'scene.yaml'), 'mesh_inputs': len(cache)}))


if __name__ == '__main__':
    main()
