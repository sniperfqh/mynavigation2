import importlib.util
from pathlib import Path
import unittest
import tempfile

import numpy as np

spec = importlib.util.spec_from_file_location(
    'map_warehouse', Path(__file__).parents[1] / 'scripts' / 'map_warehouse.py')
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)
export_spec = importlib.util.spec_from_file_location(
    'export_slam_map', Path(__file__).parents[1] / 'scripts' / 'export_slam_map.py')
exporter = importlib.util.module_from_spec(export_spec)
export_spec.loader.exec_module(exporter)


class RouteSafetyTest(unittest.TestCase):
    def test_map_snapshot_preserves_fractional_origin_and_cells(self):
        from nav_msgs.msg import OccupancyGrid
        from PIL import Image
        import yaml
        msg = OccupancyGrid()
        msg.info.width = msg.info.height = 2
        msg.info.resolution = .01
        msg.info.origin.position.x = -6.959555341702842
        msg.info.origin.position.y = -10.416521352946756
        msg.info.origin.orientation.w = 1.
        msg.data = [0, -1, 100, 0]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            exporter.export_grid(msg, output)
            config = yaml.safe_load((output / 'myworld3.yaml').read_text())
            self.assertEqual(config['origin'][:2], [msg.info.origin.position.x,
                                                   msg.info.origin.position.y])
            self.assertTrue(np.array_equal(np.asarray(Image.open(output / 'myworld3.pgm')),
                                           [[0, 254], [254, 205]]))
            probability = 1. - np.asarray(Image.open(output / 'myworld3.pgm')).astype(float) / 255.
            decoded = np.where(probability > config['occupied_thresh'], 100,
                               np.where(probability < config['free_thresh'], 0, -1))
            self.assertTrue(np.array_equal(decoded[::-1], [[0, -1], [100, 0]]))

    def test_cannot_cut_between_touching_obstacles(self):
        free = np.array([[True, False], [False, True]])
        distance, parents = mapping.shortest_paths(free, (0, 0))
        self.assertFalse(np.isfinite(distance[1, 1]))
        self.assertNotIn((1, 1), parents)

    def test_blocked_start_has_no_routes(self):
        free = np.ones((4, 4), dtype=bool)
        free[1, 1] = False
        distance, parents = mapping.shortest_paths(free, (1, 1))
        self.assertFalse(np.isfinite(distance).any())
        self.assertEqual(parents, {})

    def test_route_stays_in_connected_free_region(self):
        free = np.ones((5, 5), dtype=bool)
        free[:, 2] = False
        distance, parents = mapping.shortest_paths(free, (2, 0))
        self.assertTrue(np.isfinite(distance[4, 1]))
        self.assertFalse(np.isfinite(distance[:, 3:]).any())
        cell = (4, 1)
        while cell != (2, 0):
            self.assertTrue(free[cell])
            cell = parents[cell]


if __name__ == '__main__':
    unittest.main()
