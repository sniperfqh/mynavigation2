"""Check nested TF stamps without changing frame names or transform values."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


spec = importlib.util.spec_from_file_location(
    'tf_system_time', Path(__file__).parents[1] / 'scripts' / 'tf_system_time.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TfSystemTimeTest(unittest.TestCase):
    def test_all_nested_stamps_and_transform_preservation(self):
        message = TFMessage()
        for index in range(2):
            transform = TransformStamped()
            transform.header.stamp = Time(sec=index + 1)
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_' + str(index)
            transform.transform.translation.x = index + 0.25
            transform.transform.rotation.w = 1.0
            message.transforms.append(transform)
        before = [(t.header.frame_id, t.child_frame_id, str(t.transform))
                  for t in message.transforms]
        stamp = Time(sec=1790000000, nanosec=123456789)
        adapter = SimpleNamespace(
            system_clock=SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
            publisher=Mock())
        module.TfSystemTime.on_tf(adapter, message)
        self.assertTrue(all(t.header.stamp == stamp for t in message.transforms))
        self.assertEqual(before, [(t.header.frame_id, t.child_frame_id, str(t.transform))
                                  for t in message.transforms])
        adapter.publisher.publish.assert_called_once_with(message)

    def test_empty_message(self):
        adapter = SimpleNamespace(
            system_clock=SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
            publisher=Mock())
        message = TFMessage()
        module.TfSystemTime.on_tf(adapter, message)
        adapter.publisher.publish.assert_called_once_with(message)


if __name__ == '__main__':
    unittest.main()
