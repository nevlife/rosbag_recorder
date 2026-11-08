"""Overflow the writer cache: the summary must match the bag, not what the writer was handed."""

import array
import importlib.util
import os
from pathlib import Path
import shutil
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosbag_recorder_msgs.srv import Select
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Stop
from sensor_msgs.msg import PointCloud2
import yaml

OUT = tempfile.mkdtemp(prefix='rosbag_recorder_drop_')
TOPIC = '/test/burst'
BYTES = 1 << 20
SECONDS = 4.0


def recorder(*args, **kwargs):
    """Build the container of the installed launch file, with its remaps."""
    path = Path(get_package_share_directory('rosbag_recorder')) / 'launch' / 'recorder.launch.py'
    spec = importlib.util.spec_from_file_location('recorder_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.recorder(*args, **kwargs)


@pytest.mark.launch_test
def generate_test_description():
    """Start the recorder node."""
    data = os.path.join(os.environ['TEST_DATA_DIR'], 'cache_drop')
    return LaunchDescription(
        [
            recorder(overrides={'config_dir': data, 'output_dir': OUT}, output='screen'),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestCacheDrop(unittest.TestCase):

    def test_summary_matches_bag(self):
        rclpy.init()
        node = rclpy.create_node('cache_drop_test')
        self.addCleanup(shutil.rmtree, OUT)
        self.addCleanup(rclpy.shutdown)
        self.addCleanup(node.destroy_node)

        def call(srv_type, name, **fields):
            client = node.create_client(srv_type, '/rosbag_recorder/' + name)
            self.assertTrue(client.wait_for_service(timeout_sec=20.0), name)
            future = client.call_async(srv_type.Request(**fields))
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            node.destroy_client(client)
            return future.result()

        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        publisher = node.create_publisher(PointCloud2, TOPIC, qos)
        message = PointCloud2(height=1, width=BYTES // 4, point_step=4, row_step=BYTES)
        message.data = array.array('B', bytes(BYTES))
        self.assertTrue(call(Select, 'select', profile='burst').success)
        started = call(Start, 'start', label='burst')
        self.assertTrue(started.success, started.message)
        end = time.monotonic() + 15.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < end:
            time.sleep(0.05)
        end = time.monotonic() + SECONDS
        while time.monotonic() < end:
            publisher.publish(message)
        time.sleep(1.0)
        stopped = call(Stop, 'stop')
        self.assertTrue(stopped.success, stopped.message)

        with open(os.path.join(stopped.bag_path, 'summary.yaml')) as stream:
            summary = yaml.safe_load(stream)['topics'][TOPIC]
        with open(os.path.join(stopped.bag_path, 'metadata.yaml')) as stream:
            info = yaml.safe_load(stream)['rosbag2_bagfile_information']
        stored = next(
            t['message_count']
            for t in info['topics_with_message_count']
            if t['topic_metadata']['name'] == TOPIC
        )
        self.assertGreater(stored, 0)
        self.assertEqual(summary['written'], stored)
        self.assertGreaterEqual(summary['dropped'], 0)
