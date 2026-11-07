"""Select ROS and raw Flux topics, record them together, and verify their bytes."""

import gc
import importlib.util
from pathlib import Path
import shutil
import struct
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import flux
from flux_msgs.msg import FluxFrame
from launch import LaunchDescription
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.msg import TopicSelection
from rosbag_recorder_msgs.srv import ListTopics
from rosbag_recorder_msgs.srv import SetTopics
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Stop
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = Path(tempfile.mkdtemp(prefix='recorder_selection_'))
FINGERPRINT = 0xACDC1234


def recorder(*args, **kwargs):
    """Build the container of the installed launch file, with its remaps."""
    path = Path(get_package_share_directory('rosbag_recorder')) / 'launch' / 'recorder.launch.py'
    spec = importlib.util.spec_from_file_location('recorder_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.recorder(*args, **kwargs)


@pytest.mark.launch_test
def generate_test_description():
    """Launch with a category defined by topic name, independent of transport and type."""
    (ROOT / 'profiles').mkdir()
    (ROOT / 'catalog.yaml').write_text(
        'groups:\n  sensors:\n    topics:\n      /selection/data: {}\n'
    )
    container = recorder(overrides={'config_dir': str(ROOT), 'output_dir': str(ROOT / 'bags')})
    return LaunchDescription([container, launch_testing.actions.ReadyToTest()])


class TestSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('recorder_selection_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(ROOT)

    def call(self, kind, name, **fields):
        client = self.node.create_client(kind, '/rosbag_recorder/' + name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(kind.Request(**fields))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
            self.assertTrue(future.done())
            return future.result()
        finally:
            self.node.destroy_client(client)

    def until(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('condition did not become true')

    def test_mixed_selection_pause_and_raw_round_trip(self):
        health = []
        sub = self.node.create_subscription(Health, '/rosbag_recorder/health', health.append, 10)
        ros = self.node.create_publisher(String, '/selection/data', 10)
        raw = flux.Publisher(
            self.node, '/selection/data', fingerprint=FINGERPRINT, slot_size=4096, slot_count=4
        )
        other = flux.Publisher(self.node, '/selection/other', slot_size=4096, slot_count=4)
        phase = [1]

        def publish():
            ros.publish(String(data=str(phase[0])))
            raw.publish(np.array([phase[0], 17, 23], dtype=np.uint8))
            other.publish(np.array([99], dtype=np.uint8))

        timer = self.node.create_timer(0.03, publish)
        selected = [TopicSelection(name='/selection/data', transport=t) for t in ('ros', 'flux')]
        try:
            self.until(lambda: len(self.call(ListTopics, 'topics').topics) >= 3)
            topics = self.call(ListTopics, 'topics').topics
            self.assertEqual(
                {t.transport for t in topics if t.name == '/selection/data'}, {'ros', 'flux'}
            )
            self.assertTrue(
                all(t.group == 'sensors' for t in topics if t.name == '/selection/data')
            )
            self.assertEqual(next(t.group for t in topics if t.name == '/selection/other'), '')
            self.assertTrue(self.call(SetTopics, 'set_topics', topics=selected).success)
            self.assertFalse(
                self.call(
                    SetTopics,
                    'set_topics',
                    topics=[TopicSelection(name='/selection/data', transport='invalid')],
                ).success
            )
            self.assertTrue(self.call(Start, 'start').success)
            self.assertFalse(self.call(SetTopics, 'set_topics', topics=[]).success)
            self.until(
                lambda: (
                    health
                    and len(health[-1].topics) == 2
                    and all(t.written > 10 for t in health[-1].topics)
                )
            )
            self.assertTrue(self.call(Trigger, 'pause').success)
            phase[0] = 2
            deadline = time.monotonic() + 1.3
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            before = {t.name: t.written for t in health[-1].topics}
            phase[0] = 3
            self.assertTrue(self.call(Trigger, 'resume').success)
            self.until(lambda: all(t.written > before[t.name] + 10 for t in health[-1].topics))
            stopped = self.call(Stop, 'stop')
            self.assertTrue(stopped.success)
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=stopped.bag_path),
                rosbag2_py.ConverterOptions('cdr', 'cdr'),
            )
            names = {t.name: t.type for t in reader.get_all_topics_and_types()}
            self.assertEqual(names['/selection/data'], 'std_msgs/msg/String')
            self.assertEqual(names['/selection/data/flux'], 'flux_msgs/msg/FluxFrame')
            self.assertNotIn('/selection/other/flux', names)
            frames, strings = [], []
            while reader.has_next():
                name, data, _ = reader.read_next()
                if name == '/selection/data/flux':
                    frames.append(deserialize_message(data, FluxFrame))
                elif name == '/selection/data':
                    strings.append(deserialize_message(data, String).data)
            self.assertTrue(frames)
            self.assertEqual({f.fingerprint for f in frames}, {FINGERPRINT})
            self.assertEqual({f.flux_topic for f in frames}, {'/selection/data'})
            self.assertEqual({f.codec for f in frames}, {'raw'})
            self.assertEqual({bytes(f.data)[0] for f in frames}, {1, 3})
            self.assertEqual(set(strings), {'1', '3'})
            self.assertTrue(all(bytes(f.data)[1:] == bytes([17, 23]) for f in frames))
            self.assertEqual(struct.unpack('<II', bytes(frames[0].meta[:8])), (4096, 4))
            self.assertTrue(self.call(SetTopics, 'set_topics', topics=[]).success)
            self.assertFalse(self.call(Start, 'start').success)
        finally:
            self.node.destroy_timer(timer)
            self.call(Stop, 'stop')
            raw = other = None
            gc.collect()
            self.node.destroy_publisher(ros)
            self.node.destroy_subscription(sub)

    def test_flux_publisher_reconnects_with_new_slot_geometry(self):
        health = []
        sub = self.node.create_subscription(Health, '/rosbag_recorder/health', health.append, 10)
        publishers = [
            flux.Publisher(
                self.node,
                '/selection/restart',
                fingerprint=FINGERPRINT,
                slot_size=4096,
                slot_count=4,
            )
        ]
        phase = [1]
        timer = self.node.create_timer(
            0.03,
            lambda: publishers[0] and publishers[0].publish(np.array([phase[0]], dtype=np.uint8)),
        )
        try:
            self.until(
                lambda: any(
                    t.name == '/selection/restart' for t in self.call(ListTopics, 'topics').topics
                )
            )
            selection = TopicSelection(name='/selection/restart', transport='flux')
            self.assertTrue(self.call(SetTopics, 'set_topics', topics=[selection]).success)
            self.assertTrue(self.call(Start, 'start').success)
            self.until(lambda: health and health[-1].topics and health[-1].topics[0].written > 5)
            before = health[-1].topics[0].written
            publishers[0] = None
            gc.collect()
            self.until(lambda: health and health[-1].topics[0].publishers == 0)
            entry = next(
                t for t in self.call(ListTopics, 'topics').topics if t.name == '/selection/restart'
            )
            self.assertTrue(entry.selected)
            self.assertEqual(entry.publishers, 0)
            phase[0] = 2
            publishers[0] = flux.Publisher(
                self.node,
                '/selection/restart',
                fingerprint=FINGERPRINT,
                slot_size=8192,
                slot_count=4,
            )
            self.until(lambda: health[-1].topics[0].written > before + 5)
            stopped = self.call(Stop, 'stop')
            self.assertTrue(stopped.success)
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=stopped.bag_path),
                rosbag2_py.ConverterOptions('cdr', 'cdr'),
            )
            sizes, values = set(), set()
            while reader.has_next():
                name, data, _ = reader.read_next()
                if name == '/selection/restart/flux':
                    frame = deserialize_message(data, FluxFrame)
                    sizes.add(struct.unpack('<I', bytes(frame.meta[:4]))[0])
                    values.add(bytes(frame.data)[0])
            self.assertEqual(sizes, {4096, 8192})
            self.assertEqual(values, {1, 2})
            publishers[0] = None
            gc.collect()
            self.until(lambda: health[-1].topics[0].publishers == 0)
            self.assertTrue(self.call(SetTopics, 'set_topics', topics=[selection]).success)
            started = self.call(Start, 'start')
            self.assertTrue(started.success)
            publishers[0] = flux.Publisher(
                self.node,
                '/selection/restart',
                fingerprint=FINGERPRINT,
                slot_size=4096,
                slot_count=4,
            )
            self.until(
                lambda: (
                    health[-1].session_id == started.session_id
                    and health[-1].topics[0].written > 5
                )
            )
            self.assertTrue(self.call(Stop, 'stop').success)
            self.assertTrue(self.call(SetTopics, 'set_topics', topics=[]).success)
        finally:
            self.node.destroy_timer(timer)
            self.call(Stop, 'stop')
            publishers.clear()
            gc.collect()
            self.node.destroy_subscription(sub)
