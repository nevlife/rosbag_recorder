"""Exercise profile changes, start validation, value recovery and publisher QoS changes."""

import importlib.util
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
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.msg import TopicHealth
from rosbag_recorder_msgs.srv import Select
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Status
from rosbag_recorder_msgs.srv import Stop
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import yaml

ROOT = Path(tempfile.mkdtemp(prefix='recorder_lifecycle_'))


def recorder(*args, **kwargs):
    """Build the container of the installed launch file, with its remaps."""
    path = Path(get_package_share_directory('rosbag_recorder')) / 'launch' / 'recorder.launch.py'
    spec = importlib.util.spec_from_file_location('recorder_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.recorder(*args, **kwargs)


@pytest.mark.launch_test
def generate_test_description():
    """Launch the interactive recorder and two invalid autostart configurations."""
    (ROOT / 'profiles').mkdir()
    groups = {name: {'topics': {f'/lifecycle/{name}': {}}} for name in ('a', 'b')}
    groups['imu'] = {'topics': {'/lifecycle/imu': {'type': 'sensor_msgs/msg/Imu'}}}
    (ROOT / 'catalog.yaml').write_text(yaml.safe_dump({'groups': groups}))
    for name in ('a', 'b', 'imu', 'abstract', 'disk'):
        options = {'start_mode': 'immediate', 'disk_warn_gb': 0, 'disk_stop_gb': 0}
        if name == 'disk':
            options.update(disk_warn_gb=1e12, disk_stop_gb=1e12)
        profile = {
            'name': name,
            'groups': [name] if name in groups else [],
            'abstract': name == 'abstract',
            'options': options,
        }
        (ROOT / 'profiles' / f'{name}.yaml').write_text(yaml.safe_dump(profile))
    nodes = [recorder(overrides={'config_dir': str(ROOT), 'output_dir': str(ROOT / 'bags')})]
    for name in ('abstract', 'disk'):
        overrides = {
            'config_dir': str(ROOT),
            'output_dir': str(ROOT / name),
            'profile': name,
            'autostart': True,
        }
        nodes.append(recorder(name, overrides))
    return LaunchDescription([*nodes, launch_testing.actions.ReadyToTest()])


class TestLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('recorder_lifecycle_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(ROOT)

    def call(self, srv_type, name, prefix='/rosbag_recorder', **fields):
        client = self.node.create_client(srv_type, prefix + '/' + name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10), name)
            future = client.call_async(srv_type.Request(**fields))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
            self.assertTrue(future.done(), name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def until(self, predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail('condition did not become true')

    def test_1_profile_switch_and_immediate_stop(self):
        health = []
        sub = self.node.create_subscription(Health, '/rosbag_recorder/health', health.append, 10)
        try:
            self.assertTrue(self.call(Select, 'select', profile='a').success)
            self.until(lambda: health and health[-1].profile == 'a')
            selected = self.call(Select, 'select', profile='b')
            self.assertTrue(selected.success)
            self.assertFalse(any('/lifecycle/a' in c.text for c in selected.preflight))
            self.assertTrue(self.call(Start, 'start', force=True).success)
            stopped = self.call(Stop, 'stop')
            self.assertTrue(stopped.success)
            summary = yaml.safe_load((Path(stopped.bag_path) / 'summary.yaml').read_text())
            self.assertEqual(set(summary['topics']), {'/lifecycle/b'})
        finally:
            self.node.destroy_subscription(sub)

    def test_2_autostart_uses_start_validation(self):
        for name, text in [('abstract', 'is abstract'), ('disk', 'preflight:')]:
            status = self.call(Status, 'status', prefix='/' + name)
            self.assertEqual(status.state, Health.PREVIEW)
            self.assertIn(text, status.message)
            self.assertFalse(list((ROOT / name).glob('**/*.mcap')))
            response = self.call(Start, 'start', profile=name, force=True)
            self.assertFalse(response.success)
            self.assertIn(text, response.message)

    def test_3_zero_imu_values_are_recorded_without_semantic_issues(self):
        health = []
        health_sub = self.node.create_subscription(
            Health, '/rosbag_recorder/health', health.append, 10
        )
        publisher = self.node.create_publisher(Imu, '/lifecycle/imu', 10)
        self.assertTrue(self.call(Select, 'select', profile='imu').success)
        self.assertTrue(self.call(Start, 'start', force=True).success)
        timer = self.node.create_timer(0.05, lambda: publisher.publish(Imu()))
        try:
            self.until(lambda: health and health[-1].topics and health[-1].topics[0].written > 10)
            self.assertEqual(health[-1].topics[0].status, TopicHealth.OK)
            self.assertFalse(health[-1].topics[0].issues)
        finally:
            self.node.destroy_timer(timer)
            self.node.destroy_publisher(publisher)
            self.call(Stop, 'stop')
            self.node.destroy_subscription(health_sub)

    def test_4_reconnect_after_qos_change(self):
        health = []
        sub = self.node.create_subscription(Health, '/rosbag_recorder/health', health.append, 10)
        publisher = self.node.create_publisher(String, '/lifecycle/b', 10)
        self.assertTrue(self.call(Select, 'select', profile='b').success)
        self.assertTrue(self.call(Start, 'start', force=True).success)
        timer = self.node.create_timer(0.05, lambda: publisher.publish(String(data='qos')))

        def topic():
            return (
                next((t for t in health[-1].topics if t.name == '/lifecycle/b'), None)
                if health
                else None
            )

        try:
            self.until(lambda: topic() and topic().written > 0)
            before = topic().written
            self.node.destroy_publisher(publisher)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            publisher = self.node.create_publisher(String, '/lifecycle/b', qos)
            self.until(lambda: topic() and topic().written > before + 20)
            infos = self.node.get_subscriptions_info_by_topic('/lifecycle/b')
            infos = [i for i in infos if i.node_name == 'rosbag_recorder']
            self.assertEqual(len(infos), 1)
            self.assertEqual(infos[0].qos_profile.reliability, ReliabilityPolicy.BEST_EFFORT)
        finally:
            self.node.destroy_timer(timer)
            self.node.destroy_publisher(publisher)
            self.call(Stop, 'stop')
            self.node.destroy_subscription(sub)
