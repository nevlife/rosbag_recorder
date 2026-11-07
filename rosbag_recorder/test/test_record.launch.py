"""Record ROS topics (one of 4 MiB at 10 Hz), then read the bag back."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing.actions
import pytest
import rclpy
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.msg import Session
from rosbag_recorder_msgs.msg import TopicHealth
from rosbag_recorder_msgs.srv import Mark
from rosbag_recorder_msgs.srv import Select
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Status
from rosbag_recorder_msgs.srv import Stop
from std_srvs.srv import Trigger
import yaml

OUT = tempfile.mkdtemp(prefix='rosbag_recorder_test_')
SPLITS = os.environ.get('ROS_DISTRO') != 'humble'


def recorder(*args, **kwargs):
    """Build the container of the installed launch file, with its remaps."""
    path = Path(get_package_share_directory('rosbag_recorder')) / 'launch' / 'recorder.launch.py'
    spec = importlib.util.spec_from_file_location('recorder_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.recorder(*args, **kwargs)


@pytest.mark.launch_test
def generate_test_description():
    """Start the publishers and the recorder node."""
    here = os.path.dirname(os.path.abspath(__file__))
    data = os.path.join(os.environ['TEST_DATA_DIR'], 'launch')
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[sys.executable, os.path.join(here, 'publishers.py')], output='screen'
            ),
            recorder(overrides={'config_dir': data, 'output_dir': OUT}, output='screen'),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestRecord(unittest.TestCase):
    bag = None
    live = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rosbag_recorder_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(OUT)

    def call(self, srv_type, name, **fields):
        client = self.node.create_client(srv_type, '/rosbag_recorder/' + name)
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), name)
        future = client.call_async(srv_type.Request(**fields))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)
        self.node.destroy_client(client)
        return future.result()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_1_record(self):
        status = self.call(Status, 'status')
        self.assertIn('test', status.profiles)
        selected = self.call(Select, 'select', profile='test')
        self.assertTrue(selected.success, selected.message)
        started = self.call(Start, 'start', label='launch test')
        self.assertTrue(started.success, started.message)
        for _ in range(60):
            if self.call(Status, 'status').state == Health.RECORDING:
                break
            self.spin(0.5)
        self.assertEqual(self.call(Status, 'status').state, Health.RECORDING)

        healths = []
        sub = self.node.create_subscription(Health, '/rosbag_recorder/health', healths.append, 10)
        self.spin(5.0)
        self.node.destroy_subscription(sub)
        type(self).live = {t.name: t for t in healths[-1].topics}

        for topic in ('/test/points', '/test/chatter', '/tf_static'):
            infos = self.node.get_subscriptions_info_by_topic(topic)
            self.assertEqual([i.node_name for i in infos], ['rosbag_recorder'], topic)

        self.assertTrue(self.call(Mark, 'mark', text='hello').success)
        self.assertTrue(self.call(Trigger, 'pause').success)
        self.spin(1.0)
        self.assertTrue(self.call(Trigger, 'resume').success)
        split = self.call(Trigger, 'split')
        self.assertEqual(split.success, SPLITS, split.message)
        self.spin(3.0)
        stopped = self.call(Stop, 'stop')
        self.assertTrue(stopped.success, stopped.message)
        type(self).bag = stopped.bag_path

    def test_2_live_health(self):
        live = self.live
        self.assertEqual(live['/test/points'].status, TopicHealth.OK)
        self.assertGreater(live['/test/points'].size_kb, 4000)
        self.assertEqual(live['/test/chatter'].status, TopicHealth.OK)
        self.assertFalse(live['/test/imu'].values)
        self.assertEqual(live['/tf_static'].status, TopicHealth.OK)

    def test_3_bag_info(self):
        info = subprocess.run(
            ['ros2', 'bag', 'info', self.bag], capture_output=True, text=True, check=True
        ).stdout
        for topic in (
            '/test/points',
            '/test/chatter',
            '/test/imu',
            '/test/mode',
            '/tf_static',
            '/rosbag_recorder/session',
            '/rosbag_recorder/health',
            '/rosbag_recorder/markers',
        ):
            self.assertIn(topic, info)
        self.assertRegex(info, r'/tf_static \| Type: tf2_msgs/msg/TFMessage \| Count: 1 ')
        summary = yaml.safe_load(open(os.path.join(self.bag, 'summary.yaml')))
        self.assertEqual(summary['end_reason'], 'user')
        self.assertGreater(summary['topics']['/test/points']['written'], 40)
        self.assertTrue(any('hello' in m for m in summary['markers']))

    def test_4_dataset(self):
        try:
            import rosbag2_dataset as rd
        except ImportError:
            self.skipTest('rosbag2_dataset is not on PYTHONPATH')
        with rd.open(self.bag) as bag:
            sessions = [
                bag.topic('/rosbag_recorder/session')[i][1]
                for i in range(len(bag.topic('/rosbag_recorder/session')))
            ]
            self.assertEqual([s.event for s in sessions], [Session.START, Session.END])
            self.assertEqual(sessions[0].profile, 'test')
            self.assertEqual(sessions[0].label, 'launch test')
            self.assertIn('/test/points', sessions[0].profile_yaml)
            self.assertTrue(any('TF frames present' in c.text for c in sessions[0].preflight))
            self.assertEqual(sessions[1].end_reason, 'user')

            health = bag.topic('/rosbag_recorder/health')
            self.assertGreaterEqual(len(health), 8)
            last = {t.name: t for t in health[len(health) - 1][1].topics}
            self.assertGreater(last['/test/points'].written, 40)

            marks = bag.topic('/rosbag_recorder/markers')
            kinds = [marks[i][1].kind for i in range(len(marks))]
            texts = [marks[i][1].text for i in range(len(marks))]
            for kind in ('note', 'pause', 'resume') + ('split',) * SPLITS:
                self.assertIn(kind, kinds)
            self.assertIn('hello', texts)
            self.assertNotIn('mode', kinds)

            tf = bag.topic('/tf_static')
            self.assertEqual(tf[0][1].transforms[0].child_frame_id, 'base_link')
            points = bag.topic('/test/points')
            self.assertEqual(points[0][1].width, 262144)
