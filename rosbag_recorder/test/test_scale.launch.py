# Copyright 2026 nevlife
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A 500-topic profile: tick cost in preview and recording, memory over minutes."""

import os
import shutil
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import pytest
import rclpy
from rosbag_recorder_msgs.msg import Health, TopicHealth
from rosbag_recorder_msgs.srv import Select, Start, Status, Stop

TOPICS = int(os.environ.get('SCALE_TOPICS', '500'))
RECORD_S = float(os.environ.get('SCALE_RECORD_S', '150'))
# Time budgets leave room for slow cores; graph reads per tick are the precise check.
TICK_P50_MS = float(os.environ.get('SCALE_TICK_P50_MS', '15'))
TICK_P99_MS = float(os.environ.get('SCALE_TICK_P99_MS', '40'))
GRAPH_READ_SHARE = float(os.environ.get('SCALE_GRAPH_READ_SHARE', '0.5'))
STATUS_P50_MS = float(os.environ.get('SCALE_STATUS_P50_MS', '20'))
RSS_GROWTH_MB = float(os.environ.get('SCALE_RSS_GROWTH_MB', '20'))
DIR = tempfile.mkdtemp(prefix='rosbag_recorder_scale_')
REPORT = os.environ.get('SCALE_REPORT')


def write_config():
    groups = []
    for g in range(0, TOPICS, 50):
        topics = ''.join(
            f'      /scale/t{i}: {{type: sensor_msgs/msg/Imu, hz: 10.0'
            f'{", required: true" if i % 50 == 0 else ""}}}\n'
            for i in range(g, min(g + 50, TOPICS)))
        groups.append(f'  g{g // 50}:\n    topics:\n{topics}')
    os.makedirs(os.path.join(DIR, 'config', 'profiles'))
    with open(os.path.join(DIR, 'config', 'catalog.yaml'), 'w') as f:
        f.write('version: 1\ngroups:\n' + ''.join(groups))
    names = ', '.join(f'g{g // 50}' for g in range(0, TOPICS, 50))
    with open(os.path.join(DIR, 'config', 'profiles', 'scale.yaml'), 'w') as f:
        f.write(f'name: scale\ngroups: [{names}]\n'
                'options: {start_mode: immediate, split_duration_sec: 0, disk_warn_gb: 0.2, '
                'disk_stop_gb: 0.1}\n')


def percentile(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * p))]


@pytest.mark.launch_test
def generate_test_description():
    write_config()
    recorder = Node(package='rosbag_recorder', executable='rosbag_recorder_node', output='screen',
                    parameters=[{'config_dir': os.path.join(DIR, 'config'),
                                 'output_dir': os.path.join(DIR, 'out')}])
    publishers = Node(package='rosbag_recorder', executable='many_publishers', output='screen',
                      parameters=[{'count': TOPICS, 'rate': 10.0}])
    return LaunchDescription([recorder, publishers, launch_testing.actions.ReadyToTest()]), {
        'recorder': recorder}


class TestScale(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rosbag_recorder_scale_client')
        cls.healths = []
        cls.clients = {}
        cls.node.create_subscription(Health, '/rosbag_recorder/health', cls.healths.append, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        shutil.rmtree(DIR)

    def call(self, srv_type, name, **fields):
        # One client per service, as the rqt plugin keeps them: a new client is a graph change.
        if name not in self.clients:
            self.clients[name] = self.node.create_client(srv_type, '/rosbag_recorder/' + name)
        client = self.clients[name]
        self.assertTrue(client.wait_for_service(timeout_sec=20.0), name)
        start = time.monotonic()
        future = client.call_async(srv_type.Request(**fields))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)
        return future.result(), (time.monotonic() - start) * 1000.0

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def rss_mb(self, pid):
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('VmRSS:'):
                    return int(line.split()[1]) / 1024.0
        return 0.0

    def test_scale(self, proc_info, recorder):
        pid = proc_info[recorder].pid
        selected, select_ms = self.call(Select, 'select', profile='scale')
        self.assertTrue(selected.success, selected.message)
        start = time.monotonic()
        ok = 0
        while time.monotonic() - start < 120.0 and ok < TOPICS:
            self.spin(1.0)
            if self.healths:
                ok = sum(t.status == TopicHealth.OK for t in self.healths[-1].topics)
        all_ok_s = time.monotonic() - start
        self.assertEqual(ok, TOPICS)

        self.healths.clear()
        status_ms = []
        for _ in range(20):
            status_ms.append(self.call(Status, 'status')[1])
            self.spin(0.5)
        preview_ticks = [h.tick_ms for h in self.healths]
        graph_share = ((self.healths[-1].graph_reads - self.healths[0].graph_reads) /
                       max(1, len(self.healths) - 1))

        started, _ = self.call(Start, 'start', label='scale')
        self.assertTrue(started.success, started.message)
        self.healths.clear()
        rss = []
        end = time.monotonic() + RECORD_S
        while time.monotonic() < end:
            self.spin(5.0)
            rss.append(self.rss_mb(pid))
        record_ticks = [h.tick_ms for h in self.healths]
        stopped, stop_ms = self.call(Stop, 'stop')
        self.assertTrue(stopped.success, stopped.message)

        warm = rss[len(rss) // 3:]
        third = max(1, len(warm) // 3)
        growth = sum(warm[-third:]) / third - sum(warm[:third]) / third
        lines = [
            f'topics {TOPICS}, select {select_ms:.0f} ms, all ok after {all_ok_s:.1f} s',
            f'preview tick ms p50 {percentile(preview_ticks, 0.5):.2f} '
            f'p99 {percentile(preview_ticks, 0.99):.2f} max {max(preview_ticks):.2f}',
            f'graph read in {graph_share:.0%} of preview ticks',
            f'status call ms p50 {percentile(status_ms, 0.5):.1f} '
            f'p99 {percentile(status_ms, 0.99):.1f}',
            f'record tick ms p50 {percentile(record_ticks, 0.5):.2f} '
            f'p99 {percentile(record_ticks, 0.99):.2f} max {max(record_ticks):.2f}',
            f'rss MB start {rss[0]:.0f} end {rss[-1]:.0f} growth after warm-up {growth:.1f}',
            f'stop {stop_ms:.0f} ms',
        ]
        print('\n'.join(lines))
        if REPORT:
            with open(REPORT, 'w') as f:
                f.write('\n'.join(lines) + '\n')
        self.assertLess(graph_share, GRAPH_READ_SHARE)
        self.assertLess(percentile(preview_ticks, 0.5), TICK_P50_MS)
        self.assertLess(percentile(preview_ticks, 0.99), TICK_P99_MS)
        self.assertLess(percentile(record_ticks, 0.99), TICK_P99_MS)
        self.assertLess(percentile(status_ms, 0.5), STATUS_P50_MS)
        self.assertLess(growth, RSS_GROWTH_MB)
