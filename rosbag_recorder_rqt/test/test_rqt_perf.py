# Copyright 2026 nevlife
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""GUI-thread cost with 500 long-named topics: startup, 1 Hz updates, tab switches, Mark."""

import math
import os
import random
import threading
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ['ROS_DOMAIN_ID'] = str(random.randint(150, 200))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rosbag_recorder_msgs.msg import Health, TopicHealth  # noqa: E402

TOPICS = int(os.environ.get('RQT_PERF_TOPICS', '500'))
SECONDS = float(os.environ.get('RQT_PERF_SECONDS', '30'))
UPDATE_P99_MS = float(os.environ.get('RQT_PERF_UPDATE_P99_MS', '50'))
LOOP_MAX_MS = float(os.environ.get('RQT_PERF_LOOP_MAX_MS', '100'))
STARTUP_S = float(os.environ.get('RQT_PERF_STARTUP_S', '2'))
REPORT = os.environ.get('RQT_PERF_REPORT')
LONG = 'very_long_namespace_segment_' * 6


def health(step):
    h = Health(state=Health.RECORDING, profile='scale', session_id='s', elapsed_s=float(step),
               disk_free_gb=100.0, remaining_min=math.nan)
    for i in range(TOPICS):
        h.topics.append(TopicHealth(
            name=f'/{LONG}{i}/points', type='sensor_msgs/msg/PointCloud2', source='',
            status=TopicHealth.ABNORMAL if (i + step) % 97 == 0 else TopicHealth.OK,
            required=i % 50 == 0, expected_hz=10.0, hz=10.0 + ((i + step) % 7) / 10.0,
            age_s=0.05, size_kb=1200.0, received=step * 10, written=step * 10, publishers=1,
            issues=['stamp delay mean 900 ms (max 1200)'] if (i + step) % 97 == 0 else [],
            values=['delay 3 ms', f'{120000 + i} points']))
    return h


def percentile(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * p))]


@pytest.fixture
def setup():
    from python_qt_binding.QtWidgets import QApplication
    from rosbag_recorder_rqt import recorder
    app = QApplication.instance() or QApplication([])
    rclpy.init()
    node = rclpy.create_node('rosbag_recorder_rqt_perf')
    source = rclpy.create_node('rosbag_recorder_perf_source')
    publisher = source.create_publisher(Health, '/rosbag_recorder/health', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    yield app, recorder, node, publisher
    executor.shutdown()
    source.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def test_many_topics(setup):
    app, recorder, node, publisher = setup
    cost = []
    original = recorder.RecorderWidget.on_health

    def timed(self, h):
        start = time.perf_counter()
        original(self, h)
        cost.append((time.perf_counter() - start) * 1000.0)

    recorder.RecorderWidget.on_health = timed
    try:
        created = time.monotonic()
        widget = recorder.RecorderWidget(node)
        widget.resize(1600, 900)
        widget.show()
        first = health(0)
        while widget.table.rowCount() < TOPICS or not widget.table.item(TOPICS - 1, 0):
            assert time.monotonic() - created < 10.0
            publisher.publish(first)
            end = time.monotonic() + 0.1
            while time.monotonic() < end:
                app.processEvents()
                time.sleep(0.005)
        startup_s = time.monotonic() - created
        cost.clear()

        lateness = []
        clicks = []
        tabs = getattr(widget, 'tabs', None)
        stop = threading.Event()

        def feed():
            step = 1
            while not stop.wait(1.0):
                publisher.publish(health(step))
                step += 1

        feeder = threading.Thread(target=feed, daemon=True)
        feeder.start()
        start = time.monotonic()
        last = start
        n = 0
        while time.monotonic() - start < SECONDS:
            app.processEvents()
            now = time.monotonic()
            lateness.append((now - last) * 1000.0)
            last = now
            if n % 100 == 50:
                t = time.perf_counter()
                if tabs is not None:
                    tabs.setCurrentIndex(n // 100 % (tabs.count() - 1) + 1)
                    app.processEvents()
                    tabs.setCurrentIndex(0)
                widget.mark_text.setText('look')
                widget.mark()
                app.processEvents()
                clicks.append((time.perf_counter() - t) * 1000.0)
            n += 1
            time.sleep(0.002)
        stop.set()
        feeder.join()
        widget.shutdown()
    finally:
        recorder.RecorderWidget.on_health = original

    lines = [
        f'topics {TOPICS}, startup to filled table {startup_s:.2f} s',
        f'update ms n {len(cost)} p50 {percentile(cost, 0.5):.1f} '
        f'p99 {percentile(cost, 0.99):.1f} max {max(cost):.1f}',
        f'event loop gap ms p99 {percentile(lateness, 0.99):.1f} max {max(lateness):.1f}',
        f'tab switch + Mark ms max {max(clicks):.1f}',
    ]
    print('\n'.join(lines))
    if REPORT:
        with open(REPORT, 'w') as f:
            f.write('\n'.join(lines) + '\n')
    assert startup_s < STARTUP_S
    assert percentile(cost, 0.99) < UPDATE_P99_MS
    assert max(lateness) < LOOP_MAX_MS
    assert max(clicks) < LOOP_MAX_MS
