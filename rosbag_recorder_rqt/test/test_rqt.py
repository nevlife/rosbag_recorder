# Copyright 2026 pgw
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math
import os

from ament_copyright.main import main as copyright_main
from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257_main
from ament_xmllint.main import main as xmllint_main
import pytest
import rclpy
from rosbag_recorder_msgs.msg import CheckResult, Health, Marker, TopicHealth
from rosbag_recorder_msgs.srv import Status
from rosbag_recorder_rqt import rows

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def topic(name, status, **fields):
    t = TopicHealth(name=name, status=status, age_s=math.nan, **fields)
    return t


def test_rows():
    cells, color = rows.topic_row(topic('/a', TopicHealth.OK, hz=10.0, expected_hz=10.0,
                                        written=5, values=['delay 3 ms']))
    assert cells[0] == '/a' and cells[2] == 'ok' and cells[3] == '10.0' and cells[5] == '-'
    assert cells[-1] == 'delay 3 ms' and color is None
    cells, color = rows.topic_row(topic('/b', TopicHealth.ABNORMAL, required=True,
                                        issues=['bad']))
    assert cells[2] == 'value issue (required)' and cells[4] == '-' and color
    h = Health(state=Health.RECORDING, profile='p', session_id='s', elapsed_s=12.0,
               bag_bytes=3 * 1048576, write_mbps=1.5, disk_free_gb=100.0, remaining_min=math.nan,
               topics=[topic('/a', TopicHealth.OK), topic('/b', TopicHealth.LOW)])
    text = rows.summary(h)
    assert text.startswith('recording') and '1/2 topics ok' in text and '3 MiB' in text
    assert rows.check_line(CheckResult(level=CheckResult.WARN, text='x')) == '[warn] x'


@pytest.fixture
def widget():
    from python_qt_binding.QtWidgets import QApplication
    from rosbag_recorder_rqt.recorder import RecorderWidget
    app = QApplication.instance() or QApplication([])
    rclpy.init()
    node = rclpy.create_node('rosbag_recorder_rqt_test')
    w = RecorderWidget(node)
    yield w
    w.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    del app


def test_widget_shows_health_status_and_markers(widget):
    widget.show()
    widget.on_health(Health(state=Health.PREVIEW, disk_free_gb=50.0, remaining_min=math.nan,
                            topics=[topic('/a', TopicHealth.OK),
                                    topic('/b', TopicHealth.NO_PUBLISHER)]))
    assert widget.table.rowCount() == 2
    assert widget.table.item(1, 2).text() == 'no publisher'
    assert widget.state.text().startswith('preview')
    status = Status.Response(profiles=['a', 'b'], profile='b', message='selected b',
                             preflight=[CheckResult(level=CheckResult.OK, text='disk')])
    widget.on_reply('status', status)
    assert widget.profile.currentText() == 'b'
    assert widget.preflight.item(0).text() == '[ok] disk'
    widget.on_marker(Marker(kind='note', text='hello'))
    assert widget.markers.item(0).text().endswith('[note] hello')
    widget.call(Status, 'stop')
    assert 'not available' in widget.message.text()


def test_hidden_table_waits_and_markers_are_capped(widget):
    from python_qt_binding.QtWidgets import QHeaderView
    widget.show()
    widget.tabs.setCurrentIndex(2)
    widget.on_health(Health(remaining_min=math.nan, topics=[topic('/a', TopicHealth.OK)]))
    assert widget.table.rowCount() == 0
    widget.tabs.setCurrentIndex(0)
    assert widget.table.rowCount() == 1
    first = widget.table.item(0, 0)
    widget.on_health(Health(remaining_min=math.nan, topics=[topic('/a', TopicHealth.LOW)]))
    assert widget.table.item(0, 0) is first
    assert widget.table.item(0, 2).text() == 'low rate'
    header = widget.table.horizontalHeader()
    assert all(header.sectionResizeMode(c) != QHeaderView.ResizeToContents
               for c in range(widget.table.columnCount()))
    assert widget.table.item(0, 0).toolTip() == '/a'
    for i in range(600):
        widget.on_marker(Marker(kind='note', text=str(i)))
    assert widget.markers.count() == 500
    assert widget.markers.item(0).text().endswith('599')


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    assert copyright_main(argv=['.', 'test', '--exclude', 'setup.py']) == 0, 'Found errors'


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '\n'.join(errors)


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    assert pep257_main(argv=['.', 'test']) == 0, 'Found code style errors / warnings'


@pytest.mark.linter
@pytest.mark.xmllint
def test_xmllint():
    assert xmllint_main(argv=[]) == 0, 'Found errors'
