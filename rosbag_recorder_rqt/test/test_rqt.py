import math
import os

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257_main
from ament_xmllint.main import main as xmllint_main
import pytest
from python_qt_binding.QtCore import Qt
import rclpy
from rosbag_recorder_msgs.msg import AvailableTopic, CheckResult, Health, TopicHealth
from rosbag_recorder_msgs.srv import SetTopics
from rosbag_recorder_rqt import rows

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def topic(name, transport='ros', group='', **fields):
    return AvailableTopic(name=name, transport=transport, group=group,
                          type='custom_msgs/msg/Sample', publishers=1, **fields)


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


def test_category_selects_ros_and_flux_by_name(widget):
    topics = [topic('/lidar/a', group='lidar'), topic('/lidar/b', 'flux', 'lidar'),
              topic('/custom/data', 'flux')]
    widget.on_topics(topics)
    group = widget.tree.topLevelItem(0)
    group.setCheckState(0, Qt.Checked)
    assert {(t.transport, t.name) for t in widget.selected_topics()} == {
        ('ros', '/lidar/a'), ('flux', '/lidar/b')}
    widget.items[('ros', '/lidar/a')].setCheckState(0, Qt.Unchecked)
    assert group.checkState(0) == Qt.PartiallyChecked
    assert widget.tree.topLevelItem(1).text(0) == 'Uncategorized'
    widget.on_topics(topics + [topic('/new')])
    assert [(t.transport, t.name) for t in widget.selected_topics()] == [('flux', '/lidar/b')]


def test_refresh_preserves_selection_and_updates_statistics(widget):
    topics = [topic('/a', group='sensors'), topic('/a', 'flux', 'sensors')]
    widget.on_topics(topics)
    item = widget.items[('flux', '/a')]
    item.setCheckState(0, Qt.Checked)
    widget.on_topics(topics)
    assert item.checkState(0) == Qt.Checked
    health = Health(state=Health.PREVIEW, output_dir='/bags', disk_free_gb=50.0,
                    remaining_min=math.nan, topics=[TopicHealth(
                        name='/a/flux', source='flux', status=TopicHealth.OK,
                        hz=12.0, age_s=0.1, written=10)])
    widget.on_health(health)
    assert item.text(4) == '12.0' and item.text(6) == '10'
    assert widget.items[('ros', '/a')].text(6) == '-'
    assert widget.output.text() == 'Output: /bags'
    health.state = Health.RECORDING
    widget.on_health(health)
    assert not widget.tree.isEnabled() and not widget.start_button.isEnabled()
    assert rows.check_line(CheckResult(level=CheckResult.WARN, text='disk')) == '[warn] disk'


def test_start_applies_selection_before_recording(widget, monkeypatch):
    widget.on_topics([topic('/a')])
    widget.items[('ros', '/a')].setCheckState(0, Qt.Checked)
    calls = []
    monkeypatch.setattr(widget, 'call', lambda srv, name, **fields: calls.append((name, fields)))
    widget.start()
    assert calls[0][0] == 'set_topics'
    assert calls[0][1]['topics'][0].name == '/a'
    widget.on_reply('set_topics', SetTopics.Response(success=True))
    assert calls[1][0] == 'start'
    widget.items[('ros', '/a')].setCheckState(0, Qt.Unchecked)
    widget.start()
    assert len(calls) == 2
    assert widget.message.text() == 'Select at least one topic'


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
