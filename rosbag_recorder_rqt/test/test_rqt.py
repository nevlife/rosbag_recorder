import math
import os

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257_main
from ament_xmllint.main import main as xmllint_main
import pytest
from python_qt_binding.QtCore import Qt
import rclpy
from rosbag_recorder_msgs.msg import AvailableTopic
from rosbag_recorder_msgs.msg import CheckResult
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.msg import TopicHealth
from rosbag_recorder_msgs.srv import Select
from rosbag_recorder_msgs.srv import SetTopics
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Status
from rosbag_recorder_rqt import rows

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def topic(name, transport='ros', group='', **fields):
    return AvailableTopic(
        name=name,
        transport=transport,
        group=group,
        type='custom_msgs/msg/Sample',
        publishers=1,
        **fields,
    )


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
    topics = [
        topic('/lidar/a', group='lidar'),
        topic('/lidar/b', 'flux', 'lidar'),
        topic('/custom/data', 'flux'),
    ]
    widget.on_topics(topics)
    group = widget.tree.topLevelItem(0)
    group.setCheckState(0, Qt.Checked)
    assert {(t.transport, t.name) for t in widget.selected_topics()} == {
        ('ros', '/lidar/a'),
        ('flux', '/lidar/b'),
    }
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
    widget.on_reply('status', Status.Response())
    assert widget.state.text() == 'no recorder' and widget.output.text() == ''
    health = Health(
        state=Health.PREVIEW,
        output_dir='/bags',
        disk_free_gb=50.0,
        remaining_min=math.nan,
        topics=[
            TopicHealth(
                name='/a/flux',
                source='flux',
                status=TopicHealth.OK,
                hz=12.0,
                age_s=0.1,
                written=10,
            )
        ],
    )
    health.stamp.sec = 1
    widget.on_reply('status', Status.Response(health=health))
    assert item.text(4) == '12.0' and item.text(6) == '10'
    assert widget.items[('ros', '/a')].text(6) == '-'
    assert widget.output.text() == 'Output: /bags'
    health.state = Health.RECORDING
    widget.on_reply('status', Status.Response(health=health))
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


def test_profile_checks_its_topics_and_saves_the_selection(widget, monkeypatch):
    from python_qt_binding.QtWidgets import QInputDialog
    from python_qt_binding.QtWidgets import QMessageBox

    calls = []
    monkeypatch.setattr(widget, 'call', lambda srv, name, **fields: calls.append((name, fields)))
    widget.on_reply('status', Status.Response(profiles=['e2e', 'lidar']))
    box = widget.profile_box
    assert [box.itemText(i) for i in range(box.count())] == ['', 'e2e', 'lidar']
    widget.on_topics([topic('/a'), topic('/b')])
    widget.items[('ros', '/b')].setCheckState(0, Qt.Checked)
    widget.choose_profile(1)
    assert calls[-1] == ('select', {'profile': 'e2e'})
    widget.on_reply('select', Select.Response(success=True))
    widget.on_topics([topic('/a', selected=True), topic('/b')])
    assert [t.name for t in widget.selected_topics()] == ['/a']
    widget.on_reply('status', Status.Response(profiles=['e2e', 'lidar'], profile='e2e'))
    assert box.currentText() == 'e2e'
    monkeypatch.setattr(QInputDialog, 'getText', lambda *args: ('mine', True))
    widget.save_profile()
    name, fields = calls[-1]
    assert (name, fields['profile'], fields['overwrite']) == ('save_profile', 'mine', False)
    assert [(t.transport, t.name) for t in fields['topics']] == [('ros', '/a')]
    monkeypatch.setattr(QInputDialog, 'getText', lambda *args: ('e2e', True))
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.No)
    widget.save_profile()
    assert calls[-1][0] == 'save_profile' and calls[-1][1]['profile'] == 'mine'
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.Yes)
    widget.save_profile()
    assert calls[-1][1]['profile'] == 'e2e' and calls[-1][1]['overwrite']


def test_storage_preset_follows_the_selection_until_chosen(widget, monkeypatch):
    calls = []
    monkeypatch.setattr(widget, 'call', lambda srv, name, **fields: calls.append((name, fields)))
    box = widget.preset_box
    assert [box.itemText(i) for i in range(box.count())] == [
        'none',
        'fastwrite',
        'zstd_fast',
        'zstd_small',
    ]
    assert box.currentText() == 'none'
    widget.on_reply('status', Status.Response(profile='z', storage_preset='zstd_fast'))
    assert box.currentText() == 'zstd_fast'
    box.setCurrentText('zstd_small')
    widget.choose_preset(box.currentIndex())
    widget.on_reply('status', Status.Response(profile='z', storage_preset='zstd_fast'))
    assert box.currentText() == 'zstd_small'
    widget.on_topics([topic('/a', selected=True)])
    widget.start()
    assert calls[-1] == ('start', {'storage_preset': 'zstd_small'})
    widget.on_reply('start', Start.Response(success=True))
    health = Health(state=Health.RECORDING, output_dir='/bags', remaining_min=math.nan)
    health.stamp.sec = 1
    widget.on_reply(
        'status', Status.Response(session_id='s', storage_preset='zstd_small', health=health)
    )
    assert widget.output.text() == 'Output: /bags   storage preset: zstd_small'
    assert box.currentText() == 'zstd_small' and not box.isEnabled()


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
