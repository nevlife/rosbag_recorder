"""Keep a large topic selection intact while statistics and discovery change."""

import math
import os

from python_qt_binding.QtCore import Qt
import rclpy
from rosbag_recorder_msgs.msg import AvailableTopic
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.msg import TopicHealth

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def test_many_topics():
    from python_qt_binding.QtWidgets import QApplication
    from rosbag_recorder_rqt.recorder import RecorderWidget

    app = QApplication.instance() or QApplication([])
    rclpy.init()
    node = rclpy.create_node('rosbag_recorder_large_selection')
    widget = RecorderWidget(node)
    topics = [
        AvailableTopic(
            name=f'/long_namespace_for_sensor_data/topic_{i}',
            transport='flux' if i % 2 else 'ros',
            group='sensors',
            type='custom_msgs/msg/Sample',
        )
        for i in range(500)
    ]
    try:
        widget.on_topics(topics)
        widget.tree.topLevelItem(0).setCheckState(0, Qt.Checked)
        items = dict(widget.items)
        for count in range(5):
            health = Health(state=Health.PREVIEW, remaining_min=math.nan)
            for topic in topics:
                health.topics.append(
                    TopicHealth(
                        name=topic.name + ('/flux' if topic.transport == 'flux' else ''),
                        source='flux' if topic.transport == 'flux' else '',
                        status=TopicHealth.OK,
                        hz=10.0,
                        age_s=0.01,
                        written=count,
                    )
                )
            widget.on_health(health)
            widget.on_topics(topics)
            app.processEvents()
            assert len(widget.selected_topics()) == 500
            assert widget.items == items
            assert all(item.text(6) == str(count) for item in widget.items.values())
        widget.on_topics(topics + [AvailableTopic(name='/new', transport='ros')])
        assert len(widget.items) == 501 and len(widget.selected_topics()) == 500
        assert widget.items[('ros', '/new')].checkState(0) == Qt.Unchecked
    finally:
        widget.shutdown()
        widget.close()
        node.destroy_node()
        rclpy.shutdown()
