import sys

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QBrush
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QHeaderView
from python_qt_binding.QtWidgets import QInputDialog
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QMessageBox
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QTreeWidget
from python_qt_binding.QtWidgets import QTreeWidgetItem
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.msg import TopicSelection
from rosbag_recorder_msgs.srv import ListTopics
from rosbag_recorder_msgs.srv import SaveProfile
from rosbag_recorder_msgs.srv import Select
from rosbag_recorder_msgs.srv import SetTopics
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Status
from rosbag_recorder_msgs.srv import Stop
from rqt_gui_py.plugin import Plugin
from std_srvs.srv import Trigger

from . import rows

PREFIX = '/rosbag_recorder/'
COLUMNS = ('Topic', 'Transport', 'Type', 'Status', 'Hz', 'Age s', 'Written', 'Publishers')
WIDTHS = (360, 80, 210, 130, 70, 70, 90, 80)
PRESETS = ('none', 'fastwrite', 'zstd_fast', 'zstd_small')


class RecorderWidget(QWidget):
    """Select ROS and Flux topics by name, and show recording statistics."""

    replied = Signal(str, object)

    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('Rosbag Recorder')
        self.node = node
        self.clients = {}
        self.pending = set()
        self.items = {}
        self.profiles = []
        self.signature = None
        self.last_health = None
        self.start_after_selection = False
        self.dirty = False
        self.preset_chosen = False
        self.build()
        self.replied.connect(self.on_reply)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(1000)

    def build(self):
        self.tree = QTreeWidget()
        self.tree.setColumnCount(len(COLUMNS))
        self.tree.setHeaderLabels(COLUMNS)
        header = self.tree.header()
        header.setSectionResizeMode(QHeaderView.Interactive)
        for column, width in enumerate(WIDTHS):
            header.resizeSection(column, width)
        header.setStretchLastSection(True)
        self.tree.itemChanged.connect(self.selection_changed)
        self.state = QLabel('no recorder')
        self.output = QLabel('')
        self.message = QLabel('')
        self.checks = QLabel('')
        self.checks.setWordWrap(True)
        controls = QHBoxLayout()
        self.profile_box = QComboBox()
        self.profile_box.setMinimumContentsLength(16)
        self.profile_box.addItem('')
        self.profile_box.activated.connect(self.choose_profile)
        self.save_button = QPushButton('Save profile')
        self.save_button.clicked.connect(self.save_profile)
        controls.addWidget(QLabel('Profile'))
        controls.addWidget(self.profile_box)
        controls.addWidget(self.save_button)
        self.preset_box = QComboBox()
        self.preset_box.addItems(PRESETS)
        self.preset_box.setToolTip('MCAP storage preset; fastwrite writes no index')
        self.preset_box.activated.connect(self.choose_preset)
        controls.addWidget(QLabel('Storage preset'))
        controls.addWidget(self.preset_box)
        self.preview_button = QPushButton('Preview selection')
        self.preview_button.clicked.connect(self.select)
        self.start_button = QPushButton('Start')
        self.start_button.clicked.connect(self.start)
        controls.addWidget(self.preview_button)
        controls.addWidget(self.start_button)
        for title, service in (
            ('Stop', 'stop'),
            ('Pause', 'pause'),
            ('Resume', 'resume'),
            ('Split', 'split'),
        ):
            button = QPushButton(title)
            button.clicked.connect(
                lambda _, name=service: self.call(Stop if name == 'stop' else Trigger, name)
            )
            controls.addWidget(button)
        controls.addStretch()
        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.state)
        layout.addWidget(self.output)
        layout.addWidget(self.message)
        layout.addWidget(self.checks)
        layout.addWidget(self.tree)

    def poll(self):
        if self.isVisible():
            self.call(ListTopics, 'topics')
            self.call(Status, 'status')

    def call(self, srv_type, name, **fields):
        if name in self.pending:
            return
        client = self.clients.get(name)
        if client is None:
            client = self.clients[name] = self.node.create_client(srv_type, PREFIX + name)
        if not client.service_is_ready():
            if name not in ('status', 'topics'):
                self.message.setText(f'{PREFIX}{name} is not available')
                self.start_after_selection = False
            return
        self.pending.add(name)
        future = client.call_async(srv_type.Request(**fields))
        future.add_done_callback(lambda f: self.replied.emit(name, f.result()))

    def selected_topics(self):
        return [
            TopicSelection(name=name, transport=transport)
            for (transport, name), item in self.items.items()
            if item.checkState(0) == Qt.Checked
        ]

    def selection_changed(self, item, column):
        if column == 0:
            self.dirty = True

    def select(self):
        self.start_after_selection = False
        self.call(SetTopics, 'set_topics', topics=self.selected_topics())

    def choose_profile(self, index):
        self.preset_chosen = False
        if index > 0:
            self.call(Select, 'select', profile=self.profiles[index - 1])

    def choose_preset(self, _):
        self.preset_chosen = True

    def request_start(self):
        self.call(Start, 'start', storage_preset=self.preset_box.currentText())

    def save_profile(self):
        topics = self.selected_topics()
        if not topics:
            self.message.setText('Select at least one topic')
            return
        name, ok = QInputDialog.getText(
            self, 'Save profile', 'Profile name (lowercase letters, digits and _):'
        )
        if not ok or not name:
            return
        overwrite = name in self.profiles
        if overwrite:
            answer = QMessageBox.question(self, 'Save profile', f'Replace profile {name}?')
            if answer != QMessageBox.Yes:
                return
        self.call(SaveProfile, 'save_profile', profile=name, topics=topics, overwrite=overwrite)

    def start(self):
        topics = self.selected_topics()
        if not topics:
            self.message.setText('Select at least one topic')
            return
        if not self.dirty:
            self.request_start()
            return
        self.start_after_selection = True
        self.call(SetTopics, 'set_topics', topics=topics)

    def on_reply(self, name, response):
        self.pending.discard(name)
        if response is None:
            return
        if name == 'topics':
            if response.success:
                self.on_topics(response.topics)
            else:
                self.message.setText(response.message)
            return
        if name == 'status':
            self.show_checks(response.preflight)
            self.show_profiles(response.profiles, response.profile)
            # Until the user picks a preset, the box shows the selection's.
            if not self.preset_chosen and response.storage_preset in PRESETS:
                self.preset_box.setCurrentText(response.storage_preset)
            session_preset = response.storage_preset if response.session_id else ''
            if response.health.stamp.sec or response.health.stamp.nanosec:
                self.on_health(response.health, session_preset)
            return
        if hasattr(response, 'preflight'):
            self.show_checks(response.preflight)
        self.message.setText(f'{name}: {response.message}')
        if name == 'start' and response.success:
            self.preset_chosen = False
        if name == 'select' and response.success:
            # The topic list then checks exactly the profile's topics.
            self.dirty = False
            self.call(ListTopics, 'topics')
        if name == 'set_topics':
            if response.success:
                self.dirty = False
            if response.success and self.start_after_selection:
                self.request_start()
            self.start_after_selection = False

    def show_profiles(self, profiles, current):
        if list(profiles) != self.profiles:
            self.profiles = list(profiles)
            self.profile_box.clear()
            self.profile_box.addItems([''] + self.profiles)
        self.profile_box.setCurrentIndex(
            self.profiles.index(current) + 1 if current in self.profiles else 0
        )

    def show_checks(self, checks):
        self.checks.setText(' | '.join(rows.check_line(c) for c in checks))

    def on_topics(self, topics):
        signature = [(t.transport, t.name, t.group, t.type) for t in topics]
        self.tree.blockSignals(True)
        if signature != self.signature:
            checked = {key: item.checkState(0) for key, item in self.items.items()}
            self.tree.clear()
            self.items = {}
            groups = {}
            for topic in topics:
                category = topic.group or 'Uncategorized'
                if category not in groups:
                    group = QTreeWidgetItem(self.tree, [category])
                    group.setFlags(group.flags() | Qt.ItemIsUserCheckable | Qt.ItemIsTristate)
                    groups[category] = group
                item = QTreeWidgetItem(
                    groups[category], [topic.name, topic.transport.upper(), topic.type]
                )
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                key = (topic.transport, topic.name)
                item.setCheckState(
                    0, checked.get(key, Qt.Checked if topic.selected else Qt.Unchecked)
                )
                item.setToolTip(0, topic.name)
                item.setToolTip(2, topic.type)
                self.items[key] = item
            self.tree.expandAll()
            self.signature = signature
        for topic in topics:
            item = self.items[(topic.transport, topic.name)]
            item.setText(7, str(topic.publishers))
            if not self.dirty:
                item.setCheckState(0, Qt.Checked if topic.selected else Qt.Unchecked)
        self.tree.blockSignals(False)
        if self.last_health:
            self.refresh_health()

    def on_health(self, health, session_preset=''):
        self.last_health = health
        self.state.setText(rows.summary(health))
        output = 'Output: ' + health.output_dir
        if session_preset:
            output += '   storage preset: ' + session_preset
        self.output.setText(output)
        editable = health.state in (Health.IDLE, Health.PREVIEW)
        self.tree.setEnabled(editable)
        self.profile_box.setEnabled(editable)
        self.preset_box.setEnabled(editable)
        self.preview_button.setEnabled(editable)
        self.start_button.setEnabled(editable)
        self.refresh_health()

    def refresh_health(self):
        health = {}
        for topic in self.last_health.topics:
            key = ('flux', topic.name[:-5]) if topic.source == 'flux' else ('ros', topic.name)
            health[key] = topic
        for key, item in self.items.items():
            topic = health.get(key)
            status, color = (
                rows.STATUS.get(topic.status, ('unknown', None)) if topic else ('-', None)
            )
            item.setText(3, status)
            item.setToolTip(3, '; '.join(topic.issues) if topic else '')
            item.setText(4, rows.number(topic.hz) if topic else '-')
            item.setText(5, rows.number(topic.age_s) if topic else '-')
            item.setText(6, str(topic.written) if topic else '-')
            brush = QBrush(QColor(color)) if color else QBrush()
            item.setBackground(3, brush)

    def shutdown(self):
        self.timer.stop()
        for client in self.clients.values():
            self.node.destroy_client(client)


class RecorderPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('RosbagRecorder')
        self._widget = RecorderWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f'{self._widget.windowTitle()} ({context.serial_number()})'
            )
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()


def main():
    from rqt_gui.main import Main

    sys.exit(Main().main(sys.argv, standalone='rosbag_recorder_rqt.recorder.RecorderPlugin'))
