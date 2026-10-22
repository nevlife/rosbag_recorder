# Copyright 2026 pgw
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import sys
import time

from python_qt_binding.QtCore import Qt, QTimer, Signal
from python_qt_binding.QtGui import QBrush, QColor
from python_qt_binding.QtWidgets import (
    QCheckBox, QComboBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QListWidget,
    QPushButton, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
from rosbag_recorder_msgs.msg import CatalogDiffItem, Health, Marker
from rosbag_recorder_msgs.srv import CatalogDiff, Mark, Select, Start, Status, Stop
from rqt_gui_py.plugin import Plugin
from std_srvs.srv import Trigger

from . import rows

PREFIX = '/rosbag_recorder/'
DIFF_KINDS = {CatalogDiffItem.NEW: 'new', CatalogDiffItem.MISSING: 'missing',
              CatalogDiffItem.TYPE_CHANGED: 'type changed'}


class RecorderWidget(QWidget):
    """Buttons call the recorder's services; the table follows /rosbag_recorder/health."""

    health_received = Signal(object)
    marker_received = Signal(object)
    replied = Signal(str, object)

    def __init__(self, node):
        super().__init__()
        self.setWindowTitle('Rosbag Recorder')
        self.node = node
        self.wanted_profile = ''
        self.clients = {}
        self.pending = set()
        self.build()
        self.health_received.connect(self.on_health)
        self.marker_received.connect(self.on_marker)
        self.replied.connect(self.on_reply)
        self.subs = [
            node.create_subscription(Health, PREFIX + 'health',
                                     lambda m: self.health_received.emit(m), 10),
            node.create_subscription(Marker, PREFIX + 'markers',
                                     lambda m: self.marker_received.emit(m), 100),
        ]
        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: self.call(Status, 'status'))
        self.timer.start(1000)

    def build(self):
        self.profile = QComboBox()
        self.label = QLineEdit(placeholderText='label')
        self.notes = QLineEdit(placeholderText='notes')
        self.force = QCheckBox('force (skip start condition)')
        self.mark_text = QLineEdit(placeholderText='marker text')
        self.state = QLabel('no recorder')
        self.message = QLabel('')
        self.preflight = QListWidget()
        self.markers = QListWidget()
        self.diff = QListWidget()
        self.table = QTableWidget(0, len(rows.COLUMNS))
        self.table.setHorizontalHeaderLabels(rows.COLUMNS)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)

        def button(text, slot):
            b = QPushButton(text)
            b.clicked.connect(slot)
            return b

        top = QHBoxLayout()
        top.addWidget(QLabel('Profile'))
        top.addWidget(self.profile, 1)
        top.addWidget(button('Select', self.select))
        top.addWidget(self.label, 1)
        top.addWidget(self.notes, 2)
        controls = QHBoxLayout()
        controls.addWidget(button('Start', self.start))
        controls.addWidget(self.force)
        for text, name in (('Stop', None), ('Pause', 'pause'), ('Resume', 'resume'),
                           ('Split', 'split')):
            controls.addWidget(button(text, (lambda n=name: self.call(Trigger, n))
                                      if name else lambda: self.call(Stop, 'stop')))
        controls.addWidget(self.mark_text, 1)
        controls.addWidget(button('Mark', self.mark))
        controls.addWidget(button('Catalog diff', lambda: self.call(CatalogDiff, 'catalog_diff')))

        lists = QSplitter(Qt.Horizontal)
        for title, widget in (('Preflight', self.preflight), ('Markers', self.markers),
                              ('Catalog vs graph', self.diff)):
            box = QWidget()
            layout = QVBoxLayout(box)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(QLabel(title))
            layout.addWidget(widget)
            lists.addWidget(box)
        split = QSplitter(Qt.Vertical)
        split.addWidget(self.table)
        split.addWidget(lists)
        split.setStretchFactor(0, 3)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(controls)
        layout.addWidget(self.state)
        layout.addWidget(self.message)
        layout.addWidget(split, 1)

    def call(self, srv_type, name, **fields):
        if name in self.pending:
            return
        client = self.clients.get(name)
        if client is None:
            client = self.clients[name] = self.node.create_client(srv_type, PREFIX + name)
        if not client.service_is_ready():
            if name != 'status':
                self.message.setText(f'{PREFIX}{name} is not available')
            return
        self.pending.add(name)
        future = client.call_async(srv_type.Request(**fields))
        future.add_done_callback(lambda f: self.replied.emit(name, f.result()))

    def select(self):
        self.call(Select, 'select', profile=self.profile.currentText())

    def start(self):
        self.call(Start, 'start', profile=self.profile.currentText(), label=self.label.text(),
                  notes=self.notes.text(), force=self.force.isChecked())

    def mark(self):
        self.call(Mark, 'mark', kind='note', text=self.mark_text.text())
        self.mark_text.clear()

    def on_reply(self, name, response):
        self.pending.discard(name)
        if response is None:
            return
        if name == 'status':
            self.on_status(response)
            return
        if name == 'catalog_diff':
            self.diff.clear()
            for item in response.items:
                types = ' -> '.join(t for t in (item.catalog_type, item.graph_type) if t)
                self.diff.addItem(f'{DIFF_KINDS[item.kind]:12} {item.topic}  {types}')
        if hasattr(response, 'preflight') and response.preflight:
            self.show_preflight(response.preflight)
        self.message.setText(f'{name}: {response.message}')

    def on_status(self, status):
        names = list(status.profiles)
        if names != [self.profile.itemText(i) for i in range(self.profile.count())]:
            current = status.profile or self.wanted_profile or self.profile.currentText()
            self.profile.clear()
            self.profile.addItems(names)
            if current in names:
                self.profile.setCurrentIndex(names.index(current))
        if status.message and not self.message.text():
            self.message.setText(status.message)
        self.show_preflight(status.preflight)

    def show_preflight(self, checks):
        lines = [rows.check_line(c) for c in checks]
        if lines != [self.preflight.item(i).text() for i in range(self.preflight.count())]:
            self.preflight.clear()
            self.preflight.addItems(lines)

    def on_health(self, health):
        self.state.setText(rows.summary(health))
        self.table.setRowCount(len(health.topics))
        for r, topic in enumerate(health.topics):
            cells, color = rows.topic_row(topic)
            for c, text in enumerate(cells):
                item = self.table.item(r, c)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(r, c, item)
                item.setText(text)
                item.setBackground(QBrush(QColor(color)) if color else QBrush())

    def on_marker(self, marker):
        stamp = time.strftime('%H:%M:%S', time.localtime(marker.stamp.sec))
        self.markers.insertItem(0, f'{stamp} [{marker.kind}] {marker.text}')

    def shutdown(self):
        self.timer.stop()
        for sub in self.subs:
            self.node.destroy_subscription(sub)
        for client in self.clients.values():
            self.node.destroy_client(client)


class RecorderPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('RosbagRecorder')
        self._widget = RecorderWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f'{self._widget.windowTitle()} ({context.serial_number()})')
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('profile', self._widget.profile.currentText())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.wanted_profile = instance_settings.value('profile', '')


def main():
    from rqt_gui.main import Main
    sys.exit(Main().main(sys.argv, standalone='rosbag_recorder_rqt.recorder.RecorderPlugin'))
