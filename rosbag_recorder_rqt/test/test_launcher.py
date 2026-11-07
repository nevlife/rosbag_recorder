"""Verify that closing and reopening the GUI does not interrupt recording."""

import argparse
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rosbag_recorder_msgs.msg import Health
from rosbag_recorder_msgs.srv import Select
from rosbag_recorder_msgs.srv import Start
from rosbag_recorder_msgs.srv import Status
from rosbag_recorder_msgs.srv import Stop
from rosbag_recorder_rqt import launcher
from std_msgs.msg import String


def test_recording_survives_gui_exit(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '93')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    # A fresh rqt settings file, so a stale plugin cache in the user's config is not used.
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    (tmp_path / 'profiles').mkdir()
    (tmp_path / 'catalog.yaml').write_text(
        'groups:\n  data:\n    topics:\n      /launcher/data: {}\n'
    )
    (tmp_path / 'profiles' / 'test.yaml').write_text(
        'name: test\ngroups: [data]\noptions:\n  start_mode: immediate\n'
        '  disk_warn_gb: 0\n  disk_stop_gb: 0\n'
    )
    args = argparse.Namespace(
        config_dir=str(tmp_path),
        output_dir=str(tmp_path / 'bags'),
        profile='test',
        autostart=False,
    )
    spawned = []
    popen = subprocess.Popen

    def spawn(*args, **kwargs):
        process = popen(*args, **kwargs)
        spawned.append(process)
        return process

    monkeypatch.setattr(launcher.subprocess, 'Popen', spawn)
    context = rclpy.Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('launcher_test_client', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    health = []
    node.create_subscription(Health, '/rosbag_recorder/health', health.append, 10)
    publisher = node.create_publisher(String, '/launcher/data', 10)
    node.create_timer(0.05, lambda: publisher.publish(String(data='recording')))

    def call(srv_type, name):
        client = node.create_client(srv_type, '/rosbag_recorder/' + name)
        try:
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(srv_type.Request())
            executor.spin_until_future_complete(future, timeout_sec=5)
            assert future.done()
            return future.result()
        finally:
            node.destroy_client(client)

    def until(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('condition did not become true')

    def written():
        return health[-1].topics[0].written if health and health[-1].topics else 0

    def gui_ready(pid):
        name = f'rqt_gui_py_node_{pid}'
        if name not in node.get_node_names():
            return False
        clients = node.get_client_names_and_types_by_node(name, '/')
        return any(service == '/rosbag_recorder/topics' for service, _ in clients)

    command = str(Path(get_package_prefix('rosbag_recorder_rqt')) / 'bin' / 'rosbag_recorder')
    try:
        launcher.ensure_recorder(args)
        assert len(spawned) == 1
        daemon = spawned[0]
        lock = next(tmp_path.glob('rosbag_recorder-*/93.lock'))
        assert lock.read_text() == str(daemon.pid)
        wait_for_service = Client.wait_for_service

        def delayed_discovery(client, timeout_sec=None):
            if timeout_sec == 2.0:
                return False
            return wait_for_service(client, timeout_sec=timeout_sec)

        with monkeypatch.context() as delayed:
            delayed.setattr(Client, 'wait_for_service', delayed_discovery)
            launcher.ensure_recorder(args)
        assert len(spawned) == 1
        assert lock.read_text() == str(daemon.pid)
        reused = subprocess.run([command, '--no-gui'], capture_output=True, text=True, timeout=10)
        assert reused.returncode == 0, reused.stderr
        assert call(Start, 'start').success
        until(lambda: written() > 10)
        for _ in range(2):
            with (tmp_path / 'gui.log').open('ab') as log:
                gui = spawn([command], stdout=log, stderr=subprocess.STDOUT)
            until(lambda: gui.poll() is not None or gui_ready(gui.pid))
            assert gui.poll() is None, (tmp_path / 'gui.log').read_text()
            assert lock.read_text() == str(daemon.pid)
            before = written()
            gui.send_signal(signal.SIGINT)
            gui.wait(timeout=10)
            until(lambda: node.count_subscribers('/rosbag_recorder/health') == 1)
            until(lambda: written() > before + 10)
            assert daemon.poll() is None
            assert call(Status, 'status').state == Health.RECORDING
        assert call(Stop, 'stop').success
    finally:
        for process in reversed(spawned):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_default_profiles(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    spawned = []
    popen = subprocess.Popen

    def spawn(*args, **kwargs):
        process = popen(*args, **kwargs)
        spawned.append(process)
        return process

    monkeypatch.setattr(launcher.subprocess, 'Popen', spawn)
    context = rclpy.Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('default_profiles_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        assert launcher.main(['--no-gui']) == 0
        client = node.create_client(Status, '/rosbag_recorder/status')
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(Status.Request())
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done()
        status = future.result()
        assert status.state == Health.IDLE
        assert status.profiles == ['default']
        assert 'base' not in status.profiles
        client = node.create_client(Select, '/rosbag_recorder/select')
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(Select.Request(profile='default'))
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done()
        assert future.result().success, future.result().message
    finally:
        for process in spawned:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
