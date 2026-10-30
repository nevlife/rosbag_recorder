"""Start an independent recorder and optionally attach its rqt interface."""

import argparse
import fcntl
import os
from pathlib import Path
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rosbag_recorder_msgs.srv import Status


def ensure_recorder(args):
    """Reuse a live recorder or start one outside the GUI's process session."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', tempfile.gettempdir()))
    runtime = runtime / f'rosbag_recorder-{os.getuid()}'
    runtime.mkdir(mode=0o700, exist_ok=True)
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state')))
    log_path = state / 'rosbag_recorder' / f'recorder-{domain}.log'
    with (runtime / f'{domain}.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        context = rclpy.Context()
        rclpy.init(args=[], context=context)
        node = rclpy.create_node(f'recorder_launcher_{os.getpid()}', context=context)
        client = node.create_client(Status, '/rosbag_recorder/status')
        try:
            if client.wait_for_service(timeout_sec=2.0):
                return
            lock.seek(0)
            previous = lock.read().strip()
            if previous.isdecimal() and Path(f'/proc/{previous}').exists():
                if client.wait_for_service(timeout_sec=15.0):
                    return
                raise RuntimeError(f'recorder PID {previous} is not responding; see {log_path}')
            config_dir = args.config_dir or str(
                Path(get_package_share_directory('rosbag_recorder')) / 'config')
            executable = (Path(get_package_prefix('rosbag_recorder')) / 'lib' /
                          'rosbag_recorder' / 'rosbag_recorder_node')
            command = [str(executable), '--ros-args']
            params = {'config_dir': str(Path(config_dir).expanduser().resolve()),
                      'output_dir': args.output_dir,
                      'autostart': str(args.autostart).lower()}
            if args.profile:
                params['profile'] = args.profile
            for name, value in params.items():
                command.extend(['-p', f'{name}:={value}'])
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open('ab') as log:
                process = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
            lock.seek(0)
            lock.truncate()
            lock.write(str(process.pid))
            lock.flush()
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'recorder exited; see {log_path}')
                if client.wait_for_service(timeout_sec=0.2):
                    print(f'Recorder started (PID {process.pid}); log: {log_path}')
                    return
            raise RuntimeError(f'recorder is not responding; see {log_path}')
        finally:
            node.destroy_node()
            context.shutdown()


def main(argv=None):
    """Run the GUI without tying recording to the lifetime of the window."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', default='',
                        help='configuration folder (default: bundled SCV profiles)')
    parser.add_argument('--output-dir', default='~/rosbag_recorder')
    parser.add_argument('--profile', default='')
    parser.add_argument('--autostart', action='store_true')
    parser.add_argument('--no-gui', action='store_true', help='start the recorder only')
    args = parser.parse_args(argv)
    try:
        ensure_recorder(args)
    except RuntimeError as error:
        parser.exit(1, f'{error}\n')
    if args.no_gui:
        return 0
    from rqt_gui.main import Main
    return Main().main(['rosbag_recorder'],
                       standalone='rosbag_recorder_rqt.recorder.RecorderPlugin')
