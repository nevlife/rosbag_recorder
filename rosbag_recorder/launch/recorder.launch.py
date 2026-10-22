# Copyright 2026 nevlife
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = {
    'config_dir': ('', 'folder with catalog.yaml and profiles/'),
    'output_dir': ('~/rosbag_recorder', 'where sessions are written'),
    'profile': ('', 'profile selected at start'),
    'autostart': ('false', 'start recording the profile at once'),
    'label': ('', 'session label used by autostart'),
    'workspace': ('', 'workspace whose git state and parameter files are recorded'),
    'value_checks': ('true', 'run the value check plugins'),
    'respawn': ('false', 'restart the node when it exits'),
}


def generate_launch_description():
    params = [{
        key: LaunchConfiguration(key)
        for key in ARGUMENTS if key not in ('respawn',)
    }]
    respawn = LaunchConfiguration('respawn')
    node = {
        'package': 'rosbag_recorder',
        'executable': 'rosbag_recorder_node',
        'name': 'rosbag_recorder',
        'output': 'screen',
        'parameters': params,
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d)
          for k, (v, d) in ARGUMENTS.items()],
        Node(**node, respawn=True, respawn_delay=2.0, condition=IfCondition(respawn)),
        Node(**node, condition=UnlessCondition(respawn)),
    ])
