"""Run the recorder in a Flux component container."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# Empty keeps the value of config/rosbag_recorder.param.yaml.
ARGUMENTS = {
    'config_dir': (str, 'folder with catalog.yaml and profiles/ (default: installed config)'),
    'output_dir': (str, 'where sessions are written'),
    'profile': (str, 'profile selected at start'),
    'autostart': (bool, 'start recording the profile at once'),
    'label': (str, 'session label used by autostart'),
    'workspace': (str, 'workspace whose git state and parameter files are recorded'),
}
OUTPUTS = ('session', 'health', 'markers')


def recorder(name='rosbag_recorder', overrides=None, **container):
    """Return the container that runs one recorder; its topics are /<name>/<output>."""
    share = Path(get_package_share_directory('rosbag_recorder'))
    node = ComposableNode(
        package='rosbag_recorder',
        plugin='rosbag_recorder::RecorderNode',
        name=name,
        parameters=[
            str(share / 'config' / 'rosbag_recorder.param.yaml'),
            {'config_dir': str(share / 'config'), **(overrides or {})},
        ],
        remappings=[(f'~/output/{topic}', f'/{name}/{topic}') for topic in OUTPUTS],
    )
    return ComposableNodeContainer(
        name=f'{name}_container',
        namespace='',
        package='flux_components',
        executable='component_container',
        composable_node_descriptions=[node],
        **container,
    )


def launch_setup(context):
    overrides = {
        key: ParameterValue(LaunchConfiguration(key), value_type=value_type)
        for key, (value_type, _) in ARGUMENTS.items()
        if LaunchConfiguration(key).perform(context)
    }
    respawn = LaunchConfiguration('respawn').perform(context) == 'true'
    return [recorder(overrides=overrides, output='screen', respawn=respawn, respawn_delay=2.0)]


def generate_launch_description():
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(key, default_value='', description=description)
                for key, (_, description) in ARGUMENTS.items()
            ],
            DeclareLaunchArgument(
                'respawn', default_value='false', description='restart the recorder when it exits'
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
