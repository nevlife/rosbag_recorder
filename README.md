# rosbag_recorder

Records ROS 2 topics into rosbag2 while watching their quality, and keeps everything about the
run in the same bag.

- One subscription per topic. The serialized message goes to the bag writer
  (`rosbag2_cpp::Writer`) and to monitoring; nothing is deserialized except small types a value
  check asks for.
- Profiles and a topic catalog in the SCV `scv_logger` format; existing SCV configuration
  folders work as they are.
- Session description, 1 Hz health and markers are recorded as topics in the bag; a short
  `summary.yaml` sits beside the `.mcap` files.
- Value checks and non-ROS data sources are pluginlib plugins.
- An rqt plugin to select a profile, start, stop, pause, mark and watch the health table.

## Supported environment

| OS | ROS 2 | Notes |
| --- | --- | --- |
| Ubuntu 24.04 | Jazzy | All features |
| Ubuntu 22.04 | Humble | Same code; `split` on request is not available (Humble's writer cannot split) |

The node has no RMW-specific code.

## Packages

| Package | Content |
| --- | --- |
| `rosbag_recorder` | The node (component `rosbag_recorder::RecorderNode`, executable `rosbag_recorder_node`), profile and catalog parsing, monitoring, writer |
| `rosbag_recorder_msgs` | Session, health and marker messages; control services |
| `rosbag_recorder_checks` | Value check interface `rosbag_recorder_checks::Check` and built-in checks |
| `rosbag_recorder_source` | Source interface `rosbag_recorder_source::Source` for data that is not a ROS topic |
| `rosbag_recorder_rqt` | rqt plugin "Rosbag Recorder" |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash        # or humble
colcon build --symlink-install
source install/setup.bash
ros2 launch rosbag_recorder recorder.launch.py config_dir:=/path/to/config output_dir:=~/logs
rqt --standalone rosbag_recorder_rqt.recorder.RecorderPlugin   # or ros2 run rosbag_recorder_rqt rosbag_recorder_rqt
```

`config_dir` holds `catalog.yaml` and `profiles/*.yaml`. With `profile:=<name>
autostart:=true` the node starts recording without any call.

## Documentation

| Document | Content |
| --- | --- |
| [docs/usage.md](docs/usage.md) | Launch arguments, node parameters, services and topics, states, session layout |
| [docs/configuration.md](docs/configuration.md) | `catalog.yaml` and profile reference with defaults |
| [docs/scv-compatibility.md](docs/scv-compatibility.md) | What is shared with SCV `scv_logger` and what is added |
| [docs/plugins.md](docs/plugins.md) | Value check and source plugins |
| [docs/testing.md](docs/testing.md) | Tests, responsiveness budgets and measurements |
| [docs/release-notes/unreleased.md](docs/release-notes/unreleased.md) | Changes since the last release |

The API reference (C++ headers, Python modules, message and service definitions) is generated
with [rosdoc2](https://github.com/ros-infrastructure/rosdoc2) per package:

```bash
rosdoc2 build -p rosbag_recorder -o <output dir>
```

## License

MIT, see [LICENSE](LICENSE).
