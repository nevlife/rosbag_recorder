# rosbag_recorder

Records ROS 2 topics into rosbag2 while watching their quality, and keeps everything about the
run in the same bag. Targets ROS 2 Humble and Jazzy with the same code.

- One subscription per topic. The serialized message goes to the bag writer
  (`rosbag2_cpp::Writer`) and to monitoring; nothing is deserialized except small types a value
  check asks for.
- Profiles and a topic catalog in the SCV `scv_logger` format (`catalog.yaml`,
  `profiles/*.yaml`); existing SCV configuration folders work as they are.
- Session description, 1 Hz health and markers are recorded as topics in the bag; a short
  `summary.yaml` sits beside the `.mcap` files.
- Value checks and non-ROS data sources are pluginlib plugins.
- An rqt plugin to select a profile, start, stop, pause, mark and watch the health table.

## Packages

| Package | Content |
| --- | --- |
| `rosbag_recorder` | The node (component `rosbag_recorder::RecorderNode`, executable `rosbag_recorder_node`), profile and catalog parsing, monitoring, writer |
| `rosbag_recorder_msgs` | `Session`, `Health`, `TopicHealth`, `Marker`, `CheckResult`, `FileSnapshot`, `CatalogDiffItem`; services `Select`, `Start`, `Stop`, `Mark`, `Status`, `CatalogDiff` |
| `rosbag_recorder_checks` | Value check interface `rosbag_recorder_checks::Check` and built-in checks |
| `rosbag_recorder_source` | Source interface `rosbag_recorder_source::Source` for data that is not a ROS topic |
| `rosbag_recorder_rqt` | rqt plugin "Rosbag Recorder" |

## Build and run

```bash
source /opt/ros/jazzy/setup.bash        # or humble
colcon build --symlink-install
source install/setup.bash
ros2 launch rosbag_recorder recorder.launch.py config_dir:=/path/to/config output_dir:=~/logs
rqt --standalone rosbag_recorder_rqt.recorder.RecorderPlugin   # or ros2 run rosbag_recorder_rqt rosbag_recorder_rqt
```

Launch arguments: `config_dir`, `output_dir`, `profile`, `autostart`, `label`, `workspace`,
`value_checks`, `respawn`. With `profile` and `autostart:=true` the node starts recording without
any call. Node parameters also include `param_globs` (workspace-relative globs, `**` allowed, of
files to hash and copy into the session), `param_copy_max_kb` and `hz_tolerance` (0.8: a topic
below 80% of its catalog rate is `low`).

## Interface

| Name | Kind | Purpose |
| --- | --- | --- |
| `/rosbag_recorder/select` | `Select` | Load a profile and watch its topics without recording |
| `/rosbag_recorder/start` | `Start` | Start; waits for the profile's start condition unless `force` |
| `/rosbag_recorder/stop` | `Stop` | Stop, or cancel a waiting start |
| `/rosbag_recorder/pause`, `resume`, `split` | `std_srvs/Trigger` | Drop data while paused; split the bag file |
| `/rosbag_recorder/mark` | `Mark` | Manual marker |
| `/rosbag_recorder/status` | `Status` | State, profiles, last preflight |
| `/rosbag_recorder/catalog_diff` | `CatalogDiff` | Catalog topics against the live graph |
| `/rosbag_recorder/session` | `Session`, transient local | Start and end of a session; recorded |
| `/rosbag_recorder/health` | `Health`, 1 Hz | State and per-topic quality; recorded while a session is open |
| `/rosbag_recorder/markers` | `Marker` | Manual, automatic and source markers; recorded |

States: idle, preview (profile selected, topics watched), waiting (start condition), recording,
paused. A session is a bag directory `<output_dir>/<YYYYmmdd_HHMMSS>_<profile>[_<label>]`.

## Configuration

`catalog.yaml` and `profiles/*.yaml` follow SCV `scv_logger` exactly: groups of topics with
`type`, `hz`, `required`, `heavy`, `size_kb`, `latched`, `description`; `auto_markers`;
`value_checks`; profiles with `extends`, `groups`, `add_groups`, `exclude_groups`, `topics`,
`exclude` and the SCV options (`storage`, `compression`, `split_duration_sec`, `split_size_mb`,
`heavy_qos`, `start_mode`, `wait_ready_sec`, `start_delay_sec`, `disk_warn_gb`, `disk_stop_gb`,
`max_cache_mb`). `monitor_via` is accepted and not used, because every recorded topic is measured
on its own subscription. Optional additions:

| Where | Key | Meaning |
| --- | --- | --- |
| profile `options` | `storage_preset` | rosbag2 storage preset; default `fastwrite` (no compression) or `zstd_fast` for MCAP |
| profile `options` | `storage_config_uri` | rosbag2 storage configuration file |
| profile `options` | `on_not_ready` | `abort` (default) or `start` when `wait_ready` times out |
| profile `options` | `required_frames` | TF frames the preflight looks for on TF topics of the profile |
| profile | `sources`, `exclude_sources` | Source plugins to record, as declared in the catalog |
| catalog | `sources` | `name: {plugin, description, params, channels: {topic: {hz, required, latched}}}` |
| catalog `value_checks` | any key | Thresholds passed to every check plugin |

## Plugins

A value check implements `rosbag_recorder_checks::Check` (`accepts`, `configure`, `on_message`,
`evaluate`) and is exported for base class `rosbag_recorder_checks::Check`. Built-in:
`HeaderStamp` (delay, future, zero, backwards and repeated `header.stamp`, read from the CDR
bytes of any type whose first field is `std_msgs/Header`), `Imu`, `NavSatFix`, `TfMessage`,
`PointCloud2` (empty clouds, read from the CDR bytes).

A source implements `rosbag_recorder_source::Source`: it declares channels (name, type, schema,
expected rate), and from `start(sink)` until `stop()` hands serialized messages with their time
to `sink.write(channel, message, time_ns)` and markers to `sink.mark(...)`. Its channels are
monitored and recorded like topics. The test package builds `rosbag_recorder_test/CounterSource`
as an example.

## Tests

`colcon test` runs the linters, gtests for configuration parsing (SCV's configuration as
fixtures, compared with the profiles SCV's own code resolves), CDR reading and each check, and a
launch test that records a 4 MiB topic at 10 Hz with other topics and the counter source,
checks that each topic has one subscription, and reads the bag back with `ros2 bag info` and,
when it is on `PYTHONPATH`, `rosbag2_dataset`.

## License

MIT, see [LICENSE](LICENSE).
