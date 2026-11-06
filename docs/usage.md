# Usage

## Run

```bash
ros2 launch rosbag_recorder recorder.launch.py config_dir:=/path/to/config output_dir:=~/logs
```

The launch file starts `rosbag_recorder_node` with the node name `rosbag_recorder`; services and
topics below assume that name.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `config_dir` | empty | Folder with `catalog.yaml` and `profiles/` |
| `output_dir` | `~/rosbag_recorder` | Where sessions are written |
| `profile` | empty | Profile selected at start |
| `autostart` | `false` | Start recording `profile` at once |
| `label` | empty | Session label used by `autostart` |
| `workspace` | empty | Workspace whose git state and parameter files are recorded |
| `value_checks` | `true` | Run the value check plugins |
| `respawn` | `false` | Restart the node 2 s after it exits |

Every launch argument except `respawn` is also a node parameter. The node has three more
parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `param_globs` | `[]` | Workspace-relative globs (`**` allowed) of files to hash and copy into the session |
| `param_copy_max_kb` | `5120` | Total size of copied files; files past it are hashed only |
| `hz_tolerance` | `0.8` | A topic below this share of its catalog rate is `LOW`; clamped to 0.1-1.0 |

## rqt plugin

```bash
rqt --standalone rosbag_recorder_rqt.recorder.RecorderPlugin
ros2 run rosbag_recorder_rqt rosbag_recorder_rqt   # same plugin, own window
```

The plugin ("Rosbag Recorder", group "Logging") selects a profile, starts, stops, pauses and
marks, and shows the health table from `/rosbag_recorder/health`.

## Services and topics

| Name | Kind | Purpose |
| --- | --- | --- |
| `/rosbag_recorder/select` | `Select` | Load a profile and watch its topics without recording |
| `/rosbag_recorder/start` | `Start` | Start; waits for the profile's start condition unless `force` |
| `/rosbag_recorder/stop` | `Stop` | Stop, or cancel a waiting start |
| `/rosbag_recorder/pause`, `resume`, `split` | `std_srvs/Trigger` | Drop data while paused; split the bag file (not on Humble, whose writer cannot split on request) |
| `/rosbag_recorder/mark` | `Mark` | Manual marker (kind `note` when empty) |
| `/rosbag_recorder/status` | `Status` | State, profiles, last preflight |
| `/rosbag_recorder/catalog_diff` | `CatalogDiff` | Catalog topics against the live graph |
| `/rosbag_recorder/session` | `Session`, transient local | Start and end of a session; recorded |
| `/rosbag_recorder/health` | `Health`, 1 Hz | State and per-topic quality; recorded while a session is open |
| `/rosbag_recorder/markers` | `Marker` | Manual, automatic and source markers; recorded |

Message and service fields are in the generated reference of `rosbag_recorder_msgs`.

## States

| State | Meaning |
| --- | --- |
| idle | No profile selected |
| preview | Profile selected, its topics watched, nothing written |
| waiting | Start requested; waiting for the start condition (`start_mode`) |
| recording | Session open, data written |
| paused | Session open, data dropped |

Start conditions come from the profile options (see [configuration.md](configuration.md)):

- `immediate` starts at once; `force` in the `Start` request does the same for any mode.
- `delay` starts after `start_delay_sec`.
- `wait_ready` starts when every required topic is present, or after `wait_ready_sec`. If
  required topics are still missing then, `on_not_ready: abort` returns to preview and `start`
  records anyway with a `not_ready` marker.

While recording, the node stops the session (`end_reason: disk_low`, marker `disk_stop`) when the
free disk space stays below `disk_stop_gb` for three 1 Hz ticks in a row. Below `disk_warn_gb` it
only adds a warning to `Health`.

## Session layout

A session is a bag directory `<output_dir>/<YYYYmmdd_HHMMSS>_<profile>[_<label>]` with:

- the recorded topics and source channels;
- `/rosbag_recorder/session` twice: `START` with the resolved profile, node parameters, host,
  RMW, git state of `workspace`, files matched by `param_globs`, the graph and the preflight
  result; `END` with the end reason, size and marker count;
- `/rosbag_recorder/health` once per second and `/rosbag_recorder/markers`;
- `summary.yaml`: session id, profile, label, notes, host, start, end, duration, end reason,
  size, preflight, per-topic type, stored and dropped counts and last status, and the markers.
  `dropped` counts the messages that the writer cache lost; raise `max_cache_mb` if it is not 0.

Marker kinds:

| Kind | Source | When |
| --- | --- | --- |
| request `kind`, or `note` | `user` | `mark` service |
| `pause`, `resume`, `split` | `user` | The matching service |
| catalog `auto_markers[].kind` (default `auto`) | `auto` | The watched field of an `auto_markers` topic changes |
| `value_issue`, `value_ok` | `auto` | A topic's value checks turn abnormal, and normal again; a new `value_issue` comes at least 10 s after the previous one |
| `not_ready` | `auto` | `on_not_ready: start` began without required topics |
| `disk_stop` | `auto` | Free disk below `disk_stop_gb` |
| the source's own | source plugin name | `Sink::mark` |
