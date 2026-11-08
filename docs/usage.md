# Usage

## Desktop window

```bash
rosbag_recorder
```

The launcher starts a recorder if none is available in the current ROS domain, then opens RQT.
It starts the recorder with `ros2 launch rosbag_recorder recorder.launch.py` and uses the
installed catalog by default. The window lists discovered ROS topics and live Flux
channels, grouped by the catalog's topic names. Missing selected Flux channels remain visible
so that a publisher restart does not discard their selection.

1. Check a category or individual topics. Uncategorized inputs remain individually selectable.
2. Click Preview selection to monitor the inputs before recording, or Start to apply the checked
   selection and begin recording immediately.
3. Watch reception status, Hz, age, written counts and disk space.
4. Click Stop to finish. Topic selection is locked while recording or paused.

An empty selection cannot start recording. ROS and Flux inputs with the same name are distinct.
Newly discovered inputs are left unchecked. The window has no profile menu, preflight tab,
marker controls or catalog comparison tab. File-system and disk checks appear in the main window.
No message-type-specific value checks or automatic value markers are attached.

Pause discards incoming messages until Resume. Split starts the next file in the same session.
Closing the GUI does not stop recording. Open `rosbag_recorder` again to reconnect.

## Launcher options

| Option         | Default                                  | Meaning                                              |
| -------------- | ---------------------------------------- | ---------------------------------------------------- |
| `--config-dir` | Installed `share/rosbag_recorder/config` | Catalog and optional profiles for a new recorder     |
| `--output-dir` | `~/rosbag_recorder`                      | Session directory for a new recorder                 |
| `--profile`    | empty                                    | Optional profile for scripted startup                |
| `--autostart`  | disabled                                 | Record the supplied profile on startup               |
| `--no-gui`     | disabled                                 | Start or reuse the recorder without opening a window |

Options are passed to the launch file as `config_dir`, `output_dir`, `profile` and `autostart`.
They do not reconfigure an already running recorder. Recorder output is appended to
`$XDG_STATE_HOME/rosbag_recorder/recorder-<domain>.log`, or
`~/.local/state/rosbag_recorder/recorder-<domain>.log` when `XDG_STATE_HOME` is unset.

## Launch file

`recorder.launch.py` loads `config/rosbag_recorder.param.yaml` and overrides a parameter with
each non-empty launch argument (`config_dir`, `output_dir`, `profile`, `autostart`, `label`,
`workspace`). `config_dir` defaults to the installed `share/rosbag_recorder/config`.
`respawn:=true` restarts the container when it exits.

The node publishes on `~/output/<name>`. The launch file remaps these names to the topics below.
The same names are the bag topics of the session channels.

| Node name          | Topic                      | Type                           |
| ------------------ | -------------------------- | ------------------------------ |
| `~/output/session` | `/rosbag_recorder/session` | `rosbag_recorder_msgs/Session` |
| `~/output/health`  | `/rosbag_recorder/health`  | `rosbag_recorder_msgs/Health`  |
| `~/output/markers` | `/rosbag_recorder/markers` | `rosbag_recorder_msgs/Marker`  |

## Topic selection services

| Service                                     | Type               | Behavior                                                                               |
| ------------------------------------------- | ------------------ | -------------------------------------------------------------------------------------- |
| `/rosbag_recorder/topics`                   | `ListTopics`       | List discovered inputs, their category, transport, type, publishers and selected state |
| `/rosbag_recorder/set_topics`               | `SetTopics`        | Apply explicit `{name, transport}` selections; transport is `ros` or `flux`            |
| `/rosbag_recorder/start`                    | `Start`            | Begin recording the selection; no profile argument is needed                           |
| `/rosbag_recorder/stop`                     | `Stop`             | Close the current bag                                                                  |
| `/rosbag_recorder/pause`, `resume`, `split` | `std_srvs/Trigger` | Control an open session                                                                |

The existing `Select`, `Status`, `Mark` and `CatalogDiff` service APIs remain available for scripts.
Health and session metadata continue to be stored in the bag, with `summary.yaml` at session end.
For each topic, `summary.yaml` gives the stored count from the closed bag as `written` and the
messages that the writer cache lost as `dropped`. Raise `max_cache_mb` if `dropped` is not 0.
Disk-space thresholds from the recording options still warn or stop recording.

## Flux bags

Flux channel `NAME` is stored on bag topic `NAME/flux` as `flux_msgs/msg/FluxFrame`. The wrapper
contains the original name, fingerprint, commit timestamp, raw frame bytes and slot/array metadata.
The input's custom ROS type does not need to be installed for raw recording. Unknown fingerprints
appear as `raw` in the list. Use `ros2 bag play_flux BAG` for Flux replay.

A ROS topic whose name already equals a selected Flux input's bag topic would collide; the
selection service rejects that combination. CPU recording requires host-accessible payloads.
