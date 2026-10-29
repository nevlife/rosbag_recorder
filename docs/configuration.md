# Configuration

A configuration folder (`config_dir`) holds `catalog.yaml` and `profiles/<name>.yaml`. Unknown
keys are errors.

## catalog.yaml

| Key | Default | Meaning |
| --- | --- | --- |
| `version` | `1` | Format version |
| `groups` | | `name: {description, topics: {topic: spec}}`; group names use lowercase letters, digits and `_`; a topic belongs to one group only |
| `auto_markers` | | List of `{topic, label, field, kind}`; a change of `field` on `topic` adds a marker of `kind` (default `auto`) |
| `value_checks` | | Positive numbers passed as thresholds to every check plugin |
| `sources` | | `name: {plugin, description, params, channels: {topic: spec}}`; `plugin` is required, `params` is handed to the plugin as YAML text |

Topic spec (also used for source channels):

| Key | Meaning |
| --- | --- |
| `type` | `pkg/msg/Type` |
| `hz` | Expected rate; a positive number |
| `required` | Start condition and preflight require it |
| `heavy` | Large topic; subject to `heavy_qos` |
| `size_kb` | Expected message size |
| `latched` | Transient local; the subscription keeps the last 100 messages. A topic whose publishers are all transient local is treated as latched as well |
| `description` | Free text |

Value check thresholds read by the built-in checks:

| Key | Default | Check |
| --- | --- | --- |
| `stamp_delay_max_s` | `0.5` | `HeaderStamp` |
| `stamp_future_max_s` | `0.2` | `HeaderStamp` |
| `gnss_max_std_m` | `10.0` | `NavSatFix` |
| `tf_quaternion_tolerance` | `0.01` | `TfMessage` |

## Profiles

`profiles/<name>.yaml`; `name` must equal the file name.

| Key | Meaning |
| --- | --- |
| `name`, `description` | |
| `extends` | Parent profile; topics, sources and options are inherited |
| `abstract` | Parsed and kept in the resolved profile; the recorder does not refuse abstract profiles |
| `groups` | Replaces the inherited topic set with these catalog groups |
| `add_groups`, `exclude_groups` | Add or remove catalog groups |
| `topics`, `exclude` | Add or remove single topics; topics not in the catalog are recorded without a reference rate, with a preflight warning |
| `sources`, `exclude_sources` | Source plugins to record, as declared in the catalog |
| `options` | Recording options below; a child's options override its parent's |

Resolution order: parent, `groups`, `add_groups`, `exclude_groups`, `topics`, `exclude`.
Inheritance cycles and chains deeper than 8 profiles are errors.

### Options

| Option | Default | Values |
| --- | --- | --- |
| `storage` | `mcap` | `mcap`, `sqlite3` |
| `compression` | `none` | `none`, `zstd`. MCAP compresses through the storage preset, `sqlite3` with file compression |
| `split_duration_sec` | `300` | Integer >= 0; 0 disables |
| `split_size_mb` | `0` | Integer >= 0; 0 disables |
| `heavy_qos` | `keep` | `keep`, `best_effort` (heavy topics subscribed best effort) |
| `start_mode` | `wait_ready` | `wait_ready`, `delay`, `immediate` |
| `wait_ready_sec` | `30` | Integer >= 1 with `wait_ready` |
| `start_delay_sec` | `5` | Integer >= 0 |
| `disk_warn_gb` | `30.0` | Number >= 0 |
| `disk_stop_gb` | `10.0` | Number >= 0, not larger than `disk_warn_gb` |
| `max_cache_mb` | `100` | Integer >= 1; rosbag2 writer cache |
| `storage_preset` | see below | rosbag2 storage preset |
| `storage_config_uri` | empty | rosbag2 storage configuration file |
| `on_not_ready` | `abort` | `abort`, `start`; what `wait_ready` does when required topics are missing at the deadline |
| `required_frames` | `[]` | TF frames the preflight looks for on TF topics of the profile |

Without `storage_preset`, MCAP uses `zstd_fast` when `compression: zstd` and `fastwrite`
otherwise; `sqlite3` uses no preset.

## Example

```yaml
# catalog.yaml
version: 1
value_checks:
  stamp_delay_max_s: 0.5
groups:
  sensors:
    topics:
      /imu: {type: sensor_msgs/msg/Imu, hz: 50.0, required: true}
      /points: {type: sensor_msgs/msg/PointCloud2, hz: 10.0, heavy: true, size_kb: 4096.0}
  tf:
    topics:
      /tf_static: {type: tf2_msgs/msg/TFMessage, latched: true}
```

```yaml
# profiles/drive.yaml
name: drive
groups: [sensors, tf]
options:
  start_mode: wait_ready
  wait_ready_sec: 15
  required_frames: [base_link]
```

The launch test configuration in `rosbag_recorder/test/data/launch` adds a source and an
automatic marker.
