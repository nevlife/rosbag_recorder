# Plugins

Both plugin kinds are pluginlib classes. Signatures are in the generated reference of
`rosbag_recorder_checks` and `rosbag_recorder_source`; this page covers how the recorder uses
them.

## Value checks

A value check implements `rosbag_recorder_checks::Check` and is exported for base class
`rosbag_recorder_checks::Check`.

| Method | Called |
| --- | --- |
| `accepts(topic)` | On one prototype instance, once per watched topic or channel |
| `configure(topic, thresholds)` | On a new instance per accepted topic; thresholds are the catalog's `value_checks` |
| `on_message(message, wall)` | For every message, with the serialized bytes and the receive time |
| `evaluate(context, result)` | Once per health tick; fills issues and values, false when no message arrived |

Issues turn the topic `ABNORMAL` in `Health` and add `value_issue` and `value_ok` markers.
`value_checks:=false` loads no check.

Built-in checks (`rosbag_recorder_checks/<name>`):

| Check | Looks at |
| --- | --- |
| `HeaderStamp` | Delay, future, zero, backwards and repeated `header.stamp`, read from the CDR bytes of any type whose first field is `std_msgs/Header` |
| `Imu` | `sensor_msgs/Imu` values all zero or not finite |
| `NavSatFix` | Horizontal accuracy from the position covariance |
| `TfMessage` | Transforms not finite or quaternions not unit length |
| `PointCloud2` | Empty clouds, read from the CDR bytes |

Their thresholds are listed in [configuration.md](configuration.md#catalogyaml).

## Sources

A source implements `rosbag_recorder_source::Source` for data that is not a ROS topic.

| Method | Called |
| --- | --- |
| `initialize(node, name, params_yaml)` | When a profile with the source is selected; `params_yaml` is the catalog's `params` |
| `channels()` | Declares channels: name, type, serialization format, schema and its encoding (empty: looked up from the type), expected rate, required, latched |
| `start(sink)`, `stop()` | Right after `initialize`, and when the profile is unselected; between them the source may call the sink from any thread |
| `publishers(channel)` | Publisher count reported in `Health`; default 1 |

The source hands serialized messages with their time (on the clock of the node given to
`initialize`) to `sink.write(channel, message, time_ns)` and markers to `sink.mark(kind, text,
topic)`. Its channels are monitored and recorded like topics: watched in preview, written while recording. The catalog's `channels` entry for a channel overrides its rate and can make it required or latched.

A source is declared in the catalog's `sources` and enabled per profile with `sources`. The test
package builds `rosbag_recorder_test/CounterSource` (`rosbag_recorder/test/counter_source.cpp`)
as an example.
