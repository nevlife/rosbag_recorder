# Plugins

The recorder supports source plugins for scripted integrations. ROS and Flux selection in the
window uses built-in inputs and does not require a source plugin.

`rosbag_recorder_checks` remains an independent library. The recorder no longer discovers its
plugins, attaches checks by message type, or generates automatic value markers.

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
