# Testing

```bash
colcon test --packages-select rosbag_recorder rosbag_recorder_checks rosbag_recorder_msgs \
  rosbag_recorder_rqt rosbag_recorder_source
colcon test-result --verbose
```

`colcon test` runs:

- the linters of the CMake packages (copyright, cpplint, uncrustify, xmllint; flake8 and pep257
  for the launch files);
- gtests for configuration parsing (SCV's configuration as fixtures, compared with the profiles
  SCV's own code resolves; see [scv-compatibility.md](scv-compatibility.md)), CDR reading and
  each check;
- `test_record.launch.py`: records a 4 MiB topic at 10 Hz with other topics and the counter
  source, checks that each topic has one subscription, and reads the bag back with
  `ros2 bag info` and, when it is on `PYTHONPATH`, `rosbag2_dataset`;
- the responsiveness tests below.

## Responsiveness tests

Two tests keep the node and the rqt plugin responsive with many topics. Their budgets are
environment variables (`SCALE_*`, `RQT_PERF_*`) so a slower board can set its own.

| Test | Setup | Budget |
| --- | --- | --- |
| `test_scale.launch.py` | 500 IMU topics at 10 Hz from one process, preview for 10 s, recording for 150 s | graph read in under half of the preview ticks (`Health.graph_reads`), tick p50 < 15 ms and p99 < 40 ms, `status` call p50 < 20 ms, RSS growth after warm-up < 20 MB |
| `rosbag_recorder_rqt/test/test_rqt_perf.py` | `Health` with 500 topics of about 180-character names at 1 Hz for 30 s, offscreen; a tab round trip and Mark every 0.2 s | first table filled < 2 s, update p99 < 50 ms, longest event-loop gap and tab switch with Mark < 100 ms |
| `rosbag_recorder_rqt/test/test_rqt.py` | Hidden table, repeated updates, 600 markers | not filled while hidden, the same items after an update, no ResizeToContents column, 500 markers kept |

| Variable | Default |
| --- | --- |
| `SCALE_TOPICS`, `SCALE_RECORD_S` | `500`, `150` |
| `SCALE_TICK_P50_MS`, `SCALE_TICK_P99_MS` | `15`, `40` |
| `SCALE_GRAPH_READ_SHARE` | `0.5` |
| `SCALE_STATUS_P50_MS`, `SCALE_RSS_GROWTH_MB` | `20`, `20` |
| `RQT_PERF_TOPICS`, `RQT_PERF_SECONDS` | `500`, `30` |
| `RQT_PERF_UPDATE_P99_MS`, `RQT_PERF_LOOP_MAX_MS`, `RQT_PERF_STARTUP_S` | `50`, `100`, `2` |
| `SCALE_REPORT`, `RQT_PERF_REPORT` | unset; a path to write the measured values to |

The time budgets leave room for slow cores; the exact checks (graph reads, item identity, resize
mode) catch the regressions that time alone misses on a fast core.

## Measurements

Node, measured on reserved cores (`pinned_run.py`, CPUs 8, 10, 12, 14 of an x86 desktop, Fast
DDS, Jazzy), three runs each, before and after alternating; before is the same build with the
graph read on every tick, every profile parsed on every `status` call and 4000 arrival times
kept per topic:

| Value | Before | After |
| --- | --- | --- |
| Graph read in preview ticks | 100% | 0% |
| Tick in preview, p50 / p99 | 3.4-3.5 / 3.5-3.8 ms | 1.8 / 3.5-3.6 ms |
| Tick while recording, p50 / p99 | 3.6-3.9 / 4.2-5.5 ms | 2.0 / 3.6-3.7 ms |
| Node RSS growth over 150 s of recording | 2.2 MB | -0.6 to 2.4 MB |

Single runs at other times of the day gave preview p50 up to 8.3 ms before and 6.5 ms after, so
compare alternating runs only. On Humble in a container on cores 0-7 and 16-27 (unpinned): tick
p50 5.2 ms, p99 10.8 ms, graph read 0%.

rqt plugin, same cores, Jazzy:

| Value | Before | After |
| --- | --- | --- |
| Table update, p50 / p99 | 13 346 ms (every update) | 4.1 / 8.7 ms |
| Longest event-loop gap | 13 353 ms | 30 ms |
| Tab switch and Mark | 5.5 ms (no tabs) | 28 ms |
| First table filled | 0.11 s | 0.12 s |

Before, every table update set each cell's text with the columns in ResizeToContents, which
measures the column again per cell. On Humble (container, unpinned) the plugin measured update
p99 10.0 ms, longest gap 45 ms.
