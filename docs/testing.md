# Testing

Build with `colcon build --symlink-install` before testing. From the ROS 2 workspace:

```bash
colcon test --packages-select rosbag_recorder rosbag_recorder_msgs rosbag_recorder_rqt
colcon test-result --verbose
```

The main checks are:

| Test                                        | Behavior                                                                                                                                                                                                       |
| ------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `test_selection.launch.py`                  | ROS and Flux with the same name, categories independent of type, unknown custom Flux fingerprints, byte and metadata preservation, publisher restart, pause/resume, selection changes rejected while recording |
| `test_lifecycle.launch.py`                  | Profile changes, immediate stop, autostart validation, no automatic semantic IMU issues, QoS reconnection                                                                                                      |
| `test_record.launch.py`                     | Large ROS messages, manual markers through the service API, split, and bag readback                                                                                                                            |
| `test_cache_drop.launch.py`                 | A 1 MiB writer cache overflowed by 1 MiB messages; the `summary.yaml` counts match the bag                                                                                                                     |
| `rosbag_recorder_rqt/test/test_rqt.py`      | Category and individual checkboxes, ROS/Flux distinction, selection preservation during discovery, statistics, selection applied before Start                                                                  |
| `rosbag_recorder_rqt/test/test_rqt_perf.py` | 500 inputs keep their selection and item identity across repeated updates and newly discovered topics                                                                                                          |
| `rosbag_recorder_rqt/test/test_launcher.py` | Default configuration, recorder reuse, and uninterrupted recording when the offscreen GUI closes and reopens                                                                                                   |

Core gtests (`test_core`) cover profile resolution, catalog parsing, rate calculation, preflight,
the start modes, the disk watch, the summary, the catalog diff, topic selection and session
metadata. The launch tests start the recorder through `recorder.launch.py`, so they use its
container, parameter file and remaps. C++ and Python style checks run through the package linters.

`test_scale.launch.py` retains the optional node timing benchmark. Run timing measurements on
reserved cores. Its environment variables are defined in the test; no previous UI timings apply
to the checkbox tree.
