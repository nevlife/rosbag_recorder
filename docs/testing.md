# Testing

Build with `colcon build --symlink-install` before testing. From the ROS 2 workspace:

```bash
colcon test --packages-select rosbag_recorder rosbag_recorder_checks rosbag_recorder_msgs \
  rosbag_recorder_rqt rosbag_recorder_source
colcon test-result --verbose
```

The main checks are:

| Test | Behavior |
| --- | --- |
| `test_selection.launch.py` | ROS and Flux with the same name, categories independent of type, unknown custom Flux fingerprints, byte and metadata preservation, publisher restart, pause/resume, selection changes rejected while recording |
| `test_lifecycle.launch.py` | Profile changes, immediate stop, autostart validation, no automatic semantic IMU issues, QoS reconnection |
| `test_record.launch.py` | Large ROS messages, a source plugin, manual markers through the service API, split, and bag readback |
| `rosbag_recorder_rqt/test/test_rqt.py` | Category and individual checkboxes, ROS/Flux distinction, selection preservation during discovery, statistics, selection applied before Start |
| `rosbag_recorder_rqt/test/test_rqt_perf.py` | 500 inputs keep their selection and item identity across repeated updates and newly discovered topics |
| `rosbag_recorder_rqt/test/test_launcher.py` | Default configuration, recorder reuse, and uninterrupted recording when the offscreen GUI closes and reopens |

Core gtests cover profile resolution, catalog parsing, rate calculation and session metadata.
C++ and Python style checks run through the package linters. The checks library has independent
unit tests; the recorder does not attach its semantic checks to recorded topics.

`test_scale.launch.py` retains the optional node timing benchmark. Run timing measurements on
reserved cores. Its environment variables are defined in the test; no previous UI timings apply
to the checkbox tree.
