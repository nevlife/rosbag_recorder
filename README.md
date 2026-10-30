# rosbag_recorder

Select ROS topics and Flux channels in one desktop window and record them into one rosbag2 bag.

## Run

Build in a ROS 2 workspace with Flux installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
rosbag_recorder
```

Check individual topics or an entire category. Topics that have no configured category appear
under `Uncategorized`. Click Preview selection to monitor the selected inputs, or Start to apply
the selection and record. Stop ends the session. Pause, Resume and Split control an open session.
Closing the window leaves the recorder running; run `rosbag_recorder` again to reconnect.

The window shows transport, message type, reception status, Hz, time since the last message,
written count, publishers, output directory and free disk space. It does not infer categories
from message types or apply sensor-specific value checks.

## ROS and Flux

ROS messages retain their names and types in the bag. Flux channel `/camera/image` is recorded
as `/camera/image/flux` with type `flux_msgs/msg/FluxFrame`, using the raw format understood by
`flux_bag`. Fingerprint, slot geometry, array descriptor and frame bytes are preserved, including
custom and schemaless inputs. Recording reads Flux directly rather than through a DDS relay.
ROS and Flux inputs with the same name have separate checkboxes.

```bash
ros2 bag play_flux /path/to/bag
```

CPU recording requires a host-accessible Flux payload. Device-only CUDA payloads require a GPU
copy path and are not supported by this recorder.

## Configuration

The bundled catalog groups topics by explicit name. A category applies to both transports.
Use `rosbag_recorder --config-dir /path/to/config` to supply another catalog. There is one empty
`default` profile; topics are chosen in the window. Topic selection remains active in the running
recorder when the window is closed and reopened.

See [configuration](docs/configuration.md), [usage](docs/usage.md), and [testing](docs/testing.md).
The older profile and source-plugin service APIs remain available for scripted integrations.
