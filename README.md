# rosbag_recorder

Select ROS topics and Flux channels in one desktop window and record them into one rosbag2 bag.
Flux is optional; without it the recorder records ROS topics.

## Run

Build in a ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
rosbag_recorder
```

To record Flux channels as well, build with Flux in the workspace and `ENABLE_FLUX=1` set:
`ENABLE_FLUX=1 colcon build --symlink-install`. A build without `ENABLE_FLUX=1` needs no Flux
package and records ROS topics only.

The choice is made when CMake configures the package. colcon does not configure an existing build
directory again when only `ENABLE_FLUX` changes, so a rebuild keeps the earlier choice. To switch,
force the configure step, with or without `ENABLE_FLUX=1`:

```bash
ENABLE_FLUX=1 colcon build --symlink-install --packages-select rosbag_recorder --cmake-force-configure
```

`rosbag_recorder` reuses a running recorder or starts one with
`ros2 launch rosbag_recorder recorder.launch.py`. In a Flux build the launch file runs the node
in a `flux_components` component container, because an rclcpp executor or container delivers no
Flux frame. Otherwise it uses the `component_container` of `rclcpp_components`.

Pick a profile, or check individual topics or an entire category. Topics that have no configured
category appear under `Uncategorized`. Save profile stores the checked topics as a profile. The Storage preset list picks the MCAP preset (`none` by default). Click Preview selection to monitor the selected inputs, or Start to apply
the selection and record. Stop ends the session. Pause, Resume and Split control an open session.
Closing the window leaves the recorder running; run `rosbag_recorder` again to reconnect.

The window shows transport, message type, reception status, Hz, time since the last message,
written count, publishers, output directory and free disk space. It does not infer categories
from message types or apply sensor-specific value checks.

## ROS and Flux

This section applies to a build with `ENABLE_FLUX=1`. ROS messages retain their names and types
in the bag. Flux channel `/camera/image` is recorded as `/camera/image/flux` with type
`flux_msgs/msg/FluxFrame`, using the raw format understood by `flux_bag`. Fingerprint, slot
geometry, array descriptor and frame bytes are preserved, including custom and schemaless inputs.
Recording reads Flux directly rather than through a DDS relay. ROS and Flux inputs with the same
name have separate checkboxes.

```bash
ros2 bag play_flux /path/to/bag
```

Channels whose payload is in GPU memory, such as those of a flux_wrapper `Device::Cuda`
publisher, are recorded too. Flux hands the recorder's CPU subscription a host copy of each frame,
and the bag stores those bytes.

## Configuration

The bundled catalog groups topics by explicit name. A category applies to both transports.
Use `rosbag_recorder --config-dir /path/to/config` to supply another catalog and its
`profiles/`. The bundled folder has one empty `default` profile. Topic selection remains active in
the running recorder when the window is closed and reopened. For scripts,
`rosbag_recorder --profile NAME --autostart --no-gui` records a profile without the window.

See [configuration](docs/configuration.md), [usage](docs/usage.md), and [testing](docs/testing.md).
The profile service APIs remain available for scripted integrations.
