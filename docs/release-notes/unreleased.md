# rosbag_recorder (unreleased)

<!--
    Add an entry with every change. At release this file becomes v?.?.?.md (see template.md)
    and the package CHANGELOG.rst files get the same entries.
-->

Everything below is new since there is no release yet.

## Features

- Session, health and marker messages and the control services (rosbag_recorder_msgs)
- Recording node with profiles, one shared subscription per topic and session channels in the bag (rosbag_recorder)
- rqt window with category and individual ROS/Flux checkboxes, recording controls and reception statistics (rosbag_recorder_rqt)
- Direct raw Flux recording in the format supported by flux_bag, including unknown custom fingerprints (rosbag_recorder)
- One bundled default profile; automatic sensor value checks and automatic value markers are no longer attached (rosbag_recorder)
- `Health.graph_reads` counts the ticks that re-read the graph (rosbag_recorder_msgs)
- `Status.health` returns the last health report (rosbag_recorder_msgs)
- Profile list and Save profile in the rqt window; the `SaveProfile` service writes the checked ROS topics as a profile in the recorder's config folder (rosbag_recorder, rosbag_recorder_msgs, rosbag_recorder_rqt)
- Storage preset list in the rqt window; `Start.storage_preset` replaces the profile's MCAP preset, `Status.storage_preset` returns it, and `summary.yaml` records it as `storage_preset` (rosbag_recorder, rosbag_recorder_msgs, rosbag_recorder_rqt)
- Flux is optional at build time: `ENABLE_FLUX=1` builds the Flux input, and a build without it records ROS topics in the `rclcpp_components` container and needs no Flux package (rosbag_recorder)
- Record Flux channels in GPU memory from the host copy that Flux gives a CPU subscription (rosbag_recorder)

## Bugfixes

- Build and test on Humble (rosbag_recorder)
- Count stored messages in `summary.yaml` and report what the writer cache dropped (rosbag_recorder)
- An MCAP profile without `storage_preset` and without compression records with the `none` preset instead of `fastwrite`: `none` writes chunks with a summary index, so a reader opens the bag without scanning it, and `fastwrite` wrote no index (rosbag_recorder)
- Record a Flux frame's aux bytes and the fields in its publisher's arena, through flux_bag's `FrameEncoder`; they were left out of the bag (rosbag_recorder)

## Refactoring

- Remove the unused source plugin interface (rosbag_recorder_source) and value check library (rosbag_recorder_checks), with the catalog `sources` and profile `sources`/`exclude_sources` keys (rosbag_recorder)
- Run in a `flux_components` container from `recorder.launch.py`; Flux inputs are subscriptions in the node's callback group instead of reader threads, and the node runs on one callback group without locks (rosbag_recorder, rosbag_recorder_rqt)
- Publish no topics: session, health and marker messages go only into the bag, still on `/rosbag_recorder/{session,health,markers}`; the rqt window reads health from the `Status` service once per second instead of subscribing (rosbag_recorder, rosbag_recorder_rqt)
- Read every parameter from `config/rosbag_recorder.param.yaml`; reject `hz_tolerance` outside 0.1 to 1.0 instead of clamping it (rosbag_recorder)
- Move preflight, start modes, disk watch, summary, catalog diff and selection checks into ROS-free classes with unit tests; headers move to `src/` (rosbag_recorder)
- Reject the unused catalog keys `heavy_threshold_kb`, `monitor_via`, `auto_markers`, `value_checks` and `size_kb` (rosbag_recorder)
- Fail instead of recording empty values when git, `statvfs`, the directory size or the user name cannot be read (rosbag_recorder)

- Query the graph only when it changes and cache the profile list (rosbag_recorder)
- Preserve checkbox choices and topic items across discovery and statistics updates (rosbag_recorder_rqt)

## Workflow

- Node timing benchmark and a 500-topic checkbox consistency test
- The launch test removes its bags
- A launch test records a flux_wrapper `device="cuda"` Image publisher
- README split into docs/; rosdoc2 configuration for the API reference; package CHANGELOG.rst files
