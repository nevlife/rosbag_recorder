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

## Bugfixes

- Do not publish the end session after shutdown (rosbag_recorder)
- Build and test on Humble (rosbag_recorder)
- Count stored messages in `summary.yaml` and report what the writer cache dropped (rosbag_recorder)

## Refactoring

- Remove the unused source plugin interface (rosbag_recorder_source) and value check library (rosbag_recorder_checks), with the catalog `sources` and profile `sources`/`exclude_sources` keys (rosbag_recorder)
- Run in a `flux_components` container from `recorder.launch.py`; Flux inputs are subscriptions in the node's callback group instead of reader threads, and the node runs on one callback group without locks (rosbag_recorder, rosbag_recorder_rqt)
- Publish on `~/output/{session,health,markers}`, remapped by the launch file to the unchanged `/rosbag_recorder/...` names (rosbag_recorder)
- Read every parameter from `config/rosbag_recorder.param.yaml`; reject `hz_tolerance` outside 0.1 to 1.0 instead of clamping it (rosbag_recorder)
- Move preflight, start modes, disk watch, summary, catalog diff and selection checks into ROS-free classes with unit tests; headers move to `src/` (rosbag_recorder)
- Reject the unused catalog keys `heavy_threshold_kb`, `monitor_via`, `auto_markers`, `value_checks` and `size_kb` (rosbag_recorder)
- Fail instead of recording empty values when git, `statvfs`, the directory size or the user name cannot be read (rosbag_recorder)

- Query the graph only when it changes and cache the profile list (rosbag_recorder)
- Preserve checkbox choices and topic items across discovery and statistics updates (rosbag_recorder_rqt)

## Workflow

- Node timing benchmark and a 500-topic checkbox consistency test
- The launch test removes its bags
- README split into docs/; rosdoc2 configuration for the API reference; package CHANGELOG.rst files
