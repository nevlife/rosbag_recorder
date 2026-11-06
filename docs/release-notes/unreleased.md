# rosbag_recorder (unreleased)

<!--
    Add an entry with every change. At release this file becomes v?.?.?.md (see template.md)
    and the package CHANGELOG.rst files get the same entries.
-->

Everything below is new since there is no release yet.

## Features

* Session, health and marker messages and the control services (rosbag_recorder_msgs)
* Plugin interface for recording data that is not a ROS topic (rosbag_recorder_source)
* Value check plugin interface and the built-in checks `HeaderStamp`, `Imu`, `NavSatFix`, `TfMessage`, `PointCloud2` (rosbag_recorder_checks)
* Recording node with profiles, one shared subscription per topic and session channels in the bag (rosbag_recorder)
* rqt plugin for profile, start, stop, markers and the health table (rosbag_recorder_rqt)
* `Health.graph_reads` counts the ticks that re-read the graph (rosbag_recorder_msgs)

## Bugfixes

* Do not publish the end session after shutdown (rosbag_recorder)
* Build and test on Humble (rosbag_recorder)
* Count stored messages in `summary.yaml` and report what the writer cache dropped (rosbag_recorder)

## Refactoring

* Query the graph only when it changes and cache the profile list (rosbag_recorder)
* Update table cells in place and only while the table is visible (rosbag_recorder_rqt)

## Workflow

* Responsiveness tests for the node and the rqt plugin with budgets from environment variables
* The launch test removes its bags
* README split into docs/; rosdoc2 configuration for the API reference; package CHANGELOG.rst files
