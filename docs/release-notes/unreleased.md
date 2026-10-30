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
* Recording node with SCV profiles, one shared subscription per topic and session channels in the bag (rosbag_recorder)
* rqt window with category and individual ROS/Flux checkboxes, recording controls and reception statistics (rosbag_recorder_rqt)
* Direct raw Flux recording in the format supported by flux_bag, including unknown custom fingerprints (rosbag_recorder)
* One bundled default profile; automatic sensor value checks and automatic value markers are no longer attached (rosbag_recorder)
* `Health.graph_reads` counts the ticks that re-read the graph (rosbag_recorder_msgs)

## Bugfixes

* Do not publish the end session after shutdown (rosbag_recorder)
* Build and test on Humble (rosbag_recorder)

## Refactoring

* Query the graph only when it changes and cache the profile list (rosbag_recorder)
* Preserve checkbox choices and topic items across discovery and statistics updates (rosbag_recorder_rqt)

## Workflow

* Node timing benchmark and a 500-topic checkbox consistency test
* The launch test removes its bags
* README split into docs/; rosdoc2 configuration for the API reference; package CHANGELOG.rst files
