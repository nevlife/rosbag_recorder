# Configuration

## Categories

The configuration folder contains `catalog.yaml`. The desktop window starts without a selected
profile or any checked input. Categories are explicit lists of topic names; message types are
not used for classification. The same name belongs to the same category on ROS and Flux.

```yaml
groups:
  lidar:
    topics:
      /velodyne_packets: {}
      /front/scan: {}
      /rear/scan: {}
  vehicle:
    topics:
      /vehicle/custom_status: {}
```

Topics absent from the catalog appear under `Uncategorized`. A custom message type needs no
special category rule. Optional `type`, `hz`, `required`, `heavy`, `latched` and `description`
fields retain their existing meanings. A topic belongs to one group only.

The recorder discovers available inputs independently of the catalog. Checking a category selects
its currently listed children, and each child can be unchecked separately. A new input is left
unchecked when it appears; there is no implicit subscription to future topics in that category.

## Recording

Interactive selections use the internal name `default`, start immediately and use the existing
recording defaults: MCAP, no compression, a file split every 300 seconds, a 100 MiB writer cache,
a warning below 30 GB free and a stop threshold of 10 GB. Path and disk checks must pass before
recording. The bundled `profiles/default.yaml` is an empty profile for service-based startup;
the desktop window applies its checkbox selection through `SetTopics`.

## Scripted profiles

Existing `profiles/<name>.yaml` files and the `Select` and `Start` service APIs remain supported
for scripts. A profile can combine `groups`, `add_groups`, `exclude_groups`, individual `topics`
and `exclude`, and inherit another profile with `extends`. Abstract profiles cannot record.
The desktop window does not list or edit profiles.

Each recorded input is monitored on its own subscription.

## Node parameters

`config/rosbag_recorder.param.yaml` holds every node parameter. The node declares them without
defaults, so a run without this file fails at start. `param_globs` is the exception: it lists the
workspace files that are copied into the session, and it stays empty unless the file sets it,
because a ROS parameter file cannot hold an empty list. `hz_tolerance` must be within 0.1 to 1.0.
