# Configuration

## Categories

The configuration folder contains `catalog.yaml` and `profiles/`. The desktop window starts
without a selected profile or any checked input. Categories are explicit lists of topic names; message types are
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

### Storage preset

MCAP bags are written with one of the rosbag2 MCAP presets:

| `storage_preset` | Layout                                         |
| ---------------- | ---------------------------------------------- |
| `none`           | Chunks with a summary index, no compression    |
| `fastwrite`      | No chunks and no index                         |
| `zstd_fast`      | Chunks with an index, fastest zstd compression |
| `zstd_small`     | Chunks with an index, slowest zstd compression |

`none` keeps an index, so a reader opens the bag in seconds; `fastwrite` has none, so a reader
must scan every file, which takes minutes for a bag of hundreds of GB. A profile sets
`options.storage_preset`. Without it, `compression: zstd` records with `zstd_fast` and
`compression: none` with `none` (earlier builds used `fastwrite`). The Storage preset list in
the desktop window shows the preset of the selection and passes the chosen one as
`Start.storage_preset`, which replaces the profile's preset until a profile or topics are selected
again. `Status.storage_preset` returns the preset of the selection, and `summary.yaml` records
the session's preset as `storage_preset`.

## Scripted profiles

`profiles/<name>.yaml` files are listed in the desktop window, and the `Select` and `Start`
service APIs take them for scripts. A profile can combine `groups`, `add_groups`,
`exclude_groups`, individual `topics` and `exclude`, and inherit another profile with `extends`.
Abstract profiles cannot record and are not listed. Save profile in the window writes a profile
with `name`, `topics` and `options: {start_mode: immediate}`, as the `SaveProfile` service does:

```yaml
name: lidar_only
topics:
  - /velodyne_packets
options:
  start_mode: immediate
```

Each recorded input is monitored on its own subscription.

## Node parameters

`config/rosbag_recorder.param.yaml` holds every node parameter. The node declares them without
defaults, so a run without this file fails at start. `param_globs` is the exception: it lists the
workspace files that are copied into the session, and it stays empty unless the file sets it,
because a ROS parameter file cannot hold an empty list. `hz_tolerance` must be within 0.1 to 1.0.
