# SCV compatibility

`catalog.yaml` and `profiles/*.yaml` follow the SCV `scv_logger` format exactly, so an existing
SCV configuration folder can be passed as `config_dir` without changes.

## Shared with SCV

- Catalog: groups of topics with `type`, `hz`, `required`, `heavy`, `size_kb`, `latched`,
  `description`, `monitor_via`; `version`, `heavy_threshold_kb`, `auto_markers`,
  `value_checks`.
- Profiles: `extends`, `abstract`, `groups`, `add_groups`, `exclude_groups`, `topics`,
  `exclude`.
- Options: `storage`, `compression`, `split_duration_sec`, `split_size_mb`, `heavy_qos`,
  `start_mode`, `wait_ready_sec`, `start_delay_sec`, `disk_warn_gb`, `disk_stop_gb`,
  `max_cache_mb`, with the same defaults.

## Differences

- `monitor_via` is accepted and not used, because every recorded topic is measured on its own
  subscription.
- Added keys, all optional: catalog `sources`; profile `sources`, `exclude_sources`; options
  `storage_preset`, `storage_config_uri`, `on_not_ready`, `required_frames`. A folder that uses
  them is no longer valid for SCV.

## How it is checked

`rosbag_recorder/test/data/scv` holds SCV's catalog and profiles as fixtures, and
`resolved.yaml` the profiles that SCV's own code (`core/profile.py`) resolves from them. The
gtest `ScvConfig.EveryProfileResolvesLikeScv` resolves every profile and compares the chain,
`abstract`, topics, unknown topics and options with that file.

To check an original SCV tree (Korean descriptions included), point
`ROSBAG_RECORDER_SCV_CONFIG` at it; `ScvConfig.OriginalFiles` then compares the topics of every
profile with `resolved.yaml`. It is skipped when the variable is not set.
