"""Text and colors of the health table, without Qt."""

import math

from rosbag_recorder_msgs.msg import CheckResult, Health, TopicHealth

STATES = {
    Health.IDLE: 'idle',
    Health.PREVIEW: 'preview',
    Health.WAITING: 'waiting',
    Health.RECORDING: 'recording',
    Health.PAUSED: 'paused',
}

STATUS = {
    TopicHealth.OK: ('ok', None),
    TopicHealth.LOW: ('low rate', '#f6d365'),
    TopicHealth.STALE: ('stale', '#f4a261'),
    TopicHealth.WAITING: ('waiting', '#dddddd'),
    TopicHealth.NO_PUBLISHER: ('no publisher', '#e76f51'),
    TopicHealth.LATCHED_MISSING: ('latched missing', '#e76f51'),
    TopicHealth.TYPE_UNAVAILABLE: ('input error', '#e76f51'),
}

LEVELS = {
    CheckResult.OK: 'ok',
    CheckResult.INFO: 'info',
    CheckResult.WARN: 'warn',
    CheckResult.ERROR: 'error',
}


def number(value, digits=1):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return '-'
    return f'{value:.{digits}f}'


def summary(h):
    parts = [STATES.get(h.state, str(h.state))]
    if h.session_id:
        parts.append(f'{h.session_id}  {number(h.elapsed_s, 0)} s')
        parts.append(f'{h.bag_bytes / 1048576:.0f} MiB at {number(h.write_mbps)} MiB/s')
    parts.append(f'disk free {number(h.disk_free_gb)} GB')
    if not math.isnan(h.remaining_min):
        parts.append(f'{h.remaining_min:.0f} min left')
    bad = sum(1 for t in h.topics if t.status != TopicHealth.OK)
    parts.append(f'{len(h.topics) - bad}/{len(h.topics)} topics ok')
    return '  |  '.join(parts + list(h.warnings))


def check_line(c):
    return f'[{LEVELS.get(c.level, c.level)}] {c.text}'
