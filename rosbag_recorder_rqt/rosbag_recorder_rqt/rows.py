# Copyright 2026 nevlife
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

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
    TopicHealth.TYPE_UNAVAILABLE: ('type unavailable', '#e76f51'),
    TopicHealth.ABNORMAL: ('value issue', '#f4a261'),
}

LEVELS = {
    CheckResult.OK: 'ok',
    CheckResult.INFO: 'info',
    CheckResult.WARN: 'warn',
    CheckResult.ERROR: 'error',
}

COLUMNS = ('Topic', 'Source', 'Status', 'Hz', 'Expected', 'Age s', 'Size KB', 'Written',
           'Pub', 'Issues / values')


def number(value, digits=1):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return '-'
    return f'{value:.{digits}f}'


def topic_row(t):
    status, color = STATUS.get(t.status, (str(t.status), None))
    if t.required and t.status != TopicHealth.OK:
        status += ' (required)'
    cells = (
        t.name,
        t.source,
        status,
        number(t.hz),
        number(t.expected_hz) if t.expected_hz > 0 else '-',
        number(t.age_s),
        number(t.size_kb, 2),
        str(t.written),
        str(t.publishers),
        '; '.join(list(t.issues) + list(t.values)),
    )
    return cells, color


def summary(h):
    parts = [STATES.get(h.state, str(h.state))]
    if h.profile:
        parts.append(f'profile {h.profile}')
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
