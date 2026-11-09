"""Bench data handling without ROS: serial lines, calibration, trial summary.

The firmware (firmware/nau7802_logger) sends one text line per sample:

    D,<time_us>,<raw_0>,...,<raw_n-1>

with the raw ADC counts in the order of the channels parameter. Lines
starting with '#' are comments (header, errors). time_us is the board's
microsecond counter, a 32-bit value that wraps after about 71 minutes.

Each channel is turned into a physical value as (raw - offset) * scale:
force channels in N, and in mm the optional 'position' channel (travel
from a linear potentiometer) and 'stage' channel (the linear stage's
motor steps, see stage.py).
"""
import math

POSITION = 'position'
STAGE = 'stage'
NON_FORCE = (POSITION, STAGE)   # channels in mm, not N; never tared
WRAP = 2 ** 32


def parse_line(line, count):
    """(time_us, [raw]) from a data line, or None for comments and bad lines."""
    line = line.strip()
    if not line.startswith('D,'):
        return None
    fields = line.split(',')
    if len(fields) != count + 2:
        return None
    try:
        return int(fields[1]), [float(v) for v in fields[2:]]
    except ValueError:
        return None


class Clock:
    """Board microseconds to seconds since the first sample, across wraps."""

    def __init__(self):
        self.first = None
        self.last = None
        self.wraps = 0

    def seconds(self, time_us):
        if self.first is None:
            self.first = time_us
        elif time_us < self.last:
            self.wraps += 1
        self.last = time_us
        return (time_us + self.wraps * WRAP - self.first) * 1e-6


def apply_calibration(raw, names, calibration):
    """Physical values for raw counts; channels without calibration stay raw."""
    out = []
    for name, value in zip(names, raw):
        cal = calibration.get(name)
        out.append(value if cal is None else (value - cal['offset']) * cal['scale'])
    return out


def fit_line(points):
    """offset, scale and rms error for value = (raw - offset) * scale.

    points: [(known_value, mean_raw)], at least two different raw values.
    """
    n = len(points)
    if n < 2:
        raise ValueError('need at least two points')
    mx = sum(r for _, r in points) / n
    my = sum(v for v, _ in points) / n
    sxx = sum((r - mx) ** 2 for _, r in points)
    if sxx == 0:
        raise ValueError('all raw readings are the same: is the load cell connected?')
    scale = sum((r - mx) * (v - my) for v, r in points) / sxx
    if scale == 0:
        raise ValueError('readings do not change with the load')
    offset = mx - my / scale
    rms = math.sqrt(sum(((r - offset) * scale - v) ** 2 for v, r in points) / n)
    return offset, scale, rms


def summarize(times, values, names, main, threshold):
    """Trial summary from recorded samples.

    times: [s]; values: one list of physical values per sample, in the
    order of names. For every force channel: peak and mean. For the main
    channel (the one being cut or pulled): its peak again as main_peak_n,
    and the loaded part of the trial,
    from where it first exceeds threshold to where it last does, with its
    duration and, if there is a position channel (else the stage
    channel), the travel and the work (integral of force over travel, in
    mJ = N*mm).
    """
    if not times:
        return {'samples': 0}
    summary = {'samples': len(times), 'duration_s': round(times[-1] - times[0], 3)}
    if len(times) > 1:
        summary['rate_hz'] = round((len(times) - 1) / (times[-1] - times[0]), 1)
    for i, name in enumerate(names):
        if name in NON_FORCE:
            continue
        col = [v[i] for v in values]
        summary[f'{name}_peak_n'] = round(max(col), 3)
        summary[f'{name}_mean_n'] = round(sum(col) / len(col), 3)
    if main not in names:
        return summary
    m = names.index(main)
    summary['main_channel'] = main
    summary['main_peak_n'] = summary.get(f'{main}_peak_n')
    loaded = [k for k, v in enumerate(values) if v[m] > threshold]
    if not loaded:
        return summary
    a, b = loaded[0], loaded[-1]
    summary['loaded_s'] = round(times[b] - times[a], 3)
    travel = next((n for n in (POSITION, STAGE) if n in names), None)
    if travel is not None:
        p = names.index(travel)
        summary['travel_channel'] = travel
        summary['travel_mm'] = round(values[b][p] - values[a][p], 3)
        work = 0.0
        for k in range(a, b):
            dx = values[k + 1][p] - values[k][p]
            work += 0.5 * (values[k][m] + values[k + 1][m]) * dx
        summary['work_mj'] = round(work, 2)
    return summary
