"""The bench linear stage without ROS: commands, the board's replies, motion.

The stage (a ball screw axis with a stepper motor) is driven by the same
board that reads the load cells (firmware/nau7802_logger, second core),
so its position comes in the same data lines, on the same clock, as the
forces: the 'stage' channel, in motor steps, turned into mm by the
calibration (scale = lead / steps per revolution).

Commands, one text line each, sent to the board through force_logger's
~/stage_command topic:

    HOME                   find the home switch; position 0 is where it releases
    MOVE <mm> <mm/s>       move by mm (relative), top speed mm/s
    GOTO <mm> <mm/s>       move to mm (absolute)
    STOP                   slow down and stop
    STATUS                 report homed, moving and the position

Each may end in @<id> (a positive integer); the board then adds id=<id>
to every reply to it, including the done / stopped / error that ends
its move, so a client can tell its replies from stale ones or other
clients'.

MOVE and GOTO need a homed stage and stay within 0..travel. Speeds ramp
up and down at the firmware's acceleration, so a cut at constant speed
has to start at least v^2 / (2 a) before the blade meets the stem.

Replies are comment lines, published on ~/board_message:

    # stage: start pos=<mm> target=<mm>
    # stage: done pos=<mm>
    # stage: stopped pos=<mm>
    # stage: homed
    # stage: status homed=<0|1> moving=<0|1> pos=<mm>
    # stage: error <reason> [pos=<mm>]
"""
import math

STAGE = 'stage'
PREFIX = '# stage:'
MOTION = ('MOVE', 'GOTO')
SIMPLE = ('HOME', 'STOP', 'STATUS')


def command(name, *args, id=None):
    """One command line, checked; MOVE and GOTO take (mm, mm/s)."""
    name = name.upper()
    suffix = '' if id is None else f' @{int(id)}'
    if id is not None and int(id) <= 0:
        raise ValueError('the id must be a positive integer')
    if name in SIMPLE:
        if args:
            raise ValueError(f'{name} takes no arguments')
        return name + suffix
    if name in MOTION:
        if len(args) != 2:
            raise ValueError(f'{name} needs a distance [mm] and a speed [mm/s]')
        mm, speed = (float(a) for a in args)
        if not (math.isfinite(mm) and math.isfinite(speed)) or speed <= 0.0:
            raise ValueError('the distance must be a number and the speed a positive number')
        return f'{name} {mm:.3f} {speed:.3f}{suffix}'
    raise ValueError(f'unknown stage command {name!r}: one of {SIMPLE + MOTION}')


def split_id(line):
    """(words, id) of a command line; id is 0 without a trailing @<id>."""
    words, id = [], 0
    for word in line.strip().upper().split():
        if word.startswith('@') and word[1:].isdigit():
            id = int(word[1:])
        else:
            words.append(word)
    return words, id


def parse_message(line):
    """(event, fields) for a '# stage:' line, else None.

    fields has the key=value numbers, and 'reason' (the other words) for
    errors.
    """
    line = line.strip()
    if not line.startswith(PREFIX):
        return None
    words = line[len(PREFIX):].split()
    if not words:
        return None
    event, fields, text = words[0], {}, []
    for word in words[1:]:
        key, sep, value = word.partition('=')
        if sep:
            try:
                fields[key] = int(value) if key == 'id' else float(value)
                continue
            except ValueError:
                pass
        text.append(word)
    if text:
        fields['reason'] = ' '.join(text)
    return event, fields


class Trapezoid:
    """Move from x0 to x1 [mm] at up to vmax [mm/s] with acceleration a [mm/s^2].

    The same profile as the firmware (constant acceleration, top speed,
    constant deceleration; a triangle when the move is too short).
    """

    def __init__(self, x0, x1, vmax, accel):
        self.x0, self.x1 = x0, x1
        self.sign = 1.0 if x1 >= x0 else -1.0
        d = abs(x1 - x0)
        self.accel = accel
        self.vtop = min(vmax, math.sqrt(accel * d))
        self.t_ramp = self.vtop / accel if accel > 0 else 0.0
        d_ramp = 0.5 * accel * self.t_ramp ** 2
        self.t_flat = (d - 2.0 * d_ramp) / self.vtop if self.vtop > 0 else 0.0
        self.duration = 2.0 * self.t_ramp + self.t_flat
        self.d_ramp = d_ramp

    def distance(self, t):
        """Distance covered after t seconds (not signed)."""
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return abs(self.x1 - self.x0)
        if t < self.t_ramp:
            return 0.5 * self.accel * t ** 2
        if t < self.t_ramp + self.t_flat:
            return self.d_ramp + self.vtop * (t - self.t_ramp)
        left = self.duration - t
        return abs(self.x1 - self.x0) - 0.5 * self.accel * left ** 2

    def position(self, t):
        return self.x0 + self.sign * self.distance(t)

    def speed(self, t):
        if t <= 0.0 or t >= self.duration:
            return 0.0
        if t < self.t_ramp:
            return self.accel * t
        if t < self.t_ramp + self.t_flat:
            return self.vtop
        return self.accel * (self.duration - t)


def ramp_distance(speed, accel):
    """Travel [mm] needed to reach speed [mm/s] from rest."""
    return speed ** 2 / (2.0 * accel)
