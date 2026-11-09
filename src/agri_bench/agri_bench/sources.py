"""Where the data lines come from: the board's serial port, or a fake board.

Both return the firmware's text lines (see protocol.py), so the fake one
exercises the same parsing and calibration as the real one.
"""
import random
import threading
import time

from agri_bench.protocol import POSITION, WRAP
from agri_bench.stage import MOTION, STAGE, Trapezoid, split_id


class SerialSource:
    """The board over USB serial, or any pyserial URL.

    port is a device (/dev/ttyACM0) or a URL such as
    socket://host:7000 or rfc2217://host:7000, for a board plugged into
    another machine (Docker Desktop on macOS cannot pass USB devices to a
    container).
    """

    def __init__(self, port, baud):
        import serial  # only needed with a real board
        self.port = serial.serial_for_url(port, baudrate=baud, timeout=0.5)

    def readline(self):
        return self.port.readline().decode('ascii', errors='replace')

    def write(self, line):
        self.port.write((line.strip() + '\n').encode('ascii'))

    def close(self):
        self.port.close()


class FakeStage:
    """The linear stage of the fake board: same commands and replies as the
    firmware, same trapezoid moves and whole motor steps, but no switches,
    and STOP stops at once.
    """

    def __init__(self, travel_mm=100.0, accel=1000.0, max_speed=200.0, home_speed=10.0,
                 start_mm=30.0, steps_per_mm=160.0):
        self.travel, self.accel, self.max_speed = travel_mm, accel, max_speed
        self.home_speed = home_speed
        self.steps_per_mm = steps_per_mm
        self.pos = start_mm
        self.homed = False
        self.move = None      # (start time, Trapezoid, event when done, id)

    def whole_steps(self, mm):
        return round(mm * self.steps_per_mm) / self.steps_per_mm

    @staticmethod
    def tag(text, id):
        return f'{text} id={id}' if id else text

    def at(self, t):
        """Position [mm] at time t, and the replies due by then."""
        if self.move is None:
            return self.pos, []
        t0, path, done, id = self.move
        self.pos = path.position(t - t0)
        if t - t0 < path.duration:
            return self.pos, []
        self.move = None
        self.pos = path.x1
        if done == 'homed':
            self.pos, self.homed = 0.0, True
            return self.pos, [self.tag('homed', id)]
        return self.pos, [self.tag(f'done pos={self.pos:.3f}', id)]

    def command(self, line, t):
        """Replies to one command line received at time t."""
        words, id = split_id(line)
        if not words:
            return []
        name, args = words[0], words[1:]
        _, out = self.at(t)       # a move that ended by now reports first
        busy = self.move is not None

        def say(text):
            return out + [self.tag(text, id)]
        if name == 'STATUS' and not args:
            return say(f'status homed={int(self.homed)} moving={int(busy)} pos={self.pos:.3f}')
        if name == 'STOP' and not args:
            if busy:
                self.pos = self.whole_steps(self.pos)
                if self.move[2] == 'homed':
                    self.homed = False
                if self.move[3] != id:
                    out = out + [self.tag(f'stopped pos={self.pos:.3f}', self.move[3])]
            self.move = None
            return say(f'stopped pos={self.pos:.3f}')
        if name == 'HOME' and not args:
            if busy:
                return say('error busy')
            self.homed = False
            self.move = (t, Trapezoid(self.pos, 0.0, self.home_speed, self.accel), 'homed', id)
            return say(f'start pos={self.pos:.3f} target=0.000')
        if name not in MOTION:
            return say('error unknown command')
        try:
            mm, speed = (float(a) for a in args)
        except ValueError:
            return say('error usage: MOVE|GOTO <mm> <mm/s>')
        if busy:
            return say('error busy')
        if not self.homed:
            return say('error not homed')
        if not 0.0 < speed <= self.max_speed:
            return say('error speed')
        target = self.whole_steps(mm if name == 'GOTO' else self.pos + mm)
        if not 0.0 <= target <= self.travel:
            return say(f'error out of range target={target:.3f}')
        self.move = (t, Trapezoid(self.pos, target, speed, self.accel), 'done', id)
        return say(f'start pos={self.pos:.3f} target={target:.3f}')


class FakeSource:
    """A board with a cut load cell, two grip load cells and a position pot.

    It repeats an 8 s cut: the blade moves at 5 mm/s from 1.5 s to 5.5 s,
    touches the stem at 2.0 s, and the force rises as (s / 1.5)^2 to
    peak_n until the stem parts at 3.5 s. Over that 7.5 mm of travel the
    work is peak_n * 2.5 mJ. The fingers close on the stem at 1.0 s with
    grip_n each, feel half a newton more while it is cut, and open at
    6.0 s.

    With a 'stage' channel the blade is moved by the stage instead (see
    FakeStage), and nothing happens until it is told to move: the stem
    is met at CONTACT_MM, the force rises as (s / 7.5 mm)^2 to peak_n and
    the stem parts 7.5 mm further (work peak_n * 2.5 mJ again, at any
    speed). A new stem is put in once the stage goes back below
    CONTACT_MM - 5 mm. The fingers hold grip_n each all the time, half a
    newton more while cutting, and the position pot follows the stage.

    Raw counts use the calibration in config/calibration_example.yaml.
    """

    CAL = {'cut': (8000.0, 2.0e-4), 'grip_left': (-3000.0, 2.0e-4),
           'grip_right': (1500.0, 2.0e-4), 'position': (0.0, 0.01),
           'stage': (0.0, 0.00625)}
    PERIOD = 8.0
    CONTACT_MM = 40.0
    CUT_MM = 7.5

    def __init__(self, names, rate_hz=320.0, peak_n=30.0, grip_n=2.0, noise_n=0.02, seed=0,
                 realtime=True):
        self.names = names
        self.dt = 1.0 / rate_hz
        self.peak_n, self.grip_n, self.noise_n = peak_n, grip_n, noise_n
        self.random = random.Random(seed)
        self.start = time.monotonic()
        self.next = self.start
        self.header_sent = False
        self.realtime = realtime   # False: no waiting, for tests
        self.stage = FakeStage() if STAGE in names else None
        self.stem_cut = False
        self.lock = threading.Lock()
        self.pending = []          # comment lines to send before the next sample

    def now(self):
        return self.next - self.start

    def write(self, line):
        if self.stage is None:
            self.pending.append('# stage: error no stage\n')
            return
        with self.lock:
            for reply in self.stage.command(line, self.now()):
                self.pending.append(f'# stage: {reply}\n')

    def staged(self, name, x):
        """Physical value with the blade on the stage at x [mm]."""
        into = x - self.CONTACT_MM
        cutting = not self.stem_cut and 0.0 <= into < self.CUT_MM
        if name == 'cut':
            return self.peak_n * (into / self.CUT_MM) ** 2 if cutting else 0.0
        if name.startswith('grip'):
            return self.grip_n + (0.5 if cutting else 0.0)
        if name in (POSITION, STAGE):
            return x
        return 0.0

    def physical(self, name, t):
        s = t % self.PERIOD
        cutting = 2.0 <= s < 3.5
        if name == 'cut':
            return self.peak_n * ((s - 2.0) / 1.5) ** 2 if cutting else 0.0
        if name.startswith('grip'):
            return self.grip_n + (0.5 if cutting else 0.0) if 1.0 <= s < 6.0 else 0.0
        if name == 'position':
            return 5.0 * min(max(s - 1.5, 0.0), 4.0) if s < 5.5 else max(20.0 - 10.0 * (s - 5.5), 0.0)
        return 0.0

    def readline(self):
        if not self.header_sent:
            self.header_sent = True
            return f'# agri_bench fake board, channels={len(self.names)}\n'
        with self.lock:
            if self.pending:
                return self.pending.pop(0)
        now = time.monotonic()
        if self.realtime and self.next > now:
            time.sleep(self.next - now)
        with self.lock:
            t = self.now()
            self.next += self.dt
            if self.stage is not None:
                x, replies = self.stage.at(t)
                self.pending += [f'# stage: {r}\n' for r in replies]
                if not self.stem_cut and x >= self.CONTACT_MM + self.CUT_MM:
                    self.stem_cut = True
                    self.pending.append('# fake: stem cut\n')
                elif self.stem_cut and x < self.CONTACT_MM - 5.0:
                    self.stem_cut = False
                    self.pending.append('# fake: new stem\n')
        raws = []
        for name in self.names:
            offset, scale = self.CAL.get(name, (0.0, 1.0))
            value = self.staged(name, x) if self.stage is not None else self.physical(name, t)
            if name not in (POSITION, STAGE):
                value += self.random.gauss(0.0, self.noise_n)
            raws.append(str(int(round(value / scale + offset))))
        time_us = int(t * 1e6) % WRAP
        return 'D,' + ','.join([str(time_us)] + raws) + '\n'

    def close(self):
        pass


def fake_cut_work(peak_n):
    """Work [mJ] of one FakeSource cut, for checking the summary."""
    # integral of peak (s/1.5)^2 * 5 mm/s ds over 0..1.5 s
    return peak_n * 5.0 * 1.5 / 3.0

