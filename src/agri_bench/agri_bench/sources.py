"""Where the data lines come from: the board's serial port, or a fake board.

Both return the firmware's text lines (see protocol.py), so the fake one
exercises the same parsing and calibration as the real one.
"""
import random
import time

from agri_bench.protocol import WRAP


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

    def close(self):
        self.port.close()


class FakeSource:
    """A board with a cut load cell, two grip load cells and a position pot.

    It repeats an 8 s cut: the blade moves at 5 mm/s from 1.5 s to 5.5 s,
    touches the stem at 2.0 s, and the force rises as (s / 1.5)^2 to
    peak_n until the stem parts at 3.5 s. Over that 7.5 mm of travel the
    work is peak_n * 2.5 mJ. The fingers close on the stem at 1.0 s with
    grip_n each, feel half a newton more while it is cut, and open at
    6.0 s. Raw counts use the
    calibration in config/calibration_example.yaml.
    """

    CAL = {'cut': (8000.0, 2.0e-4), 'grip_left': (-3000.0, 2.0e-4),
           'grip_right': (1500.0, 2.0e-4), 'position': (0.0, 0.01)}
    PERIOD = 8.0

    def __init__(self, names, rate_hz=80.0, peak_n=30.0, grip_n=2.0, noise_n=0.02, seed=0,
                 realtime=True):
        self.names = names
        self.dt = 1.0 / rate_hz
        self.peak_n, self.grip_n, self.noise_n = peak_n, grip_n, noise_n
        self.random = random.Random(seed)
        self.start = time.monotonic()
        self.next = self.start
        self.header_sent = False
        self.realtime = realtime   # False: no waiting, for tests

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
        now = time.monotonic()
        if self.realtime and self.next > now:
            time.sleep(self.next - now)
        t = self.next - self.start
        self.next += self.dt
        raws = []
        for name in self.names:
            offset, scale = self.CAL.get(name, (0.0, 1.0))
            value = self.physical(name, t)
            if name != 'position':
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

