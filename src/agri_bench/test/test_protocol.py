import math

import pytest

from agri_bench.protocol import Clock, WRAP, apply_calibration, fit_line, parse_line, summarize
from agri_bench.sources import FakeSource, fake_cut_work

NAMES = ['cut', 'grip_left', 'grip_right', 'position']
CAL = {'cut': {'offset': 8000.0, 'scale': 2.0e-4},
       'grip_left': {'offset': -3000.0, 'scale': 2.0e-4},
       'grip_right': {'offset': 1500.0, 'scale': 2.0e-4},
       'position': {'offset': 0.0, 'scale': 0.01}}


def test_parse_line():
    assert parse_line('D,123,1,2,3,4\n', 4) == (123, [1.0, 2.0, 3.0, 4.0])
    assert parse_line('# header', 4) is None
    assert parse_line('D,123,1,2,3', 4) is None       # missing a channel
    assert parse_line('D,12x,1,2,3,4', 4) is None
    assert parse_line('', 4) is None


def test_clock_wraps():
    clock = Clock()
    assert clock.seconds(WRAP - 1_000_000) == 0.0
    assert math.isclose(clock.seconds(WRAP - 500_000), 0.5)
    assert math.isclose(clock.seconds(500_000), 1.5)     # wrapped


def test_fit_line_recovers_calibration():
    points = [(f, f / 2.0e-4 + 8000.0) for f in (0.0, 4.9, 9.8, 19.6)]
    offset, scale, rms = fit_line(points)
    assert math.isclose(offset, 8000.0, abs_tol=1e-6)
    assert math.isclose(scale, 2.0e-4, rel_tol=1e-9)
    assert rms < 1e-9


def test_fit_line_rejects_constant_reading():
    with pytest.raises(ValueError):
        fit_line([(0.0, 100.0), (9.8, 100.0)])


def test_apply_calibration_leaves_unknown_raw():
    assert apply_calibration([8000.0 + 5000.0, 7.0], ['cut', 'extra'], CAL) == \
        pytest.approx([1.0, 7.0])


def test_summary_of_fake_cut():
    peak = 30.0
    source = FakeSource(NAMES, rate_hz=80.0, peak_n=peak, noise_n=0.0, realtime=False)
    clock = Clock()
    times, values = [], []
    for _ in range(int(8.0 * 80)):          # one 8 s cycle
        sample = parse_line(source.readline(), len(NAMES))
        if sample is None:
            continue
        time_us, raw = sample
        times.append(clock.seconds(time_us))
        values.append(apply_calibration(raw, NAMES, CAL))
    s = summarize(times, values, NAMES, 'cut', threshold=0.2)
    assert s['main_channel'] == 'cut'
    assert s['main_peak_n'] == pytest.approx(peak, rel=0.02)
    assert s['grip_left_mean_n'] == pytest.approx((2.0 * 5.0 + 0.5 * 1.5) / 8.0, abs=0.02)
    # Loaded from about 2.12 s (0.2 N) to the cut at 3.5 s.
    assert s['loaded_s'] == pytest.approx(1.5 - 1.5 * math.sqrt(0.2 / peak), abs=0.03)
    assert s['travel_mm'] == pytest.approx(5.0 * s['loaded_s'], abs=0.1)
    assert s['work_mj'] == pytest.approx(fake_cut_work(peak), rel=0.03)
    assert s['rate_hz'] == pytest.approx(80.0, rel=0.01)


def test_summary_empty():
    assert summarize([], [], NAMES, 'cut', 0.2) == {'samples': 0}
