import math

import pytest

from agri_bench.protocol import Clock, apply_calibration, parse_line, summarize
from agri_bench.sources import FakeSource, FakeStage, fake_cut_work
from agri_bench.stage import Trapezoid, command, parse_message, ramp_distance

NAMES = ['cut', 'grip_left', 'grip_right', 'position', 'stage']
CAL = {'cut': {'offset': 8000.0, 'scale': 2.0e-4},
       'grip_left': {'offset': -3000.0, 'scale': 2.0e-4},
       'grip_right': {'offset': 1500.0, 'scale': 2.0e-4},
       'position': {'offset': 0.0, 'scale': 0.01},
       'stage': {'offset': 0.0, 'scale': 0.00625}}


def test_command_format_and_checks():
    assert command('home') == 'HOME'
    assert command('MOVE', 10, 5) == 'MOVE 10.000 5.000'
    assert command('goto', '-1.5', 20) == 'GOTO -1.500 20.000'
    with pytest.raises(ValueError):
        command('MOVE', 10)
    with pytest.raises(ValueError):
        command('MOVE', 10, 0)
    with pytest.raises(ValueError):
        command('STOP', 1)
    with pytest.raises(ValueError):
        command('JUMP')


def test_parse_message():
    assert parse_message('# stage: done pos=12.500') == ('done', {'pos': 12.5})
    assert parse_message('# stage: status homed=1 moving=0 pos=0.000') == \
        ('status', {'homed': 1.0, 'moving': 0.0, 'pos': 0.0})
    assert parse_message('# stage: error limit switch pos=3.000') == \
        ('error', {'pos': 3.0, 'reason': 'limit switch'})
    assert parse_message('# stage: error usage: MOVE|GOTO <mm> <mm/s>')[1]['reason'] == \
        'usage: MOVE|GOTO <mm> <mm/s>'
    assert parse_message('# agri_bench header') is None
    assert parse_message('D,1,2,3') is None


def test_trapezoid():
    path = Trapezoid(10.0, 40.0, 50.0, 1000.0)
    assert path.position(0.0) == 10.0
    assert path.position(path.duration) == pytest.approx(40.0)
    assert path.speed(path.duration / 2) == pytest.approx(50.0)
    # ramp 1.25 mm each way, 27.5 mm at 50 mm/s
    assert path.duration == pytest.approx(2 * 0.05 + 27.5 / 50.0)
    back = Trapezoid(40.0, 10.0, 50.0, 1000.0)
    assert back.position(back.duration / 2) == pytest.approx(25.0)
    short = Trapezoid(0.0, 1.0, 200.0, 1000.0)    # too short to reach 200 mm/s
    assert short.vtop == pytest.approx(math.sqrt(1000.0))
    assert short.position(short.duration) == pytest.approx(1.0)
    assert ramp_distance(150.0, 1000.0) == pytest.approx(11.25)


def test_fake_stage_rules():
    stage = FakeStage(start_mm=30.0)
    assert stage.command('MOVE 5 5', 0.0) == ['error not homed']
    assert stage.command('HOME', 0.0)[0].startswith('start')
    assert stage.command('MOVE 5 5', 0.1) == ['error busy']
    pos, replies = stage.at(10.0)
    assert (pos, replies) == (0.0, ['homed'])
    assert stage.command('GOTO 120 5', 10.0)[0].startswith('error out of range')
    assert stage.command('GOTO 20 500', 10.0) == ['error speed']
    assert stage.command('GOTO 20 50', 10.0)[0] == 'start pos=0.000 target=20.000'
    assert stage.command('STOP', 10.1)[0].startswith('stopped')
    assert stage.command('STATUS', 10.2)[0].startswith('status homed=1 moving=0')


def record(source, seconds):
    """Samples and stage replies for seconds of fake data."""
    clock = Clock()
    times, values, replies = [], [], []
    end = source.now() + seconds
    while source.now() < end:
        line = source.readline()
        message = parse_message(line)
        if message is not None:
            replies.append(message)
            continue
        sample = parse_line(line, len(NAMES))
        if sample is None:
            continue
        time_us, raw = sample
        times.append(clock.seconds(time_us))
        values.append(apply_calibration(raw, NAMES, CAL))
    return times, values, replies


@pytest.mark.parametrize('speed', [5.0, 50.0])
def test_stage_driven_fake_cut(speed):
    peak = 30.0
    source = FakeSource(NAMES, peak_n=peak, noise_n=0.0, realtime=False)
    source.write('HOME')
    record(source, 4.0)
    source.write('GOTO 25 50')            # 15 mm before the stem
    record(source, 1.0)
    source.write(f'MOVE 30 {speed}')      # through the stem at 40..47.5 mm
    times, values, replies = record(source, 30.0 / speed + 1.0)
    assert ('done', {'pos': 55.0}) in replies
    s = summarize(times, values, NAMES, 'cut', threshold=0.2)
    assert s['travel_channel'] == 'position'
    assert s['main_peak_n'] == pytest.approx(peak, rel=0.05)
    assert s['work_mj'] == pytest.approx(fake_cut_work(peak), rel=0.06)
    assert s['travel_mm'] == pytest.approx(7.5 * (1 - math.sqrt(0.2 / peak)), abs=0.7)
    # Constant speed through the stem: the ramp ended 15 mm before it.
    assert s['travel_mm'] / s['loaded_s'] == pytest.approx(speed, rel=0.1)


def test_stage_alone_gives_travel():
    source = FakeSource(NAMES, noise_n=0.0, realtime=False)
    source.write('HOME')
    record(source, 4.0)
    source.write('GOTO 30 50')
    record(source, 1.0)
    source.write('MOVE 20 10')
    times, values, _ = record(source, 3.0)
    names = ['cut', 'grip_left', 'grip_right', 'stage']
    values = [[v[0], v[1], v[2], v[4]] for v in values]   # no potentiometer
    s = summarize(times, values, names, 'cut', threshold=0.2)
    assert s['travel_channel'] == 'stage'
    assert s['work_mj'] == pytest.approx(fake_cut_work(30.0), rel=0.06)


def test_new_stem_after_going_back():
    source = FakeSource(NAMES, noise_n=0.0, realtime=False)
    source.write('HOME')
    record(source, 4.0)
    source.write('GOTO 50 50')
    _, _, replies = record(source, 2.0)
    source.write('GOTO 20 50')
    record(source, 2.0)
    assert source.stem_cut is False


def test_no_stage_channel():
    source = FakeSource(NAMES[:4], realtime=False)
    source.write('HOME')
    source.readline()                     # header
    assert source.readline().startswith('# stage: error no stage')


def test_fake_stage_ids_and_late_done():
    stage = FakeStage(start_mm=30.0)
    stage.command('HOME @7', 0.0)
    # STATUS after the homing ended, before anyone sampled: the homed
    # reply must still come out, first.
    replies = stage.command('STATUS @8', 10.0)
    assert replies == ['homed id=7', 'status homed=1 moving=0 pos=0.000 id=8']
    assert stage.command('GOTO 20 50 @9', 10.0) == ['start pos=0.000 target=20.000 id=9']
    # STOP from another client: both hear it (ramp 1.25 mm + 0.05 s at 50 mm/s).
    assert stage.command('STOP @10', 10.1) == ['stopped pos=3.750 id=9',
                                               'stopped pos=3.750 id=10']


def test_fake_stage_whole_steps():
    stage = FakeStage(start_mm=30.0)
    stage.command('HOME', 0.0)
    stage.at(10.0)
    for i in range(10):
        stage.command('MOVE 0.001 1', 10.0 + i)
        stage.at(10.5 + i)
    assert stage.pos == 0.0          # each move is less than half a step (1/160 mm)


def test_command_ids():
    from agri_bench.stage import split_id
    assert command('MOVE', 1, 2, id=42) == 'MOVE 1.000 2.000 @42'
    assert split_id('move 1 2 @42') == (['MOVE', '1', '2'], 42)
    assert parse_message('# stage: done pos=1.000 id=42') == ('done', {'pos': 1.0, 'id': 42})
    with pytest.raises(ValueError):
        command('MOVE', float('nan'), 5)


def test_trial_refuses_old_header(tmp_path):
    from agri_bench.trial import check_header
    path = tmp_path / 'trials.csv'
    path.write_text('trial_id,datetime,main_peak_n\n')
    with pytest.raises(SystemExit):
        check_header(str(path))
