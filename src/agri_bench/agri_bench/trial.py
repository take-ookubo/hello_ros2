"""Record one bench trial and add it to <output_dir>/trials.csv.

    ros2 run agri_bench trial --test A --variety momotaro --ripeness red \\
        --diameter 3.1 3.3 --position distal --angle 90 --speed 5

With --tare it first zeroes the force channels; do that only with
nothing on the load cells (better: call /force_logger/tare once before
mounting the sample). It then starts force_logger,
waits for Enter (or --duration seconds), stops it, asks for the result
(unless --result) and writes one row with the sample data from the
command line and the force summary. The columns follow section 6 of
docs/stem_force_test_plan.md; the raw samples stay in the file named in
raw_file.

With --stage-move MM the linear stage does the trial instead of Enter:
it moves MM at --speed (the blade speed) times --lever-ratio, the
recording stops --after seconds after the stage arrives, and with
--return the stage then goes back by MM (not recorded). The stage
pushes the scissors' handle, so with --lever-ratio r (handle arm /
blade arm, measured on the jig) the row also gets the force and travel
at the blade: blade_peak_n = main_peak_n * r and
blade_travel_mm = travel_mm / r. Home the stage first
(ros2 run agri_bench stage home) and start it at least the ramp distance
v^2 / (2 a) before the blade meets the stem.

    ros2 run agri_bench trial --test A --speed 50 --lever-ratio 2.6 \\
        --stage-move 25 --return --position distal --diameter 3.1 3.3
"""
import argparse
import csv
import datetime
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from agri_bench.stage_cli import StageClient, StageError

TESTS = {'A': 'cut', 'B': 'grip', 'C': 'cut while held', 'D': 'pull-off'}
RESULTS = ['cut', 'slipped', 'crushed', 'dropped', 'detached', 'none']
COLUMNS = [
    'trial_id', 'datetime', 'test', 'variety', 'ripeness', 'hours_since_harvest',
    'temperature_c', 'humidity_pct', 'fruit_mass_g', 'diameter1_mm', 'diameter2_mm',
    'distance_from_abscission_mm', 'position', 'blade_angle_deg', 'speed_mm_s',
    'blade_uses', 'finger_material', 'wet', 'grip_set_n',
    'lever_ratio', 'stage_speed_mm_s',
    'main_peak_n', 'travel_mm', 'work_mj', 'loaded_s', 'blade_peak_n', 'blade_travel_mm',
    'grip_left_mean_n', 'grip_right_mean_n', 'grip_left_peak_n', 'grip_right_peak_n',
    'result', 'photo', 'raw_file', 'notes',
]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--test', required=True, choices=sorted(TESTS))
    ap.add_argument('--variety', default='')
    ap.add_argument('--ripeness', default='', help='e.g. red, breaker')
    ap.add_argument('--hours', type=float, help='hours since harvest')
    ap.add_argument('--temperature', type=float, help='[°C]')
    ap.add_argument('--humidity', type=float, help='[%%]')
    ap.add_argument('--fruit-mass', type=float, help='[g]')
    ap.add_argument('--diameter', type=float, nargs=2, metavar=('D1', 'D2'),
                    help='stem diameter in two directions [mm]')
    ap.add_argument('--distance', type=float, help='from the abscission zone [mm]')
    ap.add_argument('--position', default='', help='distal or proximal')
    ap.add_argument('--angle', type=float, help='blade angle [deg]')
    ap.add_argument('--speed', type=float, help='[mm/s]')
    ap.add_argument('--blade-uses', type=int)
    ap.add_argument('--material', default='', help='finger material')
    ap.add_argument('--wet', action='store_true')
    ap.add_argument('--grip-set', type=float, help='set squeeze force per finger [N]')
    ap.add_argument('--travel', type=float,
                    help='travel [mm] read by hand, if there is no position channel')
    ap.add_argument('--result', choices=RESULTS)
    ap.add_argument('--photo', default='')
    ap.add_argument('--notes', default='')
    ap.add_argument('--duration', type=float, help='record this long instead of until Enter [s]')
    ap.add_argument('--stage-move', type=float, metavar='MM',
                    help='let the stage move this far during the recording [mm]')
    ap.add_argument('--lever-ratio', type=float,
                    help='handle arm / blade arm of the scissors jig (stage speed = speed * ratio)')
    ap.add_argument('--after', type=float, default=0.5,
                    help='keep recording this long after the stage stops [s]')
    ap.add_argument('--return', dest='return_stage', action='store_true',
                    help='move the stage back afterwards (not recorded)')
    ap.add_argument('--return-speed', type=float, default=20.0, help='[mm/s]')
    ap.add_argument('--tare', action='store_true',
                    help='zero the force channels first (nothing may be on them)')
    ap.add_argument('--logger', default='/force_logger', help='force_logger node name')
    ap.add_argument('--output-dir', default='~/bench_logs')
    args = ap.parse_args(argv)
    for name in ('speed', 'stage_move', 'lever_ratio', 'return_speed', 'after', 'duration'):
        value = getattr(args, name)
        if value is not None and not math.isfinite(value):
            ap.error(f'--{name.replace("_", "-")} must be a finite number')
    if args.stage_move is not None:
        if args.speed is None or args.speed <= 0:
            ap.error('--stage-move needs --speed (the blade speed) [mm/s]')
        if args.duration is not None:
            ap.error('--stage-move and --duration exclude each other')
    if args.return_speed <= 0:
        ap.error('--return-speed must be positive')
    if args.lever_ratio is not None and args.lever_ratio <= 0:
        ap.error('--lever-ratio must be positive')
    return args


def stage_speed(args):
    return args.speed * (args.lever_ratio or 1.0)


def call(node, name, timeout=5.0):
    client = node.create_client(Trigger, name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise SystemExit(f'{name} not available: is force_logger running?')
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    res = future.result()
    if res is None or not res.success:
        raise SystemExit(f'{name} failed: {None if res is None else res.message}')
    return res.message


def check_header(path):
    """Refuse to append to a trials.csv written with other columns."""
    if not os.path.exists(path):
        return
    with open(path, newline='') as f:
        header = next(csv.reader(f), None)
    if header is not None and header != COLUMNS:
        raise SystemExit(f'{path} has other columns than this version of trial writes; '
                         'rename it (or use --output-dir) and start a new one')


def stop_recording(node, base):
    """Stop force_logger after an interruption; returns the raw file or None."""
    try:
        return json.loads(call(node, f'{base}/stop')).get('raw_file')
    except (SystemExit, ValueError):
        return None     # was not recording (yet), or no answer


def next_trial_id(path, test, day):
    prefix = f'{test}-{day}-'
    count = 0
    if os.path.exists(path):
        with open(path, newline='') as f:
            count = sum(1 for row in csv.DictReader(f) if row['trial_id'].startswith(prefix))
    return f'{prefix}{count + 1:03d}'


def row_for(args, summary, trial_id, now):
    def num(v):
        return '' if v is None else v
    travel = summary.get('travel_mm', args.travel)
    r = args.lever_ratio
    peak = summary.get('main_peak_n')
    blade_peak = None if r is None or peak is None else round(peak * r, 3)
    blade_travel = None if r is None or travel is None else round(travel / r, 3)
    return {
        'trial_id': trial_id, 'datetime': now.isoformat(timespec='seconds'),
        'test': args.test, 'variety': args.variety, 'ripeness': args.ripeness,
        'hours_since_harvest': num(args.hours), 'temperature_c': num(args.temperature),
        'humidity_pct': num(args.humidity), 'fruit_mass_g': num(args.fruit_mass),
        'diameter1_mm': num(args.diameter and args.diameter[0]),
        'diameter2_mm': num(args.diameter and args.diameter[1]),
        'distance_from_abscission_mm': num(args.distance), 'position': args.position,
        'blade_angle_deg': num(args.angle), 'speed_mm_s': num(args.speed),
        'blade_uses': num(args.blade_uses), 'finger_material': args.material,
        'wet': 'yes' if args.wet else 'no', 'grip_set_n': num(args.grip_set),
        'lever_ratio': num(r),
        'stage_speed_mm_s': num(round(stage_speed(args), 3)
                                if args.stage_move is not None else None),
        'main_peak_n': num(peak), 'travel_mm': num(travel),
        'work_mj': num(summary.get('work_mj')), 'loaded_s': num(summary.get('loaded_s')),
        'blade_peak_n': num(blade_peak), 'blade_travel_mm': num(blade_travel),
        'grip_left_mean_n': num(summary.get('grip_left_mean_n')),
        'grip_right_mean_n': num(summary.get('grip_right_mean_n')),
        'grip_left_peak_n': num(summary.get('grip_left_peak_n')),
        'grip_right_peak_n': num(summary.get('grip_right_peak_n')),
        'result': args.result, 'photo': args.photo, 'raw_file': summary.get('raw_file', ''),
        'notes': args.notes,
    }


def main(argv=None):
    args = parse_args(argv)
    out_dir = os.path.expanduser(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'trials.csv')
    check_header(path)
    # Keep Python's Ctrl+C (KeyboardInterrupt) instead of rclpy's handler,
    # which would shut ROS down before the logger could be told to stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('bench_trial')
    try:
        base = args.logger.rstrip('/')
        stage = None
        if args.stage_move is not None:
            stage = StageClient(node, base)
            stage.connect()
            status = stage.status()
            if not status.get('homed'):
                raise SystemExit('the stage is not homed: ros2 run agri_bench stage home')
            if status.get('moving'):
                raise SystemExit('the stage is still moving')
        if args.tare:
            print('tare: ' + call(node, f'{base}/tare'))
        try:
            call(node, f'{base}/start')
            if stage is not None:
                speed = stage_speed(args)
                print(f'stage: {args.stage_move:+.1f} mm at {speed:.1f} mm/s')
                event, fields = stage.send(
                    'MOVE', args.stage_move, speed,
                    timeout=StageClient.move_timeout(args.stage_move, speed))
                if event != 'done':
                    raise StageError(f'{event} at {fields.get("pos")} mm')
                stage.spin_for(args.after)
            elif args.duration is not None:
                print(f'recording for {args.duration:.1f} s')
                time.sleep(args.duration)
            else:
                input('recording: press Enter when the trial is over ')
        except (KeyboardInterrupt, EOFError, StageError, ValueError) as exc:
            # Stop the stage and the logger; no row is added. (On Ctrl+C
            # during a move StageClient has sent STOP already; once more
            # does no harm.)
            if stage is not None:
                stage.stop_quietly()
            raw = stop_recording(node, base)
            if isinstance(exc, (KeyboardInterrupt, EOFError)):
                why = 'interrupted'
            else:
                why = f'stage: {exc}' if isinstance(exc, StageError) else str(exc)
            raise SystemExit(f'\n{why}: recording stopped, not added to trials.csv '
                             f'(samples in {raw})')
        summary = json.loads(call(node, f'{base}/stop'))
        if stage is not None and args.return_stage:
            try:
                event, fields = stage.send(
                    'MOVE', -args.stage_move, args.return_speed,
                    timeout=StageClient.move_timeout(args.stage_move, args.return_speed))
                if event != 'done':
                    print(f'stage {event} at {fields.get("pos")} mm, not back at the start')
            except StageError as exc:
                print(f'stage did not return: {exc}')
            except KeyboardInterrupt:
                print('\nreturn interrupted: STOP sent (the trial is still added)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if args.result is None:
        while True:
            answer = input(f'result ({"/".join(RESULTS)}): ').strip()
            if answer in RESULTS:
                args.result = answer
                break
    now = datetime.datetime.now()
    trial_id = next_trial_id(path, args.test, now.strftime('%Y%m%d'))
    row = row_for(args, summary, trial_id, now)
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow(row)
    print(f'{trial_id}: ' + ', '.join(f'{k} {v}' for k, v in summary.items() if k != 'raw_file'))
    print(f'added to {path}')


if __name__ == '__main__':
    main()
