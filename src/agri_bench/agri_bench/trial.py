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
"""
import argparse
import csv
import datetime
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

TESTS = {'A': 'cut', 'B': 'grip', 'C': 'cut while held', 'D': 'pull-off'}
RESULTS = ['cut', 'slipped', 'crushed', 'dropped', 'detached', 'none']
COLUMNS = [
    'trial_id', 'datetime', 'test', 'variety', 'ripeness', 'hours_since_harvest',
    'temperature_c', 'humidity_pct', 'fruit_mass_g', 'diameter1_mm', 'diameter2_mm',
    'distance_from_abscission_mm', 'position', 'blade_angle_deg', 'speed_mm_s',
    'blade_uses', 'finger_material', 'wet', 'grip_set_n',
    'main_peak_n', 'travel_mm', 'work_mj', 'loaded_s',
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
    ap.add_argument('--tare', action='store_true',
                    help='zero the force channels first (nothing may be on them)')
    ap.add_argument('--logger', default='/force_logger', help='force_logger node name')
    ap.add_argument('--output-dir', default='~/bench_logs')
    return ap.parse_args(argv)


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
        'main_peak_n': num(summary.get('main_peak_n')), 'travel_mm': num(travel),
        'work_mj': num(summary.get('work_mj')), 'loaded_s': num(summary.get('loaded_s')),
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
    # Keep Python's Ctrl+C (KeyboardInterrupt) instead of rclpy's handler,
    # which would shut ROS down before the logger could be told to stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('bench_trial')
    try:
        base = args.logger.rstrip('/')
        if args.tare:
            print('tare: ' + call(node, f'{base}/tare'))
        call(node, f'{base}/start')
        try:
            if args.duration is not None:
                print(f'recording for {args.duration:.1f} s')
                time.sleep(args.duration)
            else:
                input('recording: press Enter when the trial is over ')
        except (KeyboardInterrupt, EOFError):
            # Stop the logger so it does not keep recording; no row is added.
            summary = json.loads(call(node, f'{base}/stop'))
            raise SystemExit('\ninterrupted: recording stopped, not added to trials.csv '
                             f'(samples in {summary.get("raw_file")})')
        summary = json.loads(call(node, f'{base}/stop'))
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
