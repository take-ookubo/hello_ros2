"""Calibrate one channel with known weights (or distances) and save it.

    ros2 run agri_bench calibrate --channel cut --masses 0 0.5 1 2 \\
        --file ~/bench_logs/calibration.yaml

For each mass [kg] (or, for the position channel, each distance [mm]
with --distances) it asks you to set it, averages force_logger's raw
counts over --seconds, fits value = (raw - offset) * scale and writes the
channel into the calibration file, keeping the other channels. Start
force_logger with calibration_file:=<that file> afterwards. Hang the
weights the way the load is applied in the test (same direction and
point), and include 0 (no load).
"""
import argparse
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from agri_bench.protocol import POSITION, STAGE, fit_line

G = 9.80665


def mean_raw(node, index, seconds):
    values = []
    sub = node.create_subscription(Float64MultiArray, '/force_logger/raw',
                                   lambda m: values.append(m.data[index]), 100)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    if not values:
        raise SystemExit('no data on /force_logger/raw: is force_logger running?')
    return sum(values) / len(values), len(values)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--channel', required=True)
    ap.add_argument('--channels', nargs='+',
                    default=['cut', 'grip_left', 'grip_right', POSITION, STAGE],
                    help="force_logger's channels parameter, in order")
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument('--masses', type=float, nargs='+', help='[kg]')
    group.add_argument('--distances', type=float, nargs='+', help='[mm], position channel')
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--file', default='~/bench_logs/calibration.yaml')
    args = ap.parse_args(argv)
    if args.channel not in args.channels:
        raise SystemExit(f'{args.channel} is not one of {args.channels}')
    index = args.channels.index(args.channel)
    if args.masses is not None:
        known = [(m * G, f'{m} kg') for m in args.masses]
    else:
        known = [(d, f'{d} mm') for d in args.distances]

    rclpy.init()
    node = Node('bench_calibrate')
    points = []
    try:
        for value, label in known:
            input(f'set {label} on {args.channel}, then press Enter ')
            raw, n = mean_raw(node, index, args.seconds)
            print(f'  {label}: raw {raw:.1f} ({n} samples)')
            points.append((value, raw))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    offset, scale, rms = fit_line(points)
    unit = 'mm' if args.distances is not None else 'N'
    print(f'{args.channel}: offset {offset:.1f}, scale {scale:.4e} {unit}/count, '
          f'fit rms {rms:.4f} {unit}')

    path = os.path.expanduser(args.file)
    data = {}
    if os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    data.setdefault('channels', {})[args.channel] = {
        'offset': float(offset), 'scale': float(scale)}
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f'saved to {path}')


if __name__ == '__main__':
    main()
