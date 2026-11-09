"""Drive the bench linear stage through force_logger.

    ros2 run agri_bench stage home
    ros2 run agri_bench stage goto 30 20      # to 30 mm at 20 mm/s
    ros2 run agri_bench stage move 10 5       # 10 mm further at 5 mm/s
    ros2 run agri_bench stage status
    ros2 run agri_bench stage stop

It sends one command (see stage.py) and waits for the board's answer.
Ctrl+C while the stage moves sends STOP. The stage must be homed before
MOVE and GOTO, and again after the motor power was cut (emergency stop).
"""
import argparse
import itertools
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from agri_bench.stage import command, parse_message

# Command ids for this process: a random start, so that two clients (in
# one process, or on two machines) do not share ids.
_IDS = itertools.count(random.SystemRandom().randrange(1, 2 ** 31))

# Events that end each command, besides 'error'.
ENDS = {'HOME': ('homed', 'stopped'), 'MOVE': ('done', 'stopped'), 'GOTO': ('done', 'stopped'),
        'STOP': ('stopped',), 'STATUS': ('status',)}


class StageError(Exception):
    pass


class StageClient:
    """Send stage commands to force_logger and wait for the replies."""

    def __init__(self, node, logger='/force_logger'):
        base = logger.rstrip('/')
        self.node = node
        self.message_topic = f'{base}/board_message'
        self.pub = node.create_publisher(String, f'{base}/stage_command', 10)
        self.replies = []
        # Replies carry the command's id, so that replies to an earlier
        # command or to another client are never taken as ours.
        self.ids = _IDS
        node.create_subscription(String, self.message_topic, self.on_message, 50)

    def on_message(self, msg):
        parsed = parse_message(msg.data)
        if parsed is not None:
            self.replies.append(parsed)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def connect(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (self.pub.get_subscription_count() > 0
                    and self.node.count_publishers(self.message_topic) > 0):
                self.spin_for(0.3)   # let force_logger's side match too
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        raise SystemExit('force_logger not found: is it running?')

    def send(self, name, *args, timeout=10.0):
        """Send one command and return (event, fields) of its final reply.

        Raises StageError for an error reply or no reply in time; Ctrl+C
        sends STOP before passing the KeyboardInterrupt on.
        """
        id = next(self.ids)
        line = command(name, *args, id=id)
        ends = ENDS[line.split()[0]]
        self.replies.clear()
        end = time.monotonic() + timeout
        try:
            self.pub.publish(String(data=line))
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
                while self.replies:
                    event, fields = self.replies.pop(0)
                    if fields.get('id') != id:
                        continue      # stale, or another client's
                    if event == 'error':
                        raise StageError(fields.get('reason', 'error'))
                    if event in ends:
                        return event, fields
        except KeyboardInterrupt:
            self.stop_quietly()
            raise
        if name.upper() != 'STOP':
            self.stop_quietly()
        raise StageError(f'no answer to {line} within {timeout:.0f} s')

    def stop_quietly(self):
        """Send STOP and wait briefly, ignoring the outcome."""
        try:
            self.pub.publish(String(data='STOP'))
            self.spin_for(0.3)
        except Exception:  # noqa: BLE001 - best effort while shutting down
            pass

    def status(self):
        _, fields = self.send('STATUS', timeout=3.0)
        return fields

    @staticmethod
    def move_timeout(distance, speed):
        return abs(distance) / speed * 1.5 + 10.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('action', choices=['home', 'move', 'goto', 'stop', 'status'])
    ap.add_argument('mm', nargs='?', type=float, help='move/goto: distance or position [mm]')
    ap.add_argument('speed', nargs='?', type=float, help='move/goto: top speed [mm/s]')
    ap.add_argument('--logger', default='/force_logger', help='force_logger node name')
    args = ap.parse_args(argv)
    if args.action in ('move', 'goto') and (args.mm is None or args.speed is None):
        ap.error(f'{args.action} needs a distance [mm] and a speed [mm/s]')
    if args.action in ('move', 'goto'):
        try:
            command(args.action, args.mm, args.speed)
        except ValueError as exc:
            ap.error(str(exc))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('bench_stage')
    try:
        stage = StageClient(node, args.logger)
        stage.connect()
        if args.action == 'home':
            event, fields = stage.send('HOME', timeout=60.0)
        elif args.action in ('move', 'goto'):
            distance = args.mm
            if args.action == 'goto':
                distance = args.mm - stage.status().get('pos', 0.0)
            event, fields = stage.send(args.action, args.mm, args.speed,
                                       timeout=StageClient.move_timeout(distance, args.speed))
        else:
            event, fields = stage.send(args.action, timeout=3.0)
        print(event + ''.join(f' {k}={v:g}' for k, v in fields.items() if k != 'id'))
        if event == 'stopped' and args.action != 'stop':
            raise SystemExit('stopped before the end')
    except StageError as exc:
        raise SystemExit(f'stage: {exc}')
    except KeyboardInterrupt:
        raise SystemExit('\ninterrupted: STOP sent')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
