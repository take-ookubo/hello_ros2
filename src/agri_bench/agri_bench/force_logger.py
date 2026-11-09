"""Read the bench load cells, publish the forces and record trials.

Data come from the board's serial port (port parameter), or from a fake
board when port is empty, so the whole chain can be tried without
hardware. See docs/stem_force_test_plan.md for the tests.

Topics (std_msgs/Float64, one per channel): ~/<channel>, in N (force
channels, after tare) or mm (position, stage). ~/raw (Float64MultiArray)
has the raw counts in channel order, for calibrate. ~/board_message
(String) repeats the board's comment lines, among them the stage's
replies; ~/stage_command (String, subscribed) is sent to the board as a
stage command (see stage.py, and the stage command line tool).

Services (std_srvs/Trigger):
- ~/tare: zero the force channels at their mean over the last tare_seconds.
- ~/start: start recording a trial.
- ~/stop: stop, write the samples to <output_dir>/raw/<stamp>.csv and
  return a JSON summary (peaks, loaded time, travel, work and the file).
"""
import collections
import datetime
import json
import os
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger

from agri_bench.protocol import NON_FORCE, POSITION, Clock, apply_calibration, parse_line, summarize
from agri_bench.stage import STAGE
from agri_bench.sources import FakeSource, SerialSource


class ForceLogger(Node):
    def __init__(self):
        super().__init__('force_logger')
        default_cal = os.path.join(get_package_share_directory('agri_bench'), 'config',
                                   'calibration_example.yaml')
        self.declare_parameter('port', '')           # '' = fake board
        self.declare_parameter('baud', 115200)
        self.declare_parameter('channels', ['cut', 'grip_left', 'grip_right', POSITION, STAGE])
        self.declare_parameter('calibration_file', default_cal)
        self.declare_parameter('main_channel', 'cut')
        self.declare_parameter('threshold', 0.2)     # loaded above this [N]
        self.declare_parameter('tare_seconds', 0.5)
        self.declare_parameter('output_dir', '~/bench_logs')
        self.declare_parameter('fake_peak', 30.0)    # fake cut peak [N]
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.names = list(p('channels'))
        with open(os.path.expanduser(p('calibration_file'))) as f:
            self.calibration = (yaml.safe_load(f) or {}).get('channels', {})
        missing = [n for n in self.names if n not in self.calibration]
        if missing:
            self.get_logger().warn(f'no calibration for {missing}: published as raw counts')
        self.tare = {n: 0.0 for n in self.names}
        self.output_dir = os.path.expanduser(p('output_dir'))

        port = p('port')
        if port:
            self.source = SerialSource(port, p('baud'))
            self.get_logger().info(f'reading {port}')
        else:
            self.source = FakeSource(self.names, peak_n=p('fake_peak'))
            self.get_logger().info('no port: using the fake board')

        self.clock = Clock()
        self.lock = threading.Lock()
        self.incoming = []
        self.bad_lines = 0
        self.recent = collections.deque()   # (t, values) for tare
        self.recording = None               # [(t, values)] during a trial
        self.last_time = None

        self.pubs = {n: self.create_publisher(Float64, f'~/{n}', 10) for n in self.names}
        self.raw_pub = self.create_publisher(Float64MultiArray, '~/raw', 10)
        self.message_pub = self.create_publisher(String, '~/board_message', 50)
        self.create_subscription(String, '~/stage_command', self.on_stage_command, 10)
        self.create_service(Trigger, '~/tare', self.on_tare)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.02, self.drain)
        self.create_timer(1.0, self.check_rate)
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def read_loop(self):
        while self.running:
            try:
                line = self.source.readline()
            except Exception as exc:  # noqa: BLE001 - serial raises several types
                self.get_logger().error(f'read failed: {exc}', throttle_duration_sec=5.0)
                continue
            if not line:
                continue
            if line.startswith('#'):
                self.message_pub.publish(String(data=line.strip()))
                text = f'board: {line.strip()}'
                if 'error' in line:
                    self.get_logger().error(text, throttle_duration_sec=5.0)
                else:
                    self.get_logger().info(text)
                continue
            sample = parse_line(line, len(self.names))
            with self.lock:
                if sample is None:
                    self.bad_lines += 1
                else:
                    self.incoming.append(sample)

    def drain(self):
        with self.lock:
            samples, self.incoming = self.incoming, []
            bad, self.bad_lines = self.bad_lines, 0
        if bad:
            self.get_logger().warn(f'{bad} lines with a wrong format '
                                   f'(expected D,time_us + {len(self.names)} values)',
                                   throttle_duration_sec=5.0)
        window = self.get_parameter('tare_seconds').value
        for time_us, raw in samples:
            t = self.clock.seconds(time_us)
            self.last_time = self.get_clock().now()
            values = apply_calibration(raw, self.names, self.calibration)
            values = [v - self.tare[n] for n, v in zip(self.names, values)]
            self.recent.append((t, values))
            while self.recent and self.recent[0][0] < t - window:
                self.recent.popleft()
            if self.recording is not None:
                self.recording.append((t, values))
            self.raw_pub.publish(Float64MultiArray(data=raw))
            for n, v in zip(self.names, values):
                self.pubs[n].publish(Float64(data=v))

    def on_stage_command(self, msg):
        line = msg.data.strip()
        if not line:
            return
        self.get_logger().info(f'stage command: {line}')
        try:
            self.source.write(line)
        except Exception as exc:  # noqa: BLE001 - serial raises several types
            self.get_logger().error(f'could not send {line!r}: {exc}')

    def check_rate(self):
        if self.last_time is None or (self.get_clock().now() - self.last_time).nanoseconds > 1e9:
            self.get_logger().warn('no data from the board for 1 s', throttle_duration_sec=10.0)

    def on_tare(self, request, response):
        if not self.recent:
            response.success, response.message = False, 'no data yet'
            return response
        for i, n in enumerate(self.names):
            if n in NON_FORCE:
                continue
            mean = sum(v[i] for _, v in self.recent) / len(self.recent)
            self.tare[n] += mean
            # The cached samples are now zeroed too, so a second tare
            # right away changes nothing.
            for _, v in self.recent:
                v[i] -= mean
        response.success = True
        response.message = json.dumps({n: round(v, 4) for n, v in self.tare.items()
                                       if n not in NON_FORCE})
        self.get_logger().info(f'tare: {response.message}')
        return response

    def on_start(self, request, response):
        if self.recording is not None:
            self.get_logger().warn(f'previous recording ({len(self.recording)} samples) '
                                   'was not stopped: discarded')
        self.recording = []
        response.success, response.message = True, 'recording'
        self.get_logger().info('recording')
        return response

    def on_stop(self, request, response):
        if self.recording is None:
            response.success, response.message = False, 'not recording'
            return response
        samples, self.recording = self.recording, None
        times = [t for t, _ in samples]
        values = [v for _, v in samples]
        summary = summarize(times, values, self.names, self.get_parameter('main_channel').value,
                            self.get_parameter('threshold').value)
        raw_dir = os.path.join(self.output_dir, 'raw')
        os.makedirs(raw_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        path = os.path.join(raw_dir, f'{stamp}.csv')
        with open(path, 'w') as f:
            f.write('time_s,' + ','.join(self.names) + '\n')
            t0 = times[0] if times else 0.0
            for t, v in samples:
                f.write(f'{t - t0:.4f},' + ','.join(f'{x:.4f}' for x in v) + '\n')
        summary['raw_file'] = path
        response.success, response.message = True, json.dumps(summary)
        self.get_logger().info(f'trial: {response.message}')
        return response

    def destroy_node(self):
        self.running = False
        self.reader.join(timeout=1.0)
        self.source.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ForceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
