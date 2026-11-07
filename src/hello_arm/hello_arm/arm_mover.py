import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Amplitude [rad] and period [s] of the sine-wave target for each joint.
MOTIONS = {
    'joint1': (1.2, 8.0),
    'joint2': (0.6, 6.0),
    'joint3': (0.9, 4.0),
}


class ArmMover(Node):
    """Swing each arm joint along a sine wave and report the measured angles."""

    def __init__(self):
        super().__init__('arm_mover')
        self.cmd_pubs = {
            name: self.create_publisher(Float64, f'/arm/{name}/cmd_pos', 10)
            for name in MOTIONS
        }
        self.targets = {name: 0.0 for name in MOTIONS}
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.start = self.get_clock().now()
        self.create_timer(0.02, self.send_commands)
        self.last_log = None

    def send_commands(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        for name, (amplitude, period) in MOTIONS.items():
            self.targets[name] = amplitude * math.sin(2.0 * math.pi * t / period)
            self.cmd_pubs[name].publish(Float64(data=self.targets[name]))

    def on_joint_state(self, msg):
        now = self.get_clock().now()
        if self.last_log and (now - self.last_log).nanoseconds < 5e8:
            return
        self.last_log = now
        parts = []
        for name, position in zip(msg.name, msg.position):
            if name in self.targets:
                parts.append(f'{name} {position:+.2f} (target {self.targets[name]:+.2f})')
        self.get_logger().info(' | '.join(parts))


def main():
    rclpy.init()
    node = ArmMover()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
