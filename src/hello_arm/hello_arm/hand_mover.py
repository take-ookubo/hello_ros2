import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Fully closed angle [rad] of the proximal and distal joints of each finger.
FINGERS = {
    'thumb': (0.9, 0.9),
    'index': (1.2, 1.3),
    'middle': (1.2, 1.3),
    'ring': (1.2, 1.3),
    'little': (1.2, 1.3),
}
CYCLE = 8.0         # One grasp cycle [s].
CLOSE_DELAY = 0.6   # Each finger starts closing this long after the previous one [s].
CLOSE_TIME = 1.0    # Time for one finger to close [s].
OPEN_START = 5.0    # All fingers open together from this time in the cycle [s].
OPEN_TIME = 1.0


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def closure(index, t):
    """Return 0 (open) to 1 (closed) for finger `index` at cycle time t."""
    if t < OPEN_START:
        return smoothstep((t - CLOSE_DELAY * index) / CLOSE_TIME)
    return 1.0 - smoothstep((t - OPEN_START) / OPEN_TIME)


class HandMover(Node):
    """Close the fingers one by one from the thumb, then open them together."""

    def __init__(self):
        super().__init__('hand_mover')
        self.cmd_pubs = {}
        for finger in FINGERS:
            for part in ('proximal', 'distal'):
                joint = f'{finger}_{part}'
                self.cmd_pubs[joint] = self.create_publisher(Float64, f'/hand/{joint}/cmd_pos', 10)
        self.targets = {joint: 0.0 for joint in self.cmd_pubs}
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.start = self.get_clock().now()
        self.create_timer(0.02, self.send_commands)
        self.last_log = None

    def send_commands(self):
        t = ((self.get_clock().now() - self.start).nanoseconds * 1e-9) % CYCLE
        for index, (finger, (proximal, distal)) in enumerate(FINGERS.items()):
            c = closure(index, t)
            self.targets[f'{finger}_proximal'] = proximal * c
            self.targets[f'{finger}_distal'] = distal * c
        for joint, pub in self.cmd_pubs.items():
            pub.publish(Float64(data=self.targets[joint]))

    def on_joint_state(self, msg):
        now = self.get_clock().now()
        if self.last_log and (now - self.last_log).nanoseconds < 1e9:
            return
        self.last_log = now
        positions = dict(zip(msg.name, msg.position))
        parts = []
        for finger in FINGERS:
            joint = f'{finger}_proximal'
            if joint in positions:
                parts.append(f'{finger} {positions[joint]:+.2f} ({self.targets[joint]:+.2f})')
        self.get_logger().info('proximal (target) ' + ' | '.join(parts))


def main():
    rclpy.init()
    node = HandMover()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
