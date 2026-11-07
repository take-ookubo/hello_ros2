"""Move the arm through harvesting-like poses and report the holding torque.

Each pose is sent to the joint_trajectory_controller as a FollowJointTrajectory
goal. While the arm holds the pose, the node averages the joint efforts from
/joint_states and prints them as a percentage of the module torque used in the
URDF. For RMD modules that is the rated torque. For DYNAMIXEL it is only a
provisional reference below the stall torque, not a verified continuous rating.
"""
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Torque [N·m] used as the 100 % reference for each joint (same as the URDF effort limits).
# joint4-6 values are provisional; replace them with continuous ratings from tests.
MODULE_TORQUE = {'joint1': 20.0, 'joint2': 43.0, 'joint3': 20.0,
                 'joint4': 6.0, 'joint5': 6.0, 'joint6': 2.5}
# Joint angles [rad]. joint2 = 0 is upright; positive joint2/joint3 lean forward (+x).
POSES = [
    ('home', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ('ready', [0.0, 0.5, 1.2, 0.0, 1.0, 0.0]),
    ('reach_forward', [0.0, 1.2, 0.6, 0.0, -0.2, 0.0]),
    ('reach_left', [1.0, 0.9, 0.9, 0.5, 0.4, 1.0]),
    ('horizontal', [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]),  # Worst case for the shoulder.
]
MOVE_TIME = 3.0  # [s] per move
HOLD_TIME = 1.5  # [s] holding time used for the torque average


class PoseCycle(Node):

    def __init__(self):
        super().__init__('pose_cycle')
        self.declare_parameter('cycles', 0)  # 0 repeats forever.
        self.client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)

    def on_joint_state(self, msg):
        self.latest = msg

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_seconds(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def move(self, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint(positions=positions, velocities=[0.0] * len(JOINTS))
        point.time_from_start = Duration(sec=int(MOVE_TIME), nanosec=int(MOVE_TIME % 1 * 1e9))
        goal.trajectory.points = [point]
        # The action server exists once the controller is configured, but it
        # rejects goals until the controller is active. Retry for a while.
        for _ in range(30):
            handle = self.wait(self.client.send_goal_async(goal))
            if handle.accepted:
                break
            self.wait_seconds(1.0)
        else:
            return 'rejected'
        result = self.wait(handle.get_result_async()).result
        return 'ok' if result.error_code == 0 else f'error {result.error_code} {result.error_string}'

    def hold_and_measure(self):
        """Average position and effort over HOLD_TIME of simulated time.

        Return None when no complete joint state arrived during the hold.
        """
        sums = {j: [0.0, 0.0] for j in JOINTS}
        count = 0
        start = self.get_clock().now()
        last = None
        while (self.get_clock().now() - start).nanoseconds < HOLD_TIME * 1e9:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self.latest
            if msg is None or msg is last or not msg.effort:
                continue
            last = msg
            values = dict(zip(msg.name, zip(msg.position, msg.effort)))
            if any(j not in values for j in JOINTS):
                continue
            for j in JOINTS:
                sums[j][0] += values[j][0]
                sums[j][1] += values[j][1]
            count += 1
        if count == 0:
            return None
        return {j: (p / count, e / count) for j, (p, e) in sums.items()}

    def run(self):
        self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory ...')
        self.client.wait_for_server()
        cycles = self.get_parameter('cycles').value
        cycle = 0
        while rclpy.ok() and (cycles == 0 or cycle < cycles):
            cycle += 1
            for name, target in POSES:
                status = self.move(target)
                measured = self.hold_and_measure()
                if measured is None:
                    self.get_logger().warning(
                        f'[{cycle}] {name:<13} {status} no complete /joint_states with effort')
                    continue
                error = max(abs(measured[j][0] - t) for j, t in zip(JOINTS, target))
                torques = ' '.join(
                    f'{j[-1]}:{measured[j][1]:+6.2f}({abs(measured[j][1]) / MODULE_TORQUE[j]:4.0%})'
                    for j in JOINTS)
                self.get_logger().info(
                    f'[{cycle}] {name:<13} {status} max err {error:.3f} rad | N·m(%) {torques}')


def main():
    rclpy.init()
    node = PoseCycle()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
