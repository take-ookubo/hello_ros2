"""Measure joint torque during fast moves (run with control_mode:=effort).

Each test move is a smooth quintic trajectory between two poses. Its length
is set so the fastest joint reaches a given fraction of its URDF velocity
limit. During the move the node records, per joint:

- peak |torque| from /joint_states and its share of the module torque,
- how often the torque sat at the limit (saturation),
- the largest tracking error from the controller state,
- the error left 0.5 s after the trajectory ends.

In position mode Gazebo moves the joints ideally, so the torques printed
there do not include the motor's own control effort; use effort mode.
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from agri_arm.pose_cycle import JOINTS, MODULE_TORQUE

# URDF velocity limits [rad/s].
MAX_VEL = {'joint1': 2.0, 'joint2': 1.5, 'joint3': 2.0,
           'joint4': 3.0, 'joint5': 3.0, 'joint6': 4.0}
H = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]          # arm stretched out horizontally
# (name, start, goal). These load the shoulder, base and wrist hardest.
MOVES = [
    ('lower_to_horizontal', [0.0] * 6, H),
    ('raise_from_horizontal', H, [0.0] * 6),
    ('base_swing_extended', [-1.0] + H[1:], [1.0] + H[1:]),
    ('wrist_flip_extended', H[:4] + [-1.2, 0.0], H[:4] + [1.2, 0.0]),
]
DT = 0.01          # trajectory sample time [s]
SETTLE = 1.5       # hold at the start pose before each move [s]
TAIL = 0.5         # keep recording after the trajectory ends [s]


def quintic(s):
    """Position, velocity and acceleration factors of a 0->1 quintic at s in [0, 1]."""
    return (10 * s**3 - 15 * s**4 + 6 * s**5,
            30 * s**2 - 60 * s**3 + 30 * s**4,
            60 * s - 180 * s**2 + 120 * s**3)


def duration_for(start, goal, scale):
    # Peak velocity of the quintic is 1.875 * distance / T.
    return max(1.875 * abs(g - s) / (scale * MAX_VEL[j])
               for j, s, g in zip(JOINTS, start, goal))


def to_msg(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


class DynamicsTest(Node):
    def __init__(self):
        super().__init__('dynamics_test')
        self.declare_parameter('speeds', [0.25, 0.5, 1.0])
        self.client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.recording = False
        self.efforts = []
        self.errors = []
        self.last_error = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/arm_controller/controller_state', self.on_state, 50)

    def on_joints(self, msg):
        if self.recording:
            e = dict(zip(msg.name, msg.effort))
            if all(j in e for j in JOINTS):
                self.efforts.append([e[j] for j in JOINTS])

    def on_state(self, msg):
        if len(msg.error.positions) != len(JOINTS):
            return
        err = dict(zip(msg.joint_names, msg.error.positions))
        self.last_error = [err.get(j, 0.0) for j in JOINTS]
        if self.recording:
            self.errors.append(self.last_error)

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def sleep(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def send(self, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = points
        for _ in range(30):     # the controller may still be starting
            handle = self.wait(self.client.send_goal_async(goal))
            if handle.accepted:
                return handle
            self.sleep(1.0)
        raise RuntimeError('trajectory goal rejected')

    def go_to(self, pose, t=3.0):
        p = JointTrajectoryPoint(positions=pose, velocities=[0.0] * 6,
                                 time_from_start=to_msg(t))
        handle = self.send([p])
        self.wait(handle.get_result_async())

    def run_move(self, name, start, goal, scale):
        self.go_to(start)
        self.sleep(SETTLE)
        T = duration_for(start, goal, scale)
        points = []
        n = int(math.ceil(T / DT))
        for k in range(1, n + 1):
            t = min(k * DT, T)
            p, v, a = quintic(t / T)
            d = [g - s for s, g in zip(start, goal)]
            points.append(JointTrajectoryPoint(
                positions=[s + di * p for s, di in zip(start, d)],
                velocities=[di * v / T for di in d],
                accelerations=[di * a / T**2 for di in d],
                time_from_start=to_msg(t)))
        self.efforts, self.errors = [], []
        self.recording = True
        handle = self.send(points)
        result = self.wait(handle.get_result_async()).result
        self.sleep(TAIL)
        self.recording = False
        final = self.last_error or [math.nan] * 6
        if not self.efforts or not self.errors:
            self.get_logger().warn(f'{name}: no samples recorded')
            return
        peak = [max(abs(e[i]) for e in self.efforts) for i in range(6)]
        sat = [sum(abs(e[i]) >= 0.98 * MODULE_TORQUE[j] for e in self.efforts)
               / len(self.efforts) for i, j in enumerate(JOINTS)]
        err = [max(abs(e[i]) for e in self.errors) for i in range(6)]
        torque = ' '.join(
            f'{i + 1}:{peak[i]:.1f}({100 * peak[i] / MODULE_TORQUE[j]:.0f}%'
            + (f',sat{100 * sat[i]:.0f}%' if sat[i] > 0 else '') + ')'
            for i, j in enumerate(JOINTS))
        self.get_logger().info(
            f'{name} speed {scale:.2f} T {T:.2f} s (code {result.error_code}) | '
            f'peak N·m: {torque} | max err rad: '
            + ' '.join(f'{x:.3f}' for x in err)
            + ' | final err: ' + ' '.join(f'{abs(x):.4f}' for x in final))

    def run(self):
        self.client.wait_for_server()
        speeds = list(self.get_parameter('speeds').value)
        for name, start, goal in MOVES:
            for scale in speeds:
                self.run_move(name, start, goal, scale)
        # joint1, joint4 and joint6 are coaxial at home: check it stays stable.
        self.go_to([0.0] * 6)
        self.sleep(3.0)
        err = self.last_error or [math.nan] * 6
        self.get_logger().info(
            'home hold err rad: ' + ' '.join(f'{abs(x):.4f}' for x in err))
        self.get_logger().info('done')


def main():
    rclpy.init()
    node = DynamicsTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
