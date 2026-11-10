"""Plan collision-free harvesting moves with MoveIt 2 and run them in Gazebo.

The sequence is home -> left fruit -> crate -> right fruit -> crate -> home.
At each fruit the hand closes its fingers on the fruit stem, the blade cuts
the stem above the fingers, and the fruit, still held by its stem, is
carried to the crate and dropped. A support post
stands between the two fruits, so a straight joint-space move from one fruit
to the other would hit it. With use_camera the fruit positions come from
agri_perception's fruit_detector, read again at home before every cycle,
and their error from the true positions in the world file is logged.
Each fruit pose is turned into joint values with
collision-aware IK, seeded with an elbow-up posture so the solution is a
natural one. The fruit themselves are added to the planning scene as
spheres.

With approach:=lin (default) OMPL brings the tool to a pre-approach pose
PRE_APPROACH in front of the stem; the Pilz LIN planner then moves the
tool in a straight line to the stem and back out. With approach:=joint
OMPL goes straight to the stem.

With use_wrist_camera the hand stops at the pre-approach pose and
agri_perception's stem_detector looks at the stem with the wrist camera;
the straight approach is then moved onto the stem it sees, and with
align_roll the hand is also turned about its approach axis (joint6) so
that its "up" (tool -x) follows the stem's lean across the approach: the
finger pads then lie along the stem and the blade, 1.5 cm above the
fingers, crosses it right over the finger center instead of beside it.
With align_pitch the hand is also tilted about the finger closing
direction to the lean toward or away from it, so the approach comes in
square to the stem (from above or below) and the blade meets the stem at
the finger center's depth, not in front of or behind it. If a leaf hides
the stem, the hand looks again from the sides (wrist_view_yaws: the view
turned about the vertical through the stem) and comes back in front of the
stem it saw. If no view sees it the fruit is skipped and left on the plant
(if_unseen:=guess grasps at the straight-up guess instead). Without the
wrist camera the stem is assumed to go straight up from the fruit. After the fingers close
the node logs how far the finger center is from the true stem (STEMS).

In Gazebo the cut and the hold are detachable joints (plant-fruit and
hand-fruit) switched over ros_gz_bridge; the fingers do not hold the fruit
by friction. In MoveIt the fruit becomes an object attached to the tool
once it is cut, so the moves to the crate avoid obstacles with it. For
those moves the node reports the peak acceleration of the fruit and the
finger force that friction alone would need to hold it. After the grasp
it also logs the angle between the hand's up axis and the true stem and
how far sideways from and how deep past the finger center the blade
meets that stem. After
every cycle the fruit are hung back on the plant (set_pose + attach) for the next one.

For every move the node logs the planning time, whether the joint
states measured during execution collided, and the tool position error.
Joint moves also report whether a straight joint-space move would have
collided; LIN moves report how far the tool left the straight line, for
the planned and the executed path, and how far a joint-space move between
the same two poses would have strayed.
"""
import json
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject,
                             Constraints, JointConstraint, MotionPlanRequest,
                             OrientationConstraint, PlanningScene, PositionConstraint,
                             RobotState)
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionFK, GetPositionIK,
                             GetStateValidity)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

GROUP = 'arm'
TIP = 'tool0'
FRAME = 'world'
JOINTS = [f'joint{i}' for i in range(1, 7)]

HAND_JOINTS = ['finger_left_joint', 'finger_right_joint', 'cutter_joint']
HAND_LINKS = ['link6', 'finger_left', 'finger_right', 'cutter']

# Must match worlds/harvest_world.sdf. With use_camera, FRUITS and UNRIPE
# are only used as ground truth: to report the detection error and to know
# which Gazebo model to cut.
POST_CENTER = (0.42, 0.0, 0.45)
POST_SIZE = (0.04, 0.04, 0.9)
BRANCH_CENTER = (0.42, 0.0, 0.59)
BRANCH_SIZE = (0.02, 0.6, 0.02)
FRUITS = {'fruit_left': (0.385, 0.20, 0.45), 'fruit_right': (0.42, -0.20, 0.45)}
UNRIPE = [(0.42, -0.08, 0.45)]


def lean(across, away):
    """Unit stem direction (up) leaning across [deg] to -y in the y-z plane
    and away [deg] from the arm (+x) in the x-z plane."""
    d = (math.tan(math.radians(away)), -math.tan(math.radians(across)), 1.0)
    n = math.sqrt(sum(c * c for c in d))
    return tuple(c / n for c in d)


# True stem lines (point on the fruit top, unit direction up), as in the
# world file: the left stem leans 30° to -y and 20° away from the arm, the
# right one 35° to +y. Only used to report how far the fingers close from
# the real stem and how the hand lies on it.
STEMS = {
    'fruit_left': ((0.385, 0.20, 0.485), lean(30, 20)),
    'fruit_right': ((0.42, -0.20, 0.485), (0.0, math.sin(math.radians(35)),
                                           math.cos(math.radians(35)))),
}
CRATE = [  # (name, center, size)
    ('crate_stand', (0.05, 0.42, 0.05), (0.24, 0.20, 0.10)),
    ('crate_front', (0.165, 0.42, 0.15), (0.01, 0.20, 0.10)),
    ('crate_back', (-0.065, 0.42, 0.15), (0.01, 0.20, 0.10)),
    ('crate_left', (0.05, 0.515, 0.15), (0.22, 0.01, 0.10)),
    ('crate_right', (0.05, 0.325, 0.15), (0.22, 0.01, 0.10)),
]
CRATE_INSIDE = ((-0.06, 0.16), (0.33, 0.51), (0.10, 0.20))   # x, y, z ranges
FRUIT_RADIUS = 0.035
FRUIT_MASS = 0.15         # [kg], as in the world file
# The fingers close on the stem this far above the fruit center (radius +
# 2 cm). Without the wrist camera the stem is assumed to go straight up, so
# in the tool frame the fruit center is GRASP_ABOVE along +x (down).
GRASP_ABOVE = FRUIT_RADIUS + 0.02
PRE_APPROACH = 0.10       # LIN approach starts this far in front of the stem [m]
MATCH_DISTANCE = 0.03     # a detection this close to a true fruit is that fruit [m]
FRICTION = 0.5            # assumed finger pad - stem friction coefficient
# Hand joint targets [m]: fingers 0 = open, 0.027 = on a 6 mm stem.
FINGERS_OPEN, FINGERS_CLOSED = 0.0, 0.027
CUTTER_OPEN, CUTTER_CLOSED = 0.0, 0.06
CUT_ABOVE = 0.015         # blade above the finger center (tool -x) [m], as in the URDF
# Tool z axis pointing along +x (tool -x up): rotate +90 degrees about y.
APPROACH_QUAT = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
# Above the crate, tool z along +y and tool -x still up.
DROP_XYZ = (0.05, 0.42, 0.26)
DROP_QUAT = (-0.5, 0.5, 0.5, 0.5)
HOME = [0.0] * 6
GRAVITY = (0.0, 0.0, -9.81)

SUCCESS = 1


def box(name, center, size):
    return shape(name, center, SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))


def sphere(name, center, radius, frame=FRAME):
    return shape(name, center, SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius]),
                 frame)


def point_line_mm(p, a, d):
    """Distance [mm] of point p from the line through a with unit direction d."""
    ap = [pi - ai for pi, ai in zip(p, a)]
    t = sum(x * y for x, y in zip(ap, d))
    return 1000 * math.dist(ap, [t * di for di in d])


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_axis_angle(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def turn_angle(v, t, axis):
    """Angle [rad] about the unit axis that takes v to t (both across it)."""
    return math.atan2(dot(cross(v, t), axis), dot(v, t))


def align_to_stem(quat, up, pitch=True):
    """Turn the tool so that its up (-x) follows the stem direction up.

    First a roll about the approach axis (tool z) to the part of up across
    it; then, with pitch, a turn about the new tool y (the finger closing
    direction) to the lean toward or away from the hand, which tilts the
    approach too: it stays square to the stem. Returns (new quaternion,
    roll [deg], pitch [deg], lean left [deg]).
    """
    a = quat_apply(quat, (0.0, 0.0, 1.0))
    u0 = quat_apply(quat, (-1.0, 0.0, 0.0))
    along = dot(up, a)
    u = [x - along * y for x, y in zip(up, a)]
    n = math.hypot(*u)
    if n < 1e-6:
        return quat, 0.0, 0.0, 90.0
    u = [x / n for x in u]
    roll = turn_angle(u0, u, a)
    q = quat_mul(quat_axis_angle(a, roll), quat)
    lean_left = math.degrees(math.asin(min(1.0, abs(along))))
    tilt = 0.0
    if pitch:
        y = quat_apply(q, (0.0, 1.0, 0.0))
        tilt = turn_angle(u, up, y)
        q = quat_mul(quat_axis_angle(y, tilt), q)
        lean_left = 0.0
    return q, math.degrees(roll), math.degrees(tilt), lean_left


def in_front(point, quat):
    """PRE_APPROACH back from point along the tool's approach axis (z)."""
    a = quat_apply(quat, (0.0, 0.0, 1.0))
    return tuple(p - PRE_APPROACH * c for p, c in zip(point, a))


def quat_apply(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    # v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    ux = (y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0])
    uux = (y * ux[2] - z * ux[1], z * ux[0] - x * ux[2], x * ux[1] - y * ux[0])
    return tuple(v[i] + 2 * w * ux[i] + 2 * uux[i] for i in range(3))


def shape(name, center, prim, frame=FRAME):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center
    pose.orientation.w = 1.0
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


class HarvestDemo(Node):
    def __init__(self):
        super().__init__('harvest_demo')
        # false: only add the obstacles to the planning scene, then exit.
        self.declare_parameter('run_motions', True)
        self.declare_parameter('cycles', 0)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('velocity_scaling', 0.5)
        self.declare_parameter('use_camera', False)
        self.declare_parameter('detection_topic', '/fruit_detector/ripe')
        self.declare_parameter('unripe_topic', '/fruit_detector/unripe')
        self.declare_parameter('detection_timeout', 30.0)
        self.declare_parameter('approach', 'lin')         # lin or joint
        self.declare_parameter('approach_scaling', 0.2)   # Pilz LIN speed factor
        self.declare_parameter('use_wrist_camera', False)
        self.declare_parameter('wrist_topic', '/stem_detector/grasp')
        self.declare_parameter('wrist_timeout', 3.0)      # per view [s]
        # Views to try, as yaw [deg] about the vertical through the stem;
        # 0 is the pre-approach pose, +45 looks from the -y side.
        self.declare_parameter('wrist_view_yaws', [0.0, 45.0, -45.0])
        self.declare_parameter('if_unseen', 'skip')       # skip or guess
        self.declare_parameter('max_correction', 0.04)    # ignore larger [m]
        # Turn the hand about its approach axis to the stem's lean (needs
        # the wrist camera); at most max_roll [deg]. align_pitch also tilts
        # it to the lean toward or away from the hand, at most max_pitch.
        self.declare_parameter('align_roll', True)
        self.declare_parameter('align_pitch', True)
        self.declare_parameter('max_pitch', 35.0)
        # Append every planned arm trajectory to this file as JSON lines
        # ('' = off), for agri_arm's mujoco_dynamics --trajectories.
        self.declare_parameter('trajectory_log', '')
        self.declare_parameter('max_roll', 60.0)
        if self.get_parameter('approach').value not in ('lin', 'joint'):
            raise ValueError('approach must be lin or joint')
        if self.get_parameter('if_unseen').value not in ('skip', 'guess'):
            raise ValueError('if_unseen must be skip or guess')
        self.move = ActionClient(self, MoveGroup, 'move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.ik = self.create_client(GetPositionIK, 'compute_ik')
        self.fk = self.create_client(GetPositionFK, 'compute_fk')
        self.scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, 'check_state_validity')
        self.hand = ActionClient(self, FollowJointTrajectory,
                                 '/hand_controller/follow_joint_trajectory')
        self.set_pose = self.create_client(SetEntityPose, '/world/harvest_world/set_pose')
        # Gazebo detachable joints, see worlds/harvest_world.sdf.
        self.gz = {(m, part, action): self.create_publisher(Empty, f'/{m}/{part}/{action}', 1)
                   for m in FRUITS for part in ('hand', 'stem')
                   for action in ('attach', 'detach')}
        self.gz_poses = {}
        for m in FRUITS:
            self.create_subscription(PoseStamped, f'/model/{m}/pose',
                                     lambda msg, m=m: self.on_gz_pose(m, msg), 1)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.joint_state = None
        self.recorded = None      # joint states captured during execution
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.detections = None
        self.unripe = None
        self.wrist = None
        if self.get_parameter('use_wrist_camera').value:
            self.create_subscription(PoseStamped, self.get_parameter('wrist_topic').value,
                                     self.on_wrist, 1)
        self.fruit_ids = []       # fruit spheres currently in the planning scene
        if self.get_parameter('use_camera').value:
            self.create_subscription(PoseArray, self.get_parameter('detection_topic').value,
                                     self.on_detections, 1)
            self.create_subscription(PoseArray, self.get_parameter('unripe_topic').value,
                                     self.on_unripe, 1)

    def on_detections(self, msg):
        self.detections = msg

    def on_unripe(self, msg):
        self.unripe = msg

    def on_wrist(self, msg):
        self.wrist = msg

    def on_gz_pose(self, model, msg):
        p = msg.pose.position
        self.gz_poses[model] = (p.x, p.y, p.z)

    def on_joints(self, msg):
        self.joint_state = msg
        if self.recorded is not None:
            pos = dict(zip(msg.name, msg.position))
            if all(n in pos for n in JOINTS):
                self.recorded.append([pos[n] for n in JOINTS])

    # ----- helpers -----
    def wait(self, future, timeout=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def sleep(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def add_obstacles(self):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [
            box('post', POST_CENTER, POST_SIZE),
            box('branch', BRANCH_CENTER, BRANCH_SIZE),
            # Floor top 1 cm below the base so the pedestal is not in contact.
            box('floor', (0.0, 0.0, -0.06), (2.0, 2.0, 0.1)),
        ] + [box(name, c, size) for name, c, size in CRATE]
        self.apply(scene)

    def apply(self, scene):
        req = ApplyPlanningScene.Request(scene=scene)
        ok = self.wait(self.scene.call_async(req), 10.0)
        if not ok or not ok.success:
            raise RuntimeError('could not update the planning scene')

    def set_fruits(self, fruits):
        """Replace the fruit spheres (ripe and unripe) in the planning scene."""
        scene = PlanningScene(is_diff=True)
        for fid in self.fruit_ids:
            old = CollisionObject(id=fid, operation=CollisionObject.REMOVE)
            old.header.frame_id = FRAME
            scene.world.collision_objects.append(old)
        self.fruit_ids = [name for name, _ in fruits]
        scene.world.collision_objects += [sphere(name, xyz, FRUIT_RADIUS)
                                          for name, xyz in fruits]
        self.apply(scene)

    def attach_fruit(self, name, offset):
        """Move the fruit sphere from the world to the tool (it hangs below).

        Attaching an object with the id of a world object takes that object
        out of the world, so no separate remove is needed.
        """
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        held = AttachedCollisionObject(link_name=TIP, touch_links=HAND_LINKS)
        held.object = sphere(name, offset, FRUIT_RADIUS, frame=TIP)
        scene.robot_state.attached_collision_objects.append(held)
        self.apply(scene)
        self.fruit_ids.remove(name)

    def drop_fruit(self, name):
        """Detach the fruit from the tool and remove it from the scene."""
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        held = AttachedCollisionObject(link_name=TIP)
        held.object.id = name
        held.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(held)
        self.apply(scene)
        # Detaching puts it back into the world; remove it there too.
        scene = PlanningScene(is_diff=True)
        old = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        old.header.frame_id = FRAME
        scene.world.collision_objects.append(old)
        self.apply(scene)

    def gz_send(self, model, part, action):
        """Attach or detach a Gazebo detachable joint (part: hand or stem)."""
        pub = self.gz[(model, part, action)]
        end = time.monotonic() + 10.0
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError(f'nobody listens on /{model}/{part}/{action}')
        # A repeated attach or detach is ignored, so send a few times in
        # case the bridge drops one.
        for _ in range(3):
            pub.publish(Empty())
            self.sleep(0.1)
        self.sleep(0.2)       # applied on the next physics steps

    def set_hand(self, fingers, cutter, seconds=1.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = HAND_JOINTS
        pt = JointTrajectoryPoint(positions=[fingers, fingers, cutter])
        pt.time_from_start = Duration(seconds=seconds).to_msg()
        goal.trajectory.points = [pt]
        handle = self.wait(self.hand.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('hand goal rejected')
        result = self.wait(handle.get_result_async()).result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'hand move failed ({result.error_code})')

    def gz_pose(self, name, timeout=5.0):
        """Latest Gazebo position of a model, or None."""
        self.gz_poses.pop(name, None)
        end = time.monotonic() + timeout
        while rclpy.ok() and name not in self.gz_poses and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.gz_poses.get(name)

    def replant(self, names):
        """Hang the given fruit back on the plant at their world-file pose."""
        for name in names:
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            req.pose.position.x, req.pose.position.y, req.pose.position.z = FRUITS[name]
            req.pose.orientation.w = 1.0
            res = self.wait(self.set_pose.call_async(req), 5.0)
            if res is None or not res.success:
                raise RuntimeError(f'set_pose {name} failed')
            self.gz_send(name, 'stem', 'attach')
        self.sleep(1.0)
        for name in names:
            pos = self.gz_pose(name)
            err = None if pos is None else 1000 * math.dist(pos, FRUITS[name])
            self.get_logger().info(f'{name} hung back on the plant'
                                   + ('' if err is None else f', {err:.1f} mm from its pose'))

    def solve_ik(self, xyz, quat=APPROACH_QUAT):
        """Joint values that put tool0 at xyz with orientation quat, without collisions."""
        # Elbow-up seed: upper arm near vertical, forearm reaching down to
        # the wrist, base turned toward the wrist center behind the tool.
        back = quat_apply(quat, (0.0, 0.0, -0.19))
        seed = [math.atan2(xyz[1] + back[1], xyz[0] + back[0]), 0.0, 1.9, 0.0, -0.4, 0.0]
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = GROUP
        ik.ik_link_name = TIP
        ik.avoid_collisions = True
        ik.timeout.nanosec = 200_000_000
        ik.robot_state.joint_state.name = JOINTS
        ik.robot_state.joint_state.position = seed
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quat
        ik.pose_stamped = ps
        res = self.wait(self.ik.call_async(req), 5.0)
        if res is None or res.error_code.val != SUCCESS:
            return None
        pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [pos[n] for n in JOINTS]

    def joint_goal(self, values):
        return Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=v, tolerance_above=0.002,
                            tolerance_below=0.002, weight=1.0)
            for n, v in zip(JOINTS, values)])

    def pose_goal(self, xyz, quat=APPROACH_QUAT):
        """tool0 at xyz (within 1 mm) with orientation quat (default facing
        +x), for the Pilz LIN planner."""
        region = BoundingVolume()
        region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001]))
        center = Pose()
        center.position.x, center.position.y, center.position.z = xyz
        center.orientation.w = 1.0
        region.primitive_poses.append(center)
        pos = PositionConstraint(link_name=TIP, constraint_region=region, weight=1.0)
        pos.header.frame_id = FRAME
        ori = OrientationConstraint(link_name=TIP, weight=1.0, absolute_x_axis_tolerance=0.01,
                                    absolute_y_axis_tolerance=0.01,
                                    absolute_z_axis_tolerance=0.01)
        ori.header.frame_id = FRAME
        (ori.orientation.x, ori.orientation.y,
         ori.orientation.z, ori.orientation.w) = quat
        return Constraints(position_constraints=[pos], orientation_constraints=[ori])

    def plan(self, goal, lin=False):
        """Plan only, with OMPL or Pilz LIN. Returns (result, wall-clock seconds)."""
        req = MotionPlanRequest()
        req.group_name = GROUP
        if lin:
            req.pipeline_id = 'pilz_industrial_motion_planner'
            req.planner_id = 'LIN'
            req.num_planning_attempts = 1
            scale = self.get_parameter('approach_scaling').value
        else:
            req.pipeline_id = 'ompl'
            req.num_planning_attempts = 5
            scale = self.get_parameter('velocity_scaling').value
        req.allowed_planning_time = self.get_parameter('planning_time').value
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        req.start_state.is_diff = True
        req.goal_constraints = [goal]
        action = MoveGroup.Goal(request=req)
        action.planning_options.plan_only = True
        action.planning_options.planning_scene_diff.is_diff = True
        action.planning_options.planning_scene_diff.robot_state.is_diff = True
        t0 = time.monotonic()
        handle = self.wait(self.move.send_goal_async(action))
        if not handle.accepted:
            return None, 0.0
        result = self.wait(handle.get_result_async()).result
        return result, time.monotonic() - t0

    def run_trajectory(self, trajectory):
        self.recorded = []
        handle = self.wait(self.execute.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)))
        if not handle.accepted:
            return False
        return self.wait(handle.get_result_async()).result.error_code.val == SUCCESS

    def count_collisions(self, states):
        """Return (colliding, unchecked) counts for the given joint states."""
        bad = unchecked = 0
        for q in states:
            # A diff on the current state keeps the hand joints and any
            # fruit attached to the tool.
            state = RobotState(is_diff=True)
            state.joint_state.name = JOINTS
            state.joint_state.position = list(q)
            req = GetStateValidity.Request(robot_state=state, group_name=GROUP)
            res = self.wait(self.validity.call_async(req), 5.0)
            if res is None:
                unchecked += 1
            elif not res.valid:
                bad += 1
        return bad, unchecked

    def straight_move_collides(self, start, end, samples=40):
        """Collision counts on a straight joint-space line."""
        line = [[a + (b - a) * k / samples for a, b in zip(start, end)]
                for k in range(samples + 1)]
        return self.count_collisions(line)

    def fk_poses(self, states):
        """[(position, quaternion)] of tool0 in FRAME for joint states (compute_fk)."""
        out = []
        for q in states:
            req = GetPositionFK.Request(fk_link_names=[TIP])
            req.header.frame_id = FRAME
            req.robot_state.joint_state.name = JOINTS
            req.robot_state.joint_state.position = list(q)
            res = self.wait(self.fk.call_async(req), 5.0)
            if res is None or res.error_code.val != SUCCESS:
                raise RuntimeError('compute_fk failed')
            pose = res.pose_stamped[0].pose
            p, o = pose.position, pose.orientation
            out.append(((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))
        return out

    def tool_positions(self, states):
        """tool0 positions in FRAME for the given joint states."""
        return [p for p, _ in self.fk_poses(states)]

    def fruit_acceleration(self, points, offset, h=0.01):
        """Peak |a| and peak |a - g| [m/s²] of a held fruit's center.

        For each trajectory point the joints are moved along the local
        quadratic q(t) = q + qd t + qdd t²/2 at t = -h, 0, h; the second
        difference of the fruit center over that is its acceleration.
        """
        peak_a = peak_ag = 0.0
        for pt in points:
            q = list(pt.positions)
            qd = list(pt.velocities) or [0.0] * len(q)
            qdd = list(pt.accelerations) or [0.0] * len(q)
            states = [[x + v * t + 0.5 * a * t * t for x, v, a in zip(q, qd, qdd)]
                      for t in (-h, 0.0, h)]
            centers = []
            for p, o in self.fk_poses(states):
                off = quat_apply(o, offset)
                centers.append([pi + oi for pi, oi in zip(p, off)])
            acc = [(centers[0][i] + centers[2][i] - 2 * centers[1][i]) / (h * h)
                   for i in range(3)]
            peak_a = max(peak_a, math.hypot(*acc))
            peak_ag = max(peak_ag, math.hypot(*(a - g for a, g in zip(acc, GRAVITY))))
        return peak_a, peak_ag

    @staticmethod
    def off_line_mm(points, a, b):
        """Largest distance [mm] of points from the segment a-b."""
        ab = [bi - ai for ai, bi in zip(a, b)]
        n2 = sum(c * c for c in ab)
        if n2 == 0.0:     # a turn on the spot: distance from the point
            return 1000 * max((math.dist(p, a) for p in points), default=0.0)
        worst = 0.0
        for p in points:
            ap = [pi - ai for ai, pi in zip(a, p)]
            t = min(1.0, max(0.0, sum(x * y for x, y in zip(ap, ab)) / n2))
            closest = [ai + t * c for ai, c in zip(a, ab)]
            worst = max(worst, math.dist(p, closest))
        return 1000 * worst

    @staticmethod
    def fmt(counts, total):
        bad, unchecked = counts
        text = f'{bad}/{total} samples'
        if unchecked:
            text += f' ({unchecked} NOT CHECKED: validity service timed out)'
        return text

    def tool_pose(self):
        try:
            t = self.tf.lookup_transform(FRAME, TIP, Time()).transform
        except Exception as exc:  # noqa: BLE001 - tf raises several types
            self.get_logger().warn(f'tf lookup failed: {exc}')
            return None
        p, o = t.translation, t.rotation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    def tip_error_mm(self, xyz):
        self.sleep(0.5)
        pose = self.tool_pose()
        return None if pose is None else 1000 * math.dist(pose[0], xyz)

    def current_joints(self):
        js = self.joint_state
        if js is None:
            return None
        pos = dict(zip(js.name, js.position))
        return [pos.get(n, 0.0) for n in JOINTS]

    def wait_detection(self, attr, since, timeout):
        end = since + Duration(seconds=timeout)
        while rclpy.ok():
            msg = getattr(self, attr)
            if msg is not None and Time.from_msg(msg.header.stamp) >= since:
                return msg
            if self.get_clock().now() > end:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def find_fruits(self, cycle):
        """Ripe [(model, xyz)] to pick, left (+y) to right, and unripe [xyz].

        model is the Gazebo model of the true fruit within MATCH_DISTANCE,
        or None when the detection matches none.
        """
        if not self.get_parameter('use_camera').value:
            return list(FRUITS.items()), list(UNRIPE)
        # Use an image taken after the arm stopped at home.
        since = self.get_clock().now()
        timeout = self.get_parameter('detection_timeout').value
        msg = self.wait_detection('detections', since, timeout)
        if msg is None or not msg.poses:
            self.get_logger().error(f'no ripe fruit detected within {timeout:.0f} s')
            return None, None
        unripe_msg = self.wait_detection('unripe', since, timeout)
        unripe = [] if unripe_msg is None else [
            (p.position.x, p.position.y, p.position.z) for p in unripe_msg.poses]
        found = sorted(((p.position.x, p.position.y, p.position.z) for p in msg.poses),
                       key=lambda c: -c[1])
        fruits = []
        for i, xyz in enumerate(found, 1):
            model, truth = min(FRUITS.items(), key=lambda kv: math.dist(kv[1], xyz))
            err = math.dist(truth, xyz)
            self.get_logger().info(
                f'[{cycle}] camera ripe fruit {i} at ' + ' '.join(f'{v:+.3f}' for v in xyz)
                + f', {1000 * err:.1f} mm from the center of {model}')
            fruits.append((model if err <= MATCH_DISTANCE else None, xyz))
        for xyz in unripe:
            self.get_logger().info(f'[{cycle}] camera unripe fruit at '
                                   + ' '.join(f'{v:+.3f}' for v in xyz) + ' (obstacle)')
        return fruits, unripe

    # ----- main sequence -----
    def step(self, cycle, name, joints=None, target_xyz=None, line=None, final_approach=False,
             carrying=None, quat=APPROACH_QUAT):
        """Plan and run one move: a joint goal (OMPL), or with line=(a, b) a
        Pilz LIN move of tool0 from a to b, ending with orientation quat.

        final_approach: also report how far the tool strayed from the +x
        approach axis through target_xyz during the last PRE_APPROACH of
        travel, to compare with the LIN approach.
        carrying: offset of a held fruit's center in the tool frame; report
        its peak acceleration and the finger force needed to hold it by
        friction.
        """
        goal = self.pose_goal(line[1], quat) if line else self.joint_goal(joints)
        # The controller can still be starting on the first move, so retry.
        for _ in range(30):
            start = self.current_joints()
            result, wall = self.plan(goal, lin=line is not None)
            if result is not None and result.error_code.val == SUCCESS:
                if self.run_trajectory(result.planned_trajectory):
                    break
                self.get_logger().warn(f'{name}: execution failed, retrying')
            else:
                code = None if result is None else result.error_code.val
                self.get_logger().warn(f'{name}: planning failed (code {code}), retrying')
            self.sleep(1.0)
        else:
            self.get_logger().error(f'{name}: giving up')
            return False
        self.log_trajectory(name, result.planned_trajectory.joint_trajectory, carrying)
        executed = self.recorded[::5]     # joint_states arrive at 200 Hz
        self.recorded = None
        points = result.planned_trajectory.joint_trajectory.points
        duration = points[-1].time_from_start
        exec_hits = self.count_collisions(executed)
        text = (f'[{cycle}] {name}: planned in {wall:.2f} s, {len(points)} points, '
                f'motion {duration.sec + duration.nanosec * 1e-9:.2f} s, ')
        if line:
            # LIN starts from where the arm actually stopped, which can be
            # a little off the nominal start, so measure from there.
            planned = self.tool_positions([pt.positions for pt in points])
            a, b = planned[0], line[1]
            measured = self.tool_positions(executed)
            end = list(points[-1].positions)
            joint_line = [[s + (e - s) * k / 40 for s, e in zip(start, end)]
                          for k in range(41)]
            via_joints = self.tool_positions(joint_line)
            text += ('tool off the straight line: '
                     f'planned {self.off_line_mm(planned, a, b):.2f} mm, '
                     f'executed {self.off_line_mm(measured, a, b):.2f} mm '
                     f'(a joint-space move: {self.off_line_mm(via_joints, a, b):.1f} mm), ')
        else:
            hits = self.straight_move_collides(start, joints)
            text += f'straight joint move collides at {self.fmt(hits, 41)}, '
            if final_approach:
                tx, ty, tz = target_xyz
                axis_end = (tx - PRE_APPROACH, ty, tz)
                near = [p for p in self.tool_positions(executed)
                        if math.dist(p, target_xyz) <= PRE_APPROACH]
                text += (f'last {100 * PRE_APPROACH:.0f} cm off the approach axis '
                         f'{self.off_line_mm(near, axis_end, target_xyz):.1f} mm, ')
        if carrying is not None:
            peak_a, peak_ag = self.fruit_acceleration(points, carrying)
            force = FRUIT_MASS * peak_ag / (2 * FRICTION)
            text += (f'fruit peak acceleration {peak_a:.2f} m/s² '
                     f'(with gravity {peak_ag:.2f} m/s²: {force:.2f} N per finger '
                     f'at friction {FRICTION}), ')
        text += f'executed path collides at {self.fmt(exec_hits, len(executed))}'
        err = None if target_xyz is None else self.tip_error_mm(target_xyz)
        if err is not None:
            text += f', tool error {err:.1f} mm'
        self.get_logger().info(text)
        if err is not None and err > 20.0:
            self.get_logger().error(f'{name}: the tool did not reach the target')
            return False
        return True

    def log_trajectory(self, name, traj, carrying):
        path = self.get_parameter('trajectory_log').value
        if not path:
            return
        points = [{'t': p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                   'q': list(p.positions), 'qd': list(p.velocities) or [0.0] * len(p.positions),
                   'qdd': list(p.accelerations)}
                  for p in traj.points]
        with open(path, 'a') as f:
            f.write(json.dumps({'name': name, 'joints': list(traj.joint_names),
                                'carrying': carrying is not None, 'points': points}) + '\n')

    def fruit_offset(self, model):
        """Gazebo fruit center in the tool frame, or straight below by default."""
        pose, fruit = self.tool_pose(), self.gz_pose(model)
        if pose is None or fruit is None:
            return (GRASP_ABOVE, 0.0, 0.0)
        p, o = pose
        return quat_apply(quat_conj(o), [fi - pi for fi, pi in zip(fruit, p)])

    @staticmethod
    def stem_alignment(pose, a, d):
        """Text: angle of the hand's up axis to the stem line (a, d), and how
        far sideways from the finger center the blade meets that stem."""
        p, o = pose
        inv = quat_conj(o)
        at = quat_apply(inv, [ai - pi for ai, pi in zip(a, p)])
        dt = quat_apply(inv, d)
        angle = math.degrees(math.acos(min(1.0, abs(dt[0]))))
        text = f'hand up axis {angle:.1f}° from the stem'
        if abs(dt[0]) > 1e-6:
            # the blade moves along tool y in the plane x = -CUT_ABOVE;
            # deep is along the approach (tool z), + past the finger center
            s = (-CUT_ABOVE - at[0]) / dt[0]
            text += (f', blade meets it {1000 * (at[1] + s * dt[1]):+.1f} mm sideways'
                     f' {1000 * (at[2] + s * dt[2]):+.1f} mm deep')
        return text

    def check_held(self, cycle, model, offset):
        """Log how far the Gazebo fruit is from where the hand holds it."""
        pose, fruit = self.tool_pose(), self.gz_pose(model)
        if pose is None or fruit is None:
            self.get_logger().warn(f'[{cycle}] {model}: no pose to check the hold')
            return
        p, o = pose
        expected = [pi + oi for pi, oi in zip(p, quat_apply(o, offset))]
        self.get_logger().info(f'[{cycle}] {model} held: center {1000 * math.dist(fruit, expected):.1f}'
                               ' mm from where it was at the grasp')

    def wrist_correction(self, cycle, model, stem, view):
        """(grasp point, stem direction up) seen by the wrist camera, or None."""
        since = self.get_clock().now()
        timeout = self.get_parameter('wrist_timeout').value
        msg = self.wait_detection('wrist', since, timeout)
        if msg is None:
            self.get_logger().warn(f'[{cycle}] {model}: wrist camera saw no stem from the '
                                   f'{view} view within {timeout:.0f} s')
            return None
        p, o = msg.pose.position, msg.pose.orientation
        seen = (p.x, p.y, p.z)
        up = quat_apply((o.x, o.y, o.z, o.w), (0.0, 0.0, 1.0))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        shift = math.dist(seen, stem)
        text = (f'[{cycle}] {model} wrist camera ({view} view): stem at '
                + ' '.join(f'{v:+.3f}' for v in seen)
                + f', tilt {tilt:.0f}°, {1000 * shift:.1f} mm from the straight-up guess')
        if model in STEMS:
            a, d = STEMS[model]
            text += (f'; off the true stem: guess {point_line_mm(stem, a, d):.1f} mm, '
                     f'wrist camera {point_line_mm(seen, a, d):.1f} mm')
        self.get_logger().info(text)
        if shift > self.get_parameter('max_correction').value:
            self.get_logger().warn(f'[{cycle}] {model}: correction too large, not used')
            return None
        return seen, up

    def look_at_stem(self, cycle, model, stem):
        """Wrist camera grasp point from the first view that sees the stem.

        Returns (seen, moved): seen is (grasp point, stem direction) or
        None if no view saw it; moved is True when the hand left the
        pre-approach pose for a side view.
        """
        moved = False
        for yaw in self.get_parameter('wrist_view_yaws').value:
            view = 'front' if yaw == 0.0 else f'{yaw:+.0f}°'
            if yaw != 0.0:
                # Same distance from the stem, turned about the vertical.
                c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                xyz = (stem[0] - PRE_APPROACH * c, stem[1] - PRE_APPROACH * s, stem[2])
                quat = quat_mul((0.0, 0.0, math.sin(math.radians(yaw) / 2),
                                 math.cos(math.radians(yaw) / 2)), APPROACH_QUAT)
                joints = self.solve_ik(xyz, quat)
                if joints is None:
                    self.get_logger().warn(f'[{cycle}] {model}: no IK for the {view} view')
                    continue
                if not self.step(cycle, f'{model} {view} view', joints, xyz):
                    continue
                moved = True
            seen = self.wrist_correction(cycle, model, stem, view)
            if seen is not None:
                return seen, moved
        return None, moved

    def check_in_crate(self, cycle, model):
        self.sleep(1.5)       # let it fall and settle
        pos = self.gz_pose(model)
        if pos is None:
            self.get_logger().warn(f'[{cycle}] {model}: no pose from Gazebo')
            return False
        inside = all(lo <= v <= hi for v, (lo, hi) in zip(pos, CRATE_INSIDE))
        self.get_logger().info(f'[{cycle}] {model} released at '
                               + ' '.join(f'{v:+.3f}' for v in pos)
                               + (': in the crate' if inside else ': NOT in the crate'))
        return inside

    def turn_to_stem(self, cycle, model, up):
        """Approach orientation turned to the seen stem direction up."""
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        pitch = p('align_pitch')
        turned, roll, tilt, left = align_to_stem(APPROACH_QUAT, up, pitch)
        if abs(roll) > p('max_roll'):
            self.get_logger().warn(f'[{cycle}] {model}: roll {roll:+.0f}° is more '
                                   f'than {p("max_roll"):.0f}°, not turned')
            return APPROACH_QUAT
        if abs(tilt) > p('max_pitch'):
            self.get_logger().warn(f'[{cycle}] {model}: pitch {tilt:+.0f}° is more than '
                                   f'{p("max_pitch"):.0f}°, only rolled')
            turned, roll, tilt, left = align_to_stem(APPROACH_QUAT, up, False)
        text = f'[{cycle}] {model}: hand turned {roll:+.1f}° to the stem'
        if pitch and left == 0.0:
            text += f' and tilted {tilt:+.1f}°'
        else:
            text += f' ({left:.1f}° of lean toward or away from the hand left)'
        self.get_logger().info(text)
        return turned

    def harvest(self, cycle, model, first, stem, joints, drop_joints, lin):
        """Approach the stem, grasp, cut, carry to the crate and release.

        Returns 'picked', 'skipped' (stem not seen, nothing touched) or
        'failed'.
        """
        quat = APPROACH_QUAT
        if lin:
            if not self.step(cycle, f'{model} pre-approach', joints, first):
                return 'failed'
            if self.get_parameter('use_wrist_camera').value:
                found, moved = self.look_at_stem(cycle, model, stem)
                if found is None:
                    if self.get_parameter('if_unseen').value == 'skip':
                        self.get_logger().warn(f'[{cycle}] {model}: stem not seen from any '
                                               'view; skipped, left on the plant')
                        return 'skipped'
                    self.get_logger().warn(f'[{cycle}] {model}: stem not seen from any '
                                           'view; grasping at the straight-up guess')
                    seen, up = stem, None
                else:
                    seen, up = found
                if up is not None and self.get_parameter('align_roll').value:
                    quat = self.turn_to_stem(cycle, model, up)
                # In front of the seen stem, then straight in along the
                # tool's approach axis (+x unless pitched): by LIN from the
                # pre-approach pose (turning the hand on the way), or back
                # from a side view by OMPL.
                aligned = in_front(seen, quat)
                if moved:
                    back_joints = self.solve_ik(aligned, quat)
                    if back_joints is None or not self.step(
                            cycle, f'{model} back in front', back_joints, aligned):
                        return 'failed'
                elif (seen != stem or quat != APPROACH_QUAT) and not self.step(
                        cycle, f'{model} LIN align', target_xyz=aligned,
                        line=(first, aligned), quat=quat):
                    return 'failed'
                first, stem = aligned, seen
            if not self.step(cycle, f'{model} LIN approach', target_xyz=stem,
                             line=(first, stem), quat=quat):
                return 'failed'
        elif not self.step(cycle, model, joints, stem, final_approach=True):
            return 'failed'
        pose = self.tool_pose()
        if model in STEMS and pose is not None:
            a, d = STEMS[model]
            self.get_logger().info(f'[{cycle}] {model}: finger center '
                                   f'{point_line_mm(pose[0], a, d):.1f} mm from the true stem, '
                                   + self.stem_alignment(pose, a, d))
        # Grasp the stem, then cut above the fingers. In Gazebo the hand
        # joint holds the fruit and detaching the stem joint is the cut.
        self.set_hand(FINGERS_CLOSED, CUTTER_OPEN)
        self.gz_send(model, 'hand', 'attach')
        self.set_hand(FINGERS_CLOSED, CUTTER_CLOSED, 0.5)
        self.gz_send(model, 'stem', 'detach')
        self.set_hand(FINGERS_CLOSED, CUTTER_OPEN, 0.5)
        offset = self.fruit_offset(model)
        self.attach_fruit(model, offset)
        self.get_logger().info(f'[{cycle}] {model}: stem grasped and cut')
        back = in_front(stem, quat)
        if lin:
            ok = self.step(cycle, f'{model} LIN retreat', target_xyz=back,
                           line=(stem, back), carrying=offset, quat=quat)
        else:
            ok = self.step(cycle, f'{model} retreat', self.solve_ik(back) or joints, back,
                           carrying=offset)
        if not ok:
            return 'failed'
        self.check_held(cycle, model, offset)
        if not self.step(cycle, f'{model} to crate', drop_joints, DROP_XYZ, carrying=offset):
            return 'failed'
        self.check_held(cycle, model, offset)
        self.set_hand(FINGERS_OPEN, CUTTER_OPEN)
        self.gz_send(model, 'hand', 'detach')
        self.drop_fruit(model)
        return 'picked' if self.check_in_crate(cycle, model) else 'failed'

    def release_arm(self):
        """Gazebo attaches the hand joints when the arm is spawned: undo that.

        The joints only listen once the arm exists, so wait for its joint
        states, then repeat for a few seconds while Gazebo's transport
        connects.
        """
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(5):
            for model in FRUITS:
                self.gz_send(model, 'hand', 'detach')
            self.sleep(0.5)

    def run(self):
        self.get_logger().info('waiting for move_group')
        self.scene.wait_for_service()
        self.add_obstacles()
        self.release_arm()
        self.get_logger().info('hand joints released')
        if not self.get_parameter('run_motions').value:
            self.get_logger().info('obstacles added; motions disabled')
            return
        self.move.wait_for_server()
        self.execute.wait_for_server()
        self.hand.wait_for_server()
        self.ik.wait_for_service()
        self.fk.wait_for_service()
        self.validity.wait_for_service()
        self.set_pose.wait_for_service()
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.set_hand(FINGERS_OPEN, CUTTER_OPEN)
        # Detection must see the plant from home, so start there.
        current = self.current_joints()
        if (self.get_parameter('use_camera').value
                and max(abs(v) for v in current) > 0.01 and not self.step(0, 'home', HOME)):
            return
        drop_joints = self.solve_ik(DROP_XYZ, DROP_QUAT)
        if drop_joints is None:
            self.get_logger().error(f'no IK solution above the crate at {DROP_XYZ}')
            return
        cycles = self.get_parameter('cycles').value
        lin = self.get_parameter('approach').value == 'lin'
        cycle = 0
        while rclpy.ok() and (cycles <= 0 or cycle < cycles):
            cycle += 1
            fruits, unripe = self.find_fruits(cycle)
            if fruits is None:
                return
            self.set_fruits([(model or f'unknown_{i}', xyz) for i, (model, xyz)
                             in enumerate(fruits)]
                            + [(f'unripe_{i}', xyz) for i, xyz in enumerate(unripe)])
            targets = []
            for model, (x, y, z) in fruits:
                if model is None:
                    self.get_logger().warn(f'[{cycle}] fruit at ({x:.3f}, {y:.3f}, {z:.3f}) '
                                           'matches no Gazebo model; left as an obstacle')
                    continue
                # Fingers on the stem above the fruit; approach along +x.
                stem = (x, y, z + GRASP_ABOVE)
                first = (x - PRE_APPROACH, y, stem[2]) if lin else stem
                joints = self.solve_ik(first)
                if joints is None:
                    self.get_logger().error(f'no IK solution for {model} at {first}')
                    return
                self.get_logger().info(
                    f'[{cycle}] {model} IK: ' + ' '.join(f'{v:+.2f}' for v in joints))
                targets.append((model, first, stem, joints))
            picked, skipped = [], []
            for model, first, stem, joints in targets:
                status = self.harvest(cycle, model, first, stem, joints, drop_joints, lin)
                if status == 'failed':
                    self.get_logger().error(f'[{cycle}] {model}: harvest failed')
                    return
                (picked if status == 'picked' else skipped).append(model)
            self.get_logger().info(f'[{cycle}] picked {len(picked)}/{len(targets)}'
                                   + (f', skipped {" ".join(skipped)}' if skipped else ''))
            if not self.step(cycle, 'home', HOME):
                return
            self.replant(picked)
            self.sleep(1.0)
        self.get_logger().info('done')


def main():
    rclpy.init()
    node = HarvestDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
