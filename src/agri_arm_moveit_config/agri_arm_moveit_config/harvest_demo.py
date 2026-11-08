"""Plan collision-free harvesting moves with MoveIt 2 and run them in Gazebo.

The sequence is home -> left fruit -> right fruit -> home. A support post
stands between the two fruits, so a straight joint-space move from one fruit
to the other would hit it. With use_camera the fruit positions come from
agri_perception's fruit_detector, read again at home before every cycle,
and their error from the true positions in the world file is logged.
Each fruit pose is turned into joint values with
collision-aware IK, seeded with an elbow-up posture so the solution is a
natural one. The fruit themselves are added to the planning scene as
spheres.

With approach:=lin (default) OMPL brings the tool to a pre-approach pose
PRE_APPROACH in front of the fruit; the Pilz LIN planner then moves the
tool in a straight line to the pick pose and back out. With
approach:=joint OMPL goes straight to the pick pose, as before.

For every move the node logs the planning time, whether the joint
states measured during execution collided, and the tool position error.
Joint moves also report whether a straight joint-space move would have
collided; LIN moves report how far the tool left the straight line, for
the planned and the executed path, and how far a joint-space move between
the same two poses would have strayed.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, MotionPlanRequest, OrientationConstraint,
                             PlanningScene, PositionConstraint, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionFK, GetPositionIK,
                             GetStateValidity)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

GROUP = 'arm'
TIP = 'tool0'
FRAME = 'world'
JOINTS = [f'joint{i}' for i in range(1, 7)]

# Must match worlds/harvest_world.sdf. With use_camera, FRUITS is only
# used as ground truth to report the detection error.
POST_CENTER = (0.42, 0.0, 0.45)
POST_SIZE = (0.04, 0.04, 0.9)
FRUITS = {'left': (0.42, 0.20, 0.45), 'right': (0.42, -0.20, 0.45)}
FRUIT_RADIUS = 0.035
STANDOFF = 0.04           # tool0 stops this far in front of the fruit center [m]
PRE_APPROACH = 0.10       # LIN approach starts this far in front [m]
# Tool z axis pointing along +x: rotate +90 degrees about y.
APPROACH_QUAT = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
HOME = [0.0] * 6

SUCCESS = 1


def box(name, center, size):
    return shape(name, center, SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))


def sphere(name, center, radius):
    return shape(name, center, SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius]))


def shape(name, center, prim):
    obj = CollisionObject()
    obj.header.frame_id = FRAME
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
        self.declare_parameter('detection_timeout', 30.0)
        self.declare_parameter('approach', 'lin')         # lin or joint
        self.declare_parameter('approach_scaling', 0.2)   # Pilz LIN speed factor
        if self.get_parameter('approach').value not in ('lin', 'joint'):
            raise ValueError('approach must be lin or joint')
        self.move = ActionClient(self, MoveGroup, 'move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.ik = self.create_client(GetPositionIK, 'compute_ik')
        self.fk = self.create_client(GetPositionFK, 'compute_fk')
        self.scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, 'check_state_validity')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.joint_state = None
        self.recorded = None      # joint states captured during execution
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.detections = None
        self.fruit_ids = []       # fruit spheres currently in the planning scene
        if self.get_parameter('use_camera').value:
            self.create_subscription(PoseArray, self.get_parameter('detection_topic').value,
                                     self.on_detections, 1)

    def on_detections(self, msg):
        self.detections = msg

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
            # Floor top 1 cm below the base so the pedestal is not in contact.
            box('floor', (0.0, 0.0, -0.06), (2.0, 2.0, 0.1)),
        ]
        self.apply(scene)

    def apply(self, scene):
        req = ApplyPlanningScene.Request(scene=scene)
        ok = self.wait(self.scene.call_async(req), 10.0)
        if not ok or not ok.success:
            raise RuntimeError('could not update the planning scene')

    def set_fruits(self, fruits):
        """Replace the fruit spheres in the planning scene."""
        scene = PlanningScene(is_diff=True)
        for fid in self.fruit_ids:
            old = CollisionObject(id=fid, operation=CollisionObject.REMOVE)
            old.header.frame_id = FRAME
            scene.world.collision_objects.append(old)
        self.fruit_ids = [name for name, _ in fruits]
        scene.world.collision_objects += [sphere(name, xyz, FRUIT_RADIUS)
                                          for name, xyz in fruits]
        self.apply(scene)

    def solve_ik(self, xyz):
        """Joint values that put tool0 at xyz facing +x, without collisions."""
        x, y, z = xyz
        # Elbow-up seed: upper arm near vertical, forearm reaching down to
        # the wrist, base turned toward the wrist center behind the tool.
        seed = [math.atan2(y, x - 0.19), 0.0, 1.9, 0.0, -0.4, 0.0]
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
         ps.pose.orientation.z, ps.pose.orientation.w) = APPROACH_QUAT
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

    def pose_goal(self, xyz):
        """tool0 at xyz (within 1 mm) facing +x, for the Pilz LIN planner."""
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
         ori.orientation.z, ori.orientation.w) = APPROACH_QUAT
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
            state = RobotState()
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

    def tool_positions(self, states):
        """tool0 positions in FRAME for the given joint states (compute_fk)."""
        out = []
        for q in states:
            req = GetPositionFK.Request(fk_link_names=[TIP])
            req.header.frame_id = FRAME
            req.robot_state.joint_state.name = JOINTS
            req.robot_state.joint_state.position = list(q)
            res = self.wait(self.fk.call_async(req), 5.0)
            if res is None or res.error_code.val != SUCCESS:
                raise RuntimeError('compute_fk failed')
            p = res.pose_stamped[0].pose.position
            out.append((p.x, p.y, p.z))
        return out

    @staticmethod
    def off_line_mm(points, a, b):
        """Largest distance [mm] of points from the segment a-b."""
        ab = [bi - ai for ai, bi in zip(a, b)]
        n2 = sum(c * c for c in ab)
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

    def tip_error_mm(self, xyz):
        self.sleep(0.5)
        try:
            t = self.tf.lookup_transform(FRAME, TIP, Time()).transform.translation
        except Exception as exc:  # noqa: BLE001 - tf raises several types
            self.get_logger().warn(f'tf lookup failed: {exc}')
            return None
        return 1000 * math.dist((t.x, t.y, t.z), xyz)

    def current_joints(self):
        js = self.joint_state
        if js is None:
            return None
        pos = dict(zip(js.name, js.position))
        return [pos.get(n, 0.0) for n in JOINTS]

    def find_fruits(self, cycle):
        """[(name, xyz)] of the fruit to pick, left (+y) to right."""
        if not self.get_parameter('use_camera').value:
            return [(f'fruit_{side}', xyz) for side, xyz in FRUITS.items()]
        # Use an image taken after the arm stopped at home.
        since = self.get_clock().now()
        timeout = self.get_parameter('detection_timeout').value
        end = since + Duration(seconds=timeout)
        while rclpy.ok():
            msg = self.detections
            if msg is not None and Time.from_msg(msg.header.stamp) >= since and msg.poses:
                break
            if self.get_clock().now() > end:
                self.get_logger().error(f'no ripe fruit detected within {timeout:.0f} s')
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        found = sorted(((p.position.x, p.position.y, p.position.z) for p in msg.poses),
                       key=lambda c: -c[1])
        fruits = []
        for i, xyz in enumerate(found, 1):
            truth = min(FRUITS.values(), key=lambda t: math.dist(t, xyz))
            self.get_logger().info(
                f'[{cycle}] camera fruit_{i} at ' + ' '.join(f'{v:+.3f}' for v in xyz)
                + f', {1000 * math.dist(truth, xyz):.1f} mm from the true center')
            fruits.append((f'fruit_{i}', xyz))
        return fruits

    # ----- main sequence -----
    def step(self, cycle, name, joints=None, target_xyz=None, line=None, final_approach=False):
        """Plan and run one move: a joint goal (OMPL), or with line=(a, b) a
        Pilz LIN move of tool0 from a to b.

        final_approach: also report how far the tool strayed from the +x
        approach axis through target_xyz during the last PRE_APPROACH -
        STANDOFF of travel, to compare with the LIN approach.
        """
        goal = self.pose_goal(line[1]) if line else self.joint_goal(joints)
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
                axis_end = (tx - (PRE_APPROACH - STANDOFF), ty, tz)
                near = [p for p in self.tool_positions(executed)
                        if math.dist(p, target_xyz) <= PRE_APPROACH - STANDOFF]
                text += (f'last {100 * (PRE_APPROACH - STANDOFF):.0f} cm off the approach axis '
                         f'{self.off_line_mm(near, axis_end, target_xyz):.1f} mm, ')
        text += f'executed path collides at {self.fmt(exec_hits, len(executed))}'
        if target_xyz is not None:
            err = self.tip_error_mm(target_xyz)
            if err is not None:
                text += f', tool error {err:.1f} mm'
        self.get_logger().info(text)
        return True

    def run(self):
        self.get_logger().info('waiting for move_group')
        self.scene.wait_for_service()
        self.add_obstacles()
        if not self.get_parameter('run_motions').value:
            self.get_logger().info('obstacles added; motions disabled')
            return
        self.move.wait_for_server()
        self.execute.wait_for_server()
        self.ik.wait_for_service()
        self.fk.wait_for_service()
        self.validity.wait_for_service()
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        # Detection must see the plant from home, so start there.
        current = self.current_joints()
        if (self.get_parameter('use_camera').value
                and max(abs(v) for v in current) > 0.01 and not self.step(0, 'home', HOME)):
            return
        cycles = self.get_parameter('cycles').value
        cycle = 0
        while rclpy.ok() and (cycles <= 0 or cycle < cycles):
            cycle += 1
            fruits = self.find_fruits(cycle)
            if fruits is None:
                return
            self.set_fruits(fruits)
            lin = self.get_parameter('approach').value == 'lin'
            targets = []
            for name, (x, y, z) in fruits:
                # The tool approaches along +x, so stand off in -x.
                pick = (x - STANDOFF, y, z)
                first = (x - PRE_APPROACH, y, z) if lin else pick
                joints = self.solve_ik(first)
                if joints is None:
                    self.get_logger().error(f'no IK solution for {name} at {first}')
                    return
                self.get_logger().info(
                    f'[{cycle}] {name} IK: ' + ' '.join(f'{v:+.2f}' for v in joints))
                targets.append((name, first, pick, joints))
            for name, first, pick, joints in targets:
                if not lin:
                    if not self.step(cycle, name, joints, pick, final_approach=True):
                        return
                    self.sleep(1.0)
                    continue
                if not (self.step(cycle, f'{name} pre-approach', joints, first)
                        and self.step(cycle, f'{name} LIN approach', target_xyz=pick,
                                      line=(first, pick))):
                    return
                self.sleep(0.5)       # where the cutter would act
                if not self.step(cycle, f'{name} LIN retreat', target_xyz=first,
                                 line=(pick, first)):
                    return
            if not self.step(cycle, 'home', HOME):
                return
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
