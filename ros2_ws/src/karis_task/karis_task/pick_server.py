"""
KARIS pick action server: the link from vision to motion planning and execution.

    ros2 run karis_task pick_server --ros-args -p planning_group:=right_arm -p ee_link:=right_wrist_yaw_link
    ros2 action send_goal /karis/pick karis_interfaces/action/PickObject \
        "{class_name: 'Rund mutter', place_slot: 'Rund mutter HOLE 11432713', dry_run: true}" --feedback

For each goal it
  1. asks the perception for the part (/karis/get_objects, pose in base_frame, newest frame),
  2. computes a pre-grasp pose (approach_height above the part), the grasp pose and a lift pose,
     and the same for the slot when place_slot is set,
  3. publishes them on /karis/task/target_pose (PoseStamped, view in RViz), and
  4. unless dry_run (or execute:=false): sends each pose to MoveIt's move_group
     (moveit_msgs/action/MoveGroup on /move_action) for the arm `planning_group`,
     with the hand link `ee_link`. MoveIt plans, and executes when execute:=true.

Closing / opening the Inspire hand is not done here yet (see the TODOs in run()).
ee_link is the wrist, not the point between the fingers: grasp_offset_z (how far above
the part's top the wrist stops) and grasp_rpy_deg (hand orientation in base_frame) must
be measured on the robot. The part being picked is taken out of MoveIt's planning
scene (/karis/task/active_object) so the hand may touch it. One goal at a time.
Without a MoveIt config for the G1 (move_group not running), use dry_run: true; the
poses are still computed and published, which checks the whole vision -> motion chain.

Parameters: base_frame (pelvis), planning_group (right_arm), ee_link (right_wrist_yaw_link),
approach_height (0.08 m), grasp_offset_z (0.0 m), lift_height (0.10 m),
grasp_rpy_deg ([0, 90, 0]), execute (false = plan only), velocity_scale (0.2),
planning_time (5.0 s),
grasp_quat_xyzw ([0,0,0,0] = use grasp_rpy_deg; else this hand orientation in base_frame),
wrist_offset_xyz ([0,0,0] m, added to every wrist target, e.g. a measured wrist-to-part offset),
z_reference ("top": heights from the part's top, "centre": from the part's centre),
approach_only (false; true = only move to the approach pose above the part, then stop).

Example, the wrist 10 cm above a measured grasp pose (offset and orientation from the
team's arm test), arm moves once and stops:
    ros2 run karis_task pick_server --ros-args -p execute:=true -p approach_only:=true \
      -p z_reference:=centre -p wrist_offset_xyz:="[-0.197, -0.036, 0.1127]" \
      -p grasp_quat_xyzw:="[-0.7087, 0.0589, 0.0797, 0.6985]" -p approach_height:=0.10
"""
import math
import time

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from karis_interfaces.action import PickObject
from karis_interfaces.srv import GetObjects


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class PickServer(Node):
    def __init__(self):
        super().__init__("karis_pick_server")
        p = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.base_frame = p("base_frame", "pelvis")
        self.group = p("planning_group", "right_arm")
        self.ee_link = p("ee_link", "right_wrist_yaw_link")
        self.approach = float(p("approach_height", 0.08))
        self.grasp_dz = float(p("grasp_offset_z", 0.0))
        self.grasp_quat = [float(v) for v in p("grasp_quat_xyzw", [0.0, 0.0, 0.0, 0.0])]
        self.wrist_off = [float(v) for v in p("wrist_offset_xyz", [0.0, 0.0, 0.0])]
        self.z_ref = str(p("z_reference", "top"))
        self.approach_only = bool(p("approach_only", False))
        self.lift = float(p("lift_height", 0.10))
        self.grasp_rpy = [math.radians(a) for a in p("grasp_rpy_deg", [0.0, 90.0, 0.0])]
        self.execute = bool(p("execute", False))
        self.vel = float(p("velocity_scale", 0.2))
        self.plan_time = float(p("planning_time", 5.0))

        cb = ReentrantCallbackGroup()
        self.get_objects = self.create_client(GetObjects, "/karis/get_objects", callback_group=cb)
        self.target_pub = self.create_publisher(PoseStamped, "/karis/task/target_pose", 10)
        self.active_pub = self.create_publisher(String, "/karis/task/active_object", 10)
        self.busy = False
        self.move_group = None
        try:
            from moveit_msgs.action import MoveGroup
            self.MoveGroup = MoveGroup
            self.move_group = ActionClient(self, MoveGroup, "/move_action", callback_group=cb)
        except ImportError:
            self.get_logger().warn("moveit_msgs not found: only dry runs are possible")
        self.server = ActionServer(self, PickObject, "/karis/pick", self.run,
                                   goal_callback=self.on_goal, callback_group=cb)
        self.get_logger().info(f"/karis/pick ready (base '{self.base_frame}', group '{self.group}', "
                               f"ee '{self.ee_link}', execute={self.execute})")

    def on_goal(self, goal):
        if self.busy:
            self.get_logger().warn("rejected: a pick is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def set_active(self, oid):
        m = String()
        m.data = oid
        self.active_pub.publish(m)

    # ------------------------------------------------------------ helpers
    def wait(self, future, timeout):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.01)
        return future.result()

    def locate(self, class_name, object_id):
        if not self.get_objects.wait_for_service(timeout_sec=2.0):
            return None, "perception is not running (/karis/get_objects not available)"
        req = GetObjects.Request()
        req.class_name, req.object_id = class_name, object_id
        req.target_frame, req.fresh = self.base_frame, True
        res = self.wait(self.get_objects.call_async(req), 5.0)
        if res is None:
            return None, "perception did not answer"
        if not res.success:
            return None, res.message
        objs = [o for o in res.objects.objects if o.has_orientation or not object_id]
        # the part nearest the robot base first
        objs.sort(key=lambda o: math.hypot(o.box_pose.position.x, o.box_pose.position.y))
        return (objs[0], res.objects.header) if objs else (None, "no matching part")

    def pose_at(self, header, position, dz):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = Pose()
        ps.pose.position.x = position.x + self.wrist_off[0]
        ps.pose.position.y = position.y + self.wrist_off[1]
        ps.pose.position.z = position.z + dz + self.wrist_off[2]
        n = math.sqrt(sum(v * v for v in self.grasp_quat))
        q = [v / n for v in self.grasp_quat] if n > 1e-6 else quat_from_rpy(*self.grasp_rpy)
        o = ps.pose.orientation
        o.x, o.y, o.z, o.w = q
        return ps

    def move(self, target, plan_only):
        """Plan (and execute) a move of ee_link to target with MoveIt. Returns (ok, text)."""
        self.target_pub.publish(target)
        if self.move_group is None:
            return False, "moveit_msgs not installed"
        if not self.move_group.wait_for_server(timeout_sec=2.0):
            return False, "MoveIt move_group is not running (/move_action)"
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        goal = self.MoveGroup.Goal()
        r = goal.request
        r.group_name = self.group
        r.num_planning_attempts = 5
        r.allowed_planning_time = self.plan_time
        r.max_velocity_scaling_factor = self.vel
        r.max_acceleration_scaling_factor = self.vel
        r.workspace_parameters.header.frame_id = self.base_frame
        r.start_state.is_diff = True                      # start from the current robot state
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        pc = PositionConstraint()
        pc.header = target.header
        pc.link_name = self.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [target.pose]
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header = target.header
        oc.link_name = self.ee_link
        oc.orientation = target.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.3
        oc.weight = 1.0
        c = Constraints()
        c.position_constraints, c.orientation_constraints = [pc], [oc]
        r.goal_constraints = [c]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        gh = self.wait(self.move_group.send_goal_async(goal), 5.0)
        if gh is None or not gh.accepted:
            return False, "move_group rejected the goal"
        res = self.wait(gh.get_result_async(), self.plan_time + 60.0)
        if res is None:
            gh.cancel_goal_async()                       # stop the arm
            return False, "move_group timed out"
        code = res.result.error_code.val
        return code == 1, f"MoveIt error code {code}"     # 1 = SUCCESS

    # ------------------------------------------------------------ the action
    def run(self, gh):
        self.busy = True
        try:
            return self._run(gh)
        finally:
            self.set_active("")
            self.busy = False

    def _run(self, gh):
        g = gh.request
        result = PickObject.Result()
        fb = PickObject.Feedback()

        def step(state, progress):
            fb.state, fb.progress = state, progress
            gh.publish_feedback(fb)
            self.get_logger().info(f"  {state}")

        step("locating", 0.05)
        part, hdr = self.locate(g.class_name if not g.object_id else "", g.object_id)
        if part is None:
            gh.abort()
            result.message = f"part not found: {hdr}"
            return result
        self.get_logger().info(f"Pick {part.object_id} at "
                               f"({part.box_pose.position.x:+.3f} {part.box_pose.position.y:+.3f} "
                               f"{part.box_pose.position.z:+.3f}) in {self.base_frame}")
        self.set_active(part.object_id)
        c = part.box_pose.position
        top = part.box_size.z / 2 if self.z_ref == "top" else 0.0
        moves = [("approaching", self.pose_at(hdr, c, top + self.approach)),
                 ("grasping", self.pose_at(hdr, c, top + self.grasp_dz)),
                 ("lifting", self.pose_at(hdr, c, top + self.lift))]
        result.grasp_pose = moves[1][1]
        if self.approach_only:
            moves = moves[:1]
            self.get_logger().info("approach_only: moving only to the approach pose")

        if g.place_slot and not self.approach_only:
            slot, shdr = self.locate(g.place_slot, "")
            if slot is None:
                gh.abort()
                result.message = f"slot '{g.place_slot}' not found: {shdr}"
                return result
            s = slot.box_pose.position
            moves += [("moving to slot", self.pose_at(shdr, s, part.box_size.z + self.approach)),
                      ("placing", self.pose_at(shdr, s, part.box_size.z / 2 + self.grasp_dz)),
                      ("retreating", self.pose_at(shdr, s, part.box_size.z + self.approach))]
            result.place_pose = moves[4][1]

        plan_only = g.dry_run or not self.execute
        for k, (state, target) in enumerate(moves):
            step(state, 0.1 + 0.85 * k / len(moves))
            if g.dry_run:
                self.target_pub.publish(target)
                time.sleep(0.3)
                continue
            ok, text = self.move(target, plan_only)
            if not ok:
                gh.abort()
                result.message = f"{state}: {text}"
                return result
            if state == "grasping" and not plan_only:
                pass   # TODO: close the Inspire hand here (the Inspire hand's own SDK/topic)
            if state == "placing" and not plan_only:
                pass   # TODO: open the hand

        step("done", 1.0)
        gh.succeed()
        result.success = True
        result.message = ("dry run: poses computed and published on /karis/task/target_pose"
                          if g.dry_run else "planned only (execute:=false)" if plan_only
                          else "done")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
