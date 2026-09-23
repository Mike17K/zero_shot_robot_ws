"""Client for the cuMotion planner in isaac_ros_cumotion (the C++
CumotionPlanner node, Isaac ROS 4.x): MotionPlan and IKSolution actions under
/<namespace>/cumotion/.

Server-side facts this client is built around (see isaac_ros_cumotion/src/
cumotion_planner.cpp):
  - Goal poses are for the XRDF tool frame (gripper_tcp for group_a) and must
    be expressed in the robot BASE frame (the XRDF's set_base_frame,
    gp_base_link) - no TF is applied on the server. This client transforms
    PoseStamped goals from any frame with TfHelper.
  - goal_state / start_state are read POSITIONALLY in the XRDF cspace joint
    order - names are ignored. This client reorders by name.
  - use_current_state=True makes the server read its own joint_states topic.
  - The server only plans; execute with execution.TrajectoryExecutor.
  - One goal at a time: a goal sent while another is planning is rejected.
  - Grasp path-constraint limits: a NEGATIVE value means "off" (translation
    path) or "server default" (terminal limits). 0.0 means a zero tolerance,
    so every limit here defaults to -1.
"""
from dataclasses import dataclass, field
from typing import Optional, Sequence

from geometry_msgs.msg import PoseStamped
from isaac_ros_cumotion_interfaces.action import IKSolution, MotionPlan
from moveit_msgs.msg import MoveItErrorCodes, PlanningSceneWorld
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from ..geometry import TfHelper, make_pose
from ..joint_state import reorder
from ..ros_helpers import send_action_goal

GP70L_JOINTS = ('gp_joint_1', 'gp_joint_2', 'gp_joint_3', 'gp_joint_4', 'gp_joint_5', 'gp_joint_6')

_ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
                if k.isupper() and isinstance(v, int)}


@dataclass
class PlanResult:
    success: bool
    message: str = ''
    error_code: int = MoveItErrorCodes.FAILURE
    # One trajectory for joint/pose goals; for a grasp: approach (start ->
    # pre-grasp), then grasp (pre-grasp -> grasp) and retract, as requested.
    trajectories: list[JointTrajectory] = field(default_factory=list)
    goal_index: int = -1  # which goal of a goal set was reached
    planning_time: float = 0.0

    @property
    def error_name(self) -> str:
        return _ERROR_NAMES.get(self.error_code, str(self.error_code))

    @property
    def trajectory(self) -> Optional[JointTrajectory]:
        return self.trajectories[0] if self.trajectories else None


@dataclass(frozen=True)
class PathConstraint:
    """Constraints on a grasp approach or retract segment (meters / radians).
    Negative = disabled (path limits) or server default (terminal limits)."""
    translation_path_deviation: float = -1.0      # >= 0: stay on the straight line
    translation_terminal_deviation: float = -1.0
    constant_orientation: bool = False            # keep the orientation along the path
    orientation_path_axis_deviation: float = -1.0
    orientation_terminal_deviation: float = -1.0


class CumotionClient:
    def __init__(self, node: Node, namespace: str = 'robot_1',
                 joint_names: Sequence[str] = GP70L_JOINTS,
                 base_frame: str = 'gp_base_link',
                 tf: Optional[TfHelper] = None,
                 time_dilation_factor: float = 0.5,
                 planning_timeout_sec: float = 30.0):
        """joint_names / base_frame must match the XRDF the planner was
        launched with (planning_bringup/config/group_a/group_a.xrdf).
        tf is only needed for pose goals given in a frame other than
        base_frame - e.g. TfHelper(node, '/robot_1/tf')."""
        self._node = node
        prefix = f'/{namespace.strip("/")}/' if namespace.strip('/') else ''
        self._plan_client = ActionClient(node, MotionPlan, f'{prefix}cumotion/motion_plan')
        self._ik_client = ActionClient(node, IKSolution, f'{prefix}cumotion/ik')
        self.joint_names = tuple(joint_names)
        self.base_frame = base_frame
        self.tf = tf
        self.time_dilation_factor = time_dilation_factor
        self.planning_timeout_sec = planning_timeout_sec

    def is_ready(self, timeout_sec: float = 0.0) -> bool:
        return self._plan_client.wait_for_server(timeout_sec=timeout_sec)

    # ── Planning ────────────────────────────────────────────────────────────

    def plan_to_joints(self, positions: Sequence[float],
                       joint_names: Optional[Sequence[str]] = None,
                       start_positions: Optional[Sequence[float]] = None,
                       world: Optional[PlanningSceneWorld] = None,
                       time_dilation_factor: Optional[float] = None,
                       update_esdf: bool = True) -> PlanResult:
        """Plan to a joint configuration. positions are in joint_names order
        (default: this client's cspace order); start_positions are always in
        cspace order (None = the robot's current state)."""
        goal = self._base_goal(start_positions, world, time_dilation_factor, update_esdf)
        goal.plan_cspace = True
        try:
            ordered = (reorder(joint_names, positions, self.joint_names)
                       if joint_names is not None else list(positions))
        except KeyError as exc:
            return PlanResult(False, f'goal is missing joint {exc}',
                              MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS)
        if len(ordered) != len(self.joint_names):
            return PlanResult(False, f'expected {len(self.joint_names)} joint values, got {len(ordered)}',
                              MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS)
        goal.goal_state.name = list(self.joint_names)
        goal.goal_state.position = [float(v) for v in ordered]
        return self._send_plan(goal)

    def plan_to_pose(self, poses: PoseStamped | Sequence[PoseStamped],
                     start_positions: Optional[Sequence[float]] = None,
                     world: Optional[PlanningSceneWorld] = None,
                     time_dilation_factor: Optional[float] = None,
                     update_esdf: bool = True) -> PlanResult:
        """Plan the tool frame to a pose, or to whichever pose of a goal set
        is best (PlanResult.goal_index says which)."""
        goal = self._base_goal(start_positions, world, time_dilation_factor, update_esdf)
        goal.plan_pose = True
        if not self._fill_goal_poses(goal, poses):
            return PlanResult(False, f'could not express goal pose(s) in {self.base_frame}',
                              MoveItErrorCodes.FRAME_TRANSFORM_FAILURE)
        return self._send_plan(goal)

    def plan_grasp(self, grasp_poses: PoseStamped | Sequence[PoseStamped],
                   approach_offset: Sequence[float] = (0.0, 0.0, -0.15),
                   retract_offset: Sequence[float] = (0.0, 0.0, -0.15),
                   offsets_in_goal_frame: bool = True,
                   approach: PathConstraint = PathConstraint(),
                   retract: PathConstraint = PathConstraint(),
                   plan_approach_to_grasp: bool = True,
                   plan_grasp_to_retract: bool = True,
                   start_positions: Optional[Sequence[float]] = None,
                   world: Optional[PlanningSceneWorld] = None,
                   time_dilation_factor: Optional[float] = None) -> PlanResult:
        """start -> pre-grasp (grasp pose + approach_offset), then optionally
        pre-grasp -> grasp and grasp -> retract (grasp pose + retract_offset).
        Offsets are xyz in meters, in the grasp pose's own frame by default -
        for group_a's gripper_tcp +Z points out of the suction cups, so a
        negative Z backs the tool away from the object. Switch suction on
        between the grasp and retract trajectories."""
        goal = self._base_goal(start_positions, world, time_dilation_factor, update_esdf=True)
        goal.plan_grasp = True
        goal.plan_approach_to_grasp = plan_approach_to_grasp
        goal.plan_grasp_to_retract = plan_grasp_to_retract
        if not self._fill_goal_poses(goal, grasp_poses):
            return PlanResult(False, f'could not express grasp pose(s) in {self.base_frame}',
                              MoveItErrorCodes.FRAME_TRANSFORM_FAILURE)
        goal.grasp_offset_pose = make_pose(approach_offset)
        goal.retract_offset_pose = make_pose(retract_offset)
        goal.grasp_approach_constraint_in_goal_frame = offsets_in_goal_frame
        goal.retract_constraint_in_goal_frame = offsets_in_goal_frame
        self._fill_constraint(goal, 'grasp', approach)
        self._fill_constraint(goal, 'retract', retract)
        return self._send_plan(goal)

    # ── IK ──────────────────────────────────────────────────────────────────

    def solve_ik(self, pose: PoseStamped, num_solutions: int = 1,
                 seed_positions: Optional[Sequence[float]] = None,
                 timeout_sec: float = 10.0) -> list[list[float]]:
        """Collision-free IK solutions for the tool frame, each in cspace order
        (empty if unreachable)."""
        base_pose = self._to_base_frame(pose)
        if base_pose is None:
            return []
        goal = IKSolution.Goal()
        goal.goal_pose = base_pose.pose
        goal.num_solutions_to_return = int(num_solutions)
        if seed_positions is not None:
            goal.seed_state.name = list(self.joint_names)
            goal.seed_state.position = [float(v) for v in seed_positions]
        response, _ = send_action_goal(self._node, self._ik_client, goal, timeout_sec=timeout_sec)
        if response is None:
            return []
        result = response.result
        return [list(js.position) for ok, js in zip(result.success, result.joint_states) if ok]

    # ── Internals ───────────────────────────────────────────────────────────

    def _base_goal(self, start_positions, world, time_dilation_factor, update_esdf) -> MotionPlan.Goal:
        goal = MotionPlan.Goal()
        goal.time_dilation_factor = float(time_dilation_factor if time_dilation_factor is not None
                                          else self.time_dilation_factor)
        if start_positions is None:
            goal.use_current_state = True
        else:
            goal.use_current_state = False
            goal.start_state.name = list(self.joint_names)
            goal.start_state.position = [float(v) for v in start_positions]
        # Always true: the server then rebuilds its world from the static
        # scene + `world` (empty = static scene only) instead of keeping
        # whatever the previous request left behind.
        goal.use_planning_scene = True
        if world is not None:
            goal.world = world
        goal.update_esdf = update_esdf
        for prefix in ('grasp', 'retract'):
            self._fill_constraint(goal, prefix, PathConstraint())
        return goal

    @staticmethod
    def _fill_constraint(goal: MotionPlan.Goal, prefix: str, c: PathConstraint) -> None:
        setattr(goal, f'{prefix}_translation_path_deviation_limit', float(c.translation_path_deviation))
        setattr(goal, f'{prefix}_translation_terminal_deviation_limit', float(c.translation_terminal_deviation))
        setattr(goal, f'{prefix}_enable_orientation_path_axis_constraint', bool(c.constant_orientation))
        setattr(goal, f'{prefix}_orientation_path_axis_deviation_limit', float(c.orientation_path_axis_deviation))
        setattr(goal, f'{prefix}_orientation_terminal_deviation_limit', float(c.orientation_terminal_deviation))

    def _to_base_frame(self, pose: PoseStamped) -> Optional[PoseStamped]:
        if pose.header.frame_id in ('', self.base_frame):
            return pose
        if self.tf is None:
            self._node.get_logger().error(
                f'pose is in {pose.header.frame_id!r}, planner needs {self.base_frame!r} '
                f'- pass a TfHelper to CumotionClient')
            return None
        return self.tf.transform_pose(pose, self.base_frame)

    def _fill_goal_poses(self, goal: MotionPlan.Goal, poses) -> bool:
        poses = [poses] if isinstance(poses, PoseStamped) else list(poses)
        if not poses:
            return False
        goal.goal_pose.header.frame_id = self.base_frame
        for p in poses:
            base = self._to_base_frame(p)
            if base is None:
                return False
            goal.goal_pose.poses.append(base.pose)
        return True

    def _send_plan(self, goal: MotionPlan.Goal) -> PlanResult:
        response, handle = send_action_goal(self._node, self._plan_client, goal,
                                            timeout_sec=self.planning_timeout_sec)
        if response is None:
            reason = ('planner busy or unavailable' if handle is None or not handle.accepted
                      else 'planning timed out')
            return PlanResult(False, reason, MoveItErrorCodes.FAILURE)
        r = response.result
        result = PlanResult(
            success=bool(r.success),
            message=r.message,
            error_code=r.error_code.val,
            trajectories=[t.joint_trajectory for t in r.planned_trajectory],
            goal_index=r.goal_index,
            planning_time=r.planning_time,
        )
        if not result.success:
            self._node.get_logger().warn(
                f'cuMotion planning failed: {result.error_name} {result.message}')
        return result

