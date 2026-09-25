"""Client for MoveIt's move_group (/<namespace>/move_action, MoveGroup action)
in plan-only mode - the MoveIt counterpart of CumotionClient, returning the
same PlanResult so the two can be swapped.

Facts about the group_a move_group this client is built around (see
group_a_bringup/launch/bringup.launch.py):
  - move_group runs in the robot's namespace and reads /<namespace>/tf, so
    pose goals can be given in any frame it knows - no client-side TF.
  - Its DEFAULT pipeline is isaac_ros_cumotion (the cuMotion MoveIt plugin);
    ompl, chomp, stomp and pilz_industrial_motion_planner are loaded too.
    This client asks for ompl unless told otherwise, e.g.
    pipeline_id='pilz_industrial_motion_planner', planner_id='LIN' for a
    straight-line move or 'PTP' for a joint-interpolated one.
  - Joint goals are matched by name, so any joint order works.
  - `world` objects go into planning_options.planning_scene_diff, which is
    used for this request only and not stored in move_group's scene.
  - The server only plans here (plan_only=True); execute with
    execution.TrajectoryExecutor.
"""
from typing import Optional, Sequence

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
                             PlanningSceneWorld, PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from ..ros_helpers import send_action_goal
from .result import GP70L_JOINTS, PlanResult


class MoveItClient:
    def __init__(self, node: Node, namespace: str = 'robot_1',
                 group: str = 'manipulator',
                 joint_names: Sequence[str] = GP70L_JOINTS,
                 tool_frame: str = 'gripper_tcp',
                 planning_frame: str = 'world',
                 pipeline_id: str = 'ompl',
                 planner_id: str = '',
                 velocity_scaling: float = 0.5,
                 acceleration_scaling: float = 0.5,
                 allowed_planning_time: float = 5.0,
                 num_planning_attempts: int = 1,
                 planning_timeout_sec: float = 30.0):
        """group / tool_frame / planning_frame must match the SRDF
        (group_a_description/config/combined_system.srdf.xacro). planner_id ''
        = the pipeline's default planner for the group (RRTConnect for ompl)."""
        self._node = node
        prefix = f'/{namespace.strip("/")}/' if namespace.strip('/') else ''
        self._client = ActionClient(node, MoveGroup, f'{prefix}move_action')
        self.group = group
        self.joint_names = tuple(joint_names)
        self.tool_frame = tool_frame
        self.planning_frame = planning_frame
        self.pipeline_id = pipeline_id
        self.planner_id = planner_id
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.allowed_planning_time = allowed_planning_time
        self.num_planning_attempts = num_planning_attempts
        self.planning_timeout_sec = planning_timeout_sec

    def is_ready(self, timeout_sec: float = 0.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    # ── Planning ────────────────────────────────────────────────────────────

    def plan_to_joints(self, positions: Sequence[float],
                       joint_names: Optional[Sequence[str]] = None,
                       start_positions: Optional[Sequence[float]] = None,
                       world: Optional[PlanningSceneWorld] = None,
                       tolerance: float = 1e-3,
                       **overrides) -> PlanResult:
        """Plan to a joint configuration. positions are in joint_names order
        (default: this client's joint order); start_positions are always in
        this client's order (None = the robot's current state). overrides:
        pipeline_id, planner_id, velocity_scaling, acceleration_scaling,
        allowed_planning_time, num_planning_attempts."""
        names = tuple(joint_names) if joint_names is not None else self.joint_names
        if len(positions) != len(names):
            return PlanResult(False, f'expected {len(names)} joint values, got {len(positions)}',
                              MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS)
        constraints = Constraints()
        for name, value in zip(names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = jc.tolerance_below = float(tolerance)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return self._plan([constraints], start_positions, world, overrides)

    def plan_to_pose(self, poses: PoseStamped | Sequence[PoseStamped],
                     start_positions: Optional[Sequence[float]] = None,
                     world: Optional[PlanningSceneWorld] = None,
                     position_tolerance: float = 1e-3,
                     orientation_tolerance: float = 1e-2,
                     **overrides) -> PlanResult:
        """Plan the tool frame to a pose, or to any pose of a goal set (MoveIt
        does not report which one, so goal_index stays -1). An empty
        frame_id means planning_frame. overrides: see plan_to_joints."""
        poses = [poses] if isinstance(poses, PoseStamped) else list(poses)
        if not poses:
            return PlanResult(False, 'no goal pose', MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS)
        goals = [self._pose_constraints(p, position_tolerance, orientation_tolerance) for p in poses]
        return self._plan(goals, start_positions, world, overrides)

    # ── Internals ───────────────────────────────────────────────────────────

    def _pose_constraints(self, pose: PoseStamped, position_tolerance: float,
                          orientation_tolerance: float) -> Constraints:
        frame = pose.header.frame_id or self.planning_frame
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = self.tool_frame
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(position_tolerance)])
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = self.tool_frame
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = \
            oc.absolute_z_axis_tolerance = float(orientation_tolerance)
        oc.weight = 1.0
        return Constraints(position_constraints=[pc], orientation_constraints=[oc])

    def _plan(self, goal_constraints: list[Constraints], start_positions,
              world: Optional[PlanningSceneWorld], overrides: dict) -> PlanResult:
        unknown = set(overrides) - {'pipeline_id', 'planner_id', 'velocity_scaling',
                                    'acceleration_scaling', 'allowed_planning_time',
                                    'num_planning_attempts'}
        if unknown:
            raise TypeError(f'unexpected planning options: {sorted(unknown)}')
        opt = {k: overrides.get(k, getattr(self, k)) for k in
               ('pipeline_id', 'planner_id', 'velocity_scaling', 'acceleration_scaling',
                'allowed_planning_time', 'num_planning_attempts')}

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.pipeline_id = opt['pipeline_id']
        req.planner_id = opt['planner_id']
        req.goal_constraints = goal_constraints
        req.num_planning_attempts = int(opt['num_planning_attempts'])
        req.allowed_planning_time = float(opt['allowed_planning_time'])
        req.max_velocity_scaling_factor = float(opt['velocity_scaling'])
        req.max_acceleration_scaling_factor = float(opt['acceleration_scaling'])
        if start_positions is None:
            req.start_state.is_diff = True  # = move_group's current state
        else:
            if len(start_positions) != len(self.joint_names):
                return PlanResult(False, f'expected {len(self.joint_names)} start values, '
                                         f'got {len(start_positions)}',
                                  MoveItErrorCodes.INVALID_ROBOT_STATE)
            req.start_state.joint_state.name = list(self.joint_names)
            req.start_state.joint_state.position = [float(v) for v in start_positions]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        if world is not None:
            goal.planning_options.planning_scene_diff.world = world
        return self._send(goal)

    def _send(self, goal: MoveGroup.Goal) -> PlanResult:
        response, handle = send_action_goal(self._node, self._client, goal,
                                            timeout_sec=self.planning_timeout_sec)
        if response is None:
            reason = ('move_group unavailable or goal rejected' if handle is None or not handle.accepted
                      else 'planning timed out')
            return PlanResult(False, reason, MoveItErrorCodes.FAILURE)
        r = response.result
        traj = r.planned_trajectory.joint_trajectory
        success = r.error_code.val == MoveItErrorCodes.SUCCESS and bool(traj.points)
        result = PlanResult(
            success=success,
            message=r.error_code.message,
            error_code=r.error_code.val,
            trajectories=[traj] if traj.points else [],
            planning_time=r.planning_time,
        )
        if not result.success:
            self._node.get_logger().warn(
                f'MoveIt planning ({goal.request.pipeline_id}) failed: {result.error_name} {result.message}')
        return result
