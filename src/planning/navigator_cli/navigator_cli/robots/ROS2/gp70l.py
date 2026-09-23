"""BaseRobotManager for group_a (Yaskawa GP70L + suction gripper) in this
workspace: cuMotion plans, the joint_trajectory_controller executes, TF and
MoveIt's compute_fk give poses. Everything is namespaced per robot."""
from typing import List, Optional, Tuple

from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from std_srvs.srv import SetBool

from shared_utils.execution import TrajectoryExecutor
from shared_utils.geometry import TfHelper, make_pose_stamped
from shared_utils.joint_state import JointStateCache
from shared_utils.planning import GP70L_JOINTS, CartesianPlanner, CartesianResult, CumotionClient
from shared_utils.ros_helpers import call_service

from ...shared.constants import AVAILABLE_GROUPS
from ...shared.logger import Logger
from ...shared.types import Point, Quaternion, TrajectoryMovementOptions
from ..interface import BaseRobotManager, MoveGroupState, RobotPose

DEFAULT_SPEED = 0.5  # cuMotion time_dilation_factor when an edge sets none


class Gp70lRobotManager(BaseRobotManager):
    def __init__(self, node: Node, namespace: str = 'robot_1', world_frame: str = 'world',
                 tool_frame: str = 'gripper_tcp',
                 controller: str = 'gp70l_joint_trajectory_controller'):
        self._node = node
        ns = namespace.strip('/')
        self.world_frame = world_frame
        self.tool_frame = tool_frame
        self.joint_names = list(GP70L_JOINTS)
        self.joint_states = JointStateCache(node, f'/{ns}/joint_states')
        self.tf = TfHelper(node, f'/{ns}/tf')
        self.planner = CumotionClient(node, namespace=ns, joint_names=self.joint_names, tf=self.tf)
        self.cartesian = CartesianPlanner(node, self.tf, namespace=ns, tool_frame=tool_frame,
                                          world_frame=world_frame)
        self.executor = TrajectoryExecutor(
            node, f'/{ns}/{controller}/follow_joint_trajectory', self.joint_states)
        self._fk_client = node.create_client(GetPositionFK, f'/{ns}/compute_fk')
        self._suction_client = node.create_client(SetBool, f'/{ns}/gripper/set_suction')
        Logger.INFO(f'GP70L manager on /{ns}: planner cumotion/motion_plan, '
                    f'controller {controller}, tool {tool_frame} in {world_frame}')

    # ── State ───────────────────────────────────────────────────────────────

    def _check_group(self, group_name: str) -> None:
        if group_name not in AVAILABLE_GROUPS:
            raise ValueError(f'unknown group {group_name!r}, available: {AVAILABLE_GROUPS}')

    def get_ee_pose_for_group(self, group_name: str) -> Optional[MoveGroupState]:
        self._check_group(group_name)
        joints = self.joint_states.positions(self.joint_names)
        if joints is None:
            Logger.WARN('no joint states received yet')
            return None
        pose = self.tf.frame_pose(self.world_frame, self.tool_frame)
        if pose is None:
            return None
        p, q = pose.pose.position, pose.pose.orientation
        return MoveGroupState(pos=Point(p.x, p.y, p.z), rot=Quaternion(q.x, q.y, q.z, q.w),
                              joints=tuple(joints))

    # ── Motion ──────────────────────────────────────────────────────────────

    @staticmethod
    def _speed(options: Optional[TrajectoryMovementOptions]) -> float:
        speed = options.speed if options is not None and options.speed is not None else DEFAULT_SPEED
        if not 0.0 < speed <= 1.0:
            raise ValueError(f'speed must be in (0, 1], got {speed}')
        return speed

    @staticmethod
    def _attempts(options: Optional[TrajectoryMovementOptions]) -> int:
        return options.max_attempts if options is not None and options.max_attempts else 1

    def _plan_and_execute(self, plan_fn, options) -> bool:
        result = None
        for attempt in range(1, self._attempts(options) + 1):
            result = plan_fn(self._speed(options))
            if result.success:
                break
            Logger.WARN(f'planning attempt {attempt} failed: {result.error_name} {result.message}')
        if result is None or not result.success:
            return False
        return self.executor.execute(result.trajectory)

    def execute_joint_goal(self, group_name: str, goal: List[float],
                           options: Optional[TrajectoryMovementOptions] = None) -> bool:
        self._check_group(group_name)
        if len(goal) != len(self.joint_names):
            Logger.ERROR(f'joint goal needs {len(self.joint_names)} values, got {len(goal)}')
            return False
        return self._plan_and_execute(
            lambda speed: self.planner.plan_to_joints(list(goal), self.joint_names,
                                                      time_dilation_factor=speed), options)

    def execute_pose_goal(self, group_name: str, goal: RobotPose,
                          options: Optional[TrajectoryMovementOptions] = None) -> bool:
        self._check_group(group_name)
        state = goal.all_ee_poses[group_name]
        target = make_pose_stamped(self.world_frame, (state.pos.x, state.pos.y, state.pos.z),
                                   quat=(state.rot.x, state.rot.y, state.rot.z, state.rot.w))
        return self._plan_and_execute(
            lambda speed: self.planner.plan_to_pose(target, time_dilation_factor=speed), options)

    def execute_cartesian(self, offset, frame: str = 'world', max_speed: float = 0.05) -> CartesianResult:
        """Straight-line tool move by offset (m), 'world' or 'tool' axes. Blocking."""
        result = self.cartesian.plan_offset(offset, frame=frame, max_speed=max_speed)
        if not result.success:
            Logger.ERROR(f'cartesian move: {result.message}')
            return result
        if not self.executor.execute(result.trajectory):
            result.success, result.message = False, 'execution failed'
        return result

    def cancel_motion(self) -> bool:
        """Stop the trajectory currently executing (from another thread)."""
        return self.executor.cancel()

    def set_suction(self, on: bool) -> bool:
        response = call_service(self._node, self._suction_client, SetBool.Request(data=bool(on)))
        if response is None:
            return False
        Logger.INFO(response.message)
        return bool(response.success)

    # ── Kinematics ──────────────────────────────────────────────────────────

    def get_pose_from_fk(self, group_name: str, joint_positions: List[float]
                         ) -> Tuple[bool, Optional[Point], Optional[Quaternion]]:
        self._check_group(group_name)
        req = GetPositionFK.Request()
        req.header.frame_id = self.world_frame
        req.fk_link_names = [self.tool_frame]
        req.robot_state.joint_state.name = list(self.joint_names)
        req.robot_state.joint_state.position = [float(v) for v in joint_positions]
        response = call_service(self._node, self._fk_client, req)
        if response is None or response.error_code.val != 1 or not response.pose_stamped:
            Logger.ERROR('FK failed' + (f' (error {response.error_code.val})' if response else ''))
            return False, None, None
        p, q = response.pose_stamped[0].pose.position, response.pose_stamped[0].pose.orientation
        return True, Point(p.x, p.y, p.z), Quaternion(q.x, q.y, q.z, q.w)

    def get_pose_from_ik(self, group_name: str, pos: Point, rot: Quaternion
                         ) -> Tuple[List[float], bool]:
        self._check_group(group_name)
        target = make_pose_stamped(self.world_frame, (pos.x, pos.y, pos.z),
                                   quat=(rot.x, rot.y, rot.z, rot.w))
        seed = self.joint_states.positions(self.joint_names)
        solutions = self.planner.solve_ik(target, num_solutions=1, seed_positions=seed)
        if not solutions:
            Logger.ERROR('IK found no collision-free solution')
            return [], False
        return solutions[0], True
