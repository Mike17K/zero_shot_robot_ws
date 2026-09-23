"""Straight-line tool moves via MoveIt's compute_cartesian_path
(/<namespace>/compute_cartesian_path, served by the robot's move_group).

cuMotion plans point-to-point; for "10 cm straight down" the path itself
matters, which is exactly what compute_cartesian_path does: IK along
interpolated waypoints, collision-checked, time-parameterized.
"""
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetCartesianPath
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from ..geometry import TfHelper, matrix_to_pose, pose_to_matrix
from ..ros_helpers import call_service


@dataclass
class CartesianResult:
    success: bool
    message: str = ''
    fraction: float = 0.0  # part of the path that could be followed
    trajectory: Optional[JointTrajectory] = None
    waypoints: list = field(default_factory=list)


class CartesianPlanner:
    def __init__(self, node: Node, tf: TfHelper, namespace: str = 'robot_1',
                 group: str = 'manipulator', tool_frame: str = 'gripper_tcp',
                 world_frame: str = 'world'):
        self._node = node
        self.tf = tf
        ns = namespace.strip('/')
        self._client = node.create_client(GetCartesianPath, f'/{ns}/compute_cartesian_path' if ns
                                          else 'compute_cartesian_path')
        self.group = group
        self.tool_frame = tool_frame
        self.world_frame = world_frame

    def tool_pose(self) -> Optional[PoseStamped]:
        return self.tf.frame_pose(self.world_frame, self.tool_frame)

    def plan_offset(self, offset: Sequence[float], frame: str = 'world',
                    max_speed: float = 0.05, max_step: float = 0.005,
                    min_fraction: float = 0.99, avoid_collisions: bool = True) -> CartesianResult:
        """Plan the tool from where it is now along a straight line by
        offset (meters) - in the world frame, or with frame='tool' in the
        tool's own axes. Succeeds only if at least min_fraction of the line
        can be followed."""
        start = self.tool_pose()
        if start is None:
            return CartesianResult(False, f'no TF {self.world_frame} -> {self.tool_frame}')
        T = pose_to_matrix(start.pose)
        d = np.asarray(offset, dtype=float)
        if frame == 'tool':
            d = T[:3, :3] @ d
        elif frame != 'world':
            return CartesianResult(False, f"frame must be 'world' or 'tool', got {frame!r}")
        goal = T.copy()
        goal[:3, 3] += d
        return self.plan_waypoints([matrix_to_pose(goal)], max_speed, max_step, min_fraction,
                                   avoid_collisions)

    def plan_waypoints(self, waypoints: Sequence[Pose], max_speed: float = 0.05,
                       max_step: float = 0.005, min_fraction: float = 0.99,
                       avoid_collisions: bool = True) -> CartesianResult:
        """Plan through world-frame tool waypoints, starting at the current state."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.world_frame
        req.group_name = self.group
        req.link_name = self.tool_frame
        req.waypoints = list(waypoints)
        req.max_step = float(max_step)
        req.jump_threshold = 0.0  # disabled; revolute threshold below instead
        req.revolute_jump_threshold = 0.5
        req.avoid_collisions = avoid_collisions
        req.start_state.is_diff = True  # = move_group's current state
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.cartesian_speed_limited_link = self.tool_frame
        req.max_cartesian_speed = float(max_speed)
        response = call_service(self._node, self._client, req, timeout_sec=10.0)
        if response is None:
            return CartesianResult(False, f'{self._client.srv_name} unavailable (is move_group running?)')
        traj = response.solution.joint_trajectory
        if response.error_code.val != 1 or not traj.points:
            return CartesianResult(False, f'cartesian planning failed (error {response.error_code.val})',
                                   response.fraction)
        if response.fraction < min_fraction:
            return CartesianResult(False, f'only {response.fraction * 100:.0f}% of the line is feasible',
                                   response.fraction)
        return CartesianResult(True, 'ok', response.fraction, traj, list(waypoints))
