from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

import numpy as np
from ..shared.constants import CANONICAL_EE_GROUP
from ..shared.model import FrozenModel
from ..shared.types import Point, Quaternion, TrajectoryMovementOptions


class MoveGroupState(FrozenModel):
    """State of a move group: tool position/rotation in the world frame and
    the group's joint values (in the group's joint order)."""
    pos: Point
    rot: Quaternion
    joints: Tuple[float, ...]


class RobotPose(FrozenModel):
    """Graph node: one MoveGroupState per group (for the GP70L only
    CANONICAL_EE_GROUP)."""
    id: Optional[int] = None
    all_ee_poses: Dict[str, MoveGroupState]
    is_free_to_move_from: Optional[bool] = False  # planner may start a path from here
    is_joint_goal: Optional[bool] = False  # execute as a joint goal (vs. a pose goal)
    label: Optional[str] = None

    def __hash__(self):
        return hash((self.id, tuple(sorted(self.all_ee_poses.items())),
                     self.is_free_to_move_from, self.is_joint_goal, self.label))

    def __eq__(self, other):
        # Two poses are "the same node" when their tool positions are within 2 cm.
        if not isinstance(other, RobotPose):
            return NotImplemented
        return self.canonical.pos.distance_to(other.canonical.pos) < 0.02

    @property
    def canonical(self) -> MoveGroupState:
        return self.all_ee_poses[CANONICAL_EE_GROUP]

    def copy_with(self, **kwargs) -> 'RobotPose':
        data = self.model_dump(exclude_none=True)
        data.update(kwargs)
        return RobotPose(**data)

    @classmethod
    def model_validate_json(cls, data: str) -> 'RobotPose':
        import json
        raw = json.loads(data)
        raw['all_ee_poses'] = {
            k: v if isinstance(v, MoveGroupState) else MoveGroupState(**v)
            for k, v in raw.get('all_ee_poses', {}).items()
        }
        return cls(**raw)

    def get_similarity(self, other: 'RobotPose') -> float:
        """RMS joint difference over the groups both poses have (rad)."""
        total, count = 0.0, 0
        for group, state in self.all_ee_poses.items():
            if group in other.all_ee_poses:
                j1, j2 = state.joints, other.all_ee_poses[group].joints
                if len(j1) != len(j2):
                    raise ValueError(f'joint length mismatch for group {group}')
                total += sum((a - b) ** 2 for a, b in zip(j1, j2))
                count += len(j1)
        return float('inf') if count == 0 else float(np.sqrt(total / count))

    @classmethod
    def from_joints(cls, robot: 'BaseRobotManager', joints: List[float],
                    is_joint_goal: bool = True) -> Tuple[Optional['RobotPose'], bool]:
        """Pose for a joint configuration of the canonical group (tool pose via FK)."""
        ok, pos, rot = robot.get_pose_from_fk(CANONICAL_EE_GROUP, joints)
        if not ok:
            return None, False
        state = MoveGroupState(pos=pos, rot=rot, joints=tuple(joints))
        return cls(all_ee_poses={CANONICAL_EE_GROUP: state}, is_joint_goal=is_joint_goal), True

    @classmethod
    def from_posrot(cls, robot: 'BaseRobotManager', pos: Point, rot: Quaternion,
                    is_joint_goal: bool = False) -> Tuple[Optional['RobotPose'], bool]:
        """Pose for a tool pose (joints via IK)."""
        joints, ok = robot.get_pose_from_ik(CANONICAL_EE_GROUP, pos, rot)
        if not ok:
            return None, False
        state = MoveGroupState(pos=pos, rot=rot, joints=tuple(joints))
        return cls(all_ee_poses={CANONICAL_EE_GROUP: state}, is_joint_goal=is_joint_goal), True


class BaseRobotManager(ABC):
    """Robot client API. Positions/rotations are the tool frame in the world
    frame; joints are in the group's joint order."""

    @abstractmethod
    def get_ee_pose_for_group(self, group_name: str) -> Optional[MoveGroupState]:
        """Current tool pose + joints, or None if not available yet."""

    @abstractmethod
    def execute_joint_goal(self, group_name: str, goal: List[float],
                           options: Optional[TrajectoryMovementOptions] = None) -> bool:
        """Plan to the joint goal and execute it. Blocking."""

    @abstractmethod
    def execute_pose_goal(self, group_name: str, goal: RobotPose,
                          options: Optional[TrajectoryMovementOptions] = None) -> bool:
        """Plan the tool to goal's pose for group_name and execute it. Blocking."""

    @abstractmethod
    def get_pose_from_fk(self, group_name: str, joint_positions: List[float]
                         ) -> Tuple[bool, Optional[Point], Optional[Quaternion]]:
        """(ok, tool position, tool rotation) for the joint positions."""

    @abstractmethod
    def get_pose_from_ik(self, group_name: str, pos: Point, rot: Quaternion
                         ) -> Tuple[List[float], bool]:
        """(joint positions, ok) reaching the tool pose, collision-free."""


__all__ = ['BaseRobotManager', 'MoveGroupState', 'RobotPose']
