from typing import Dict, List, Optional

from rclpy.node import Node

from ..shared.constants import AVAILABLE_GROUPS
from ..shared.logger import Logger
from ..shared.types import TrajectoryMovementOptions
from .interface import BaseRobotManager, MoveGroupState, RobotPose


class RobotInterface:
    """Thin static facade over the single active BaseRobotManager."""
    manager: BaseRobotManager = None

    @staticmethod
    def SetMode(node: Node, isReal: bool = True):
        """Sim and real use the same ROS interfaces for the GP70L (Gazebo runs
        the same controllers), so both select Gp70lRobotManager."""
        from .ROS2.gp70l import Gp70lRobotManager
        p = node.get_parameter
        RobotInterface.manager = Gp70lRobotManager(
            node,
            namespace=p('namespace').value,
            world_frame=p('world_frame').value,
            tool_frame=p('tool_frame').value,
            controller=p('controller').value,
        )
        Logger.INFO(f"Robot manager ready ({'real' if isReal else 'simulation'} mode).")

    @staticmethod
    def get_ee_pose_for_group(group_name: str) -> Optional[MoveGroupState]:
        assert group_name in AVAILABLE_GROUPS, f"Group {group_name} not in {AVAILABLE_GROUPS}"
        return RobotInterface.manager.get_ee_pose_for_group(group_name)

    @staticmethod
    def collect_all_ee_poses(group_names: List[str]) -> Dict[str, MoveGroupState]:
        poses = {}
        for group_name in group_names:
            state = RobotInterface.get_ee_pose_for_group(group_name)
            if state:
                poses[group_name] = state
        return poses

    @staticmethod
    def execute_joint_goal(group_name: str, goal: List[float],
                           options: TrajectoryMovementOptions = None) -> bool:
        assert group_name in AVAILABLE_GROUPS, f"Group {group_name} not in {AVAILABLE_GROUPS}"
        Logger.INFO(f"Joint goal for '{group_name}': " + ", ".join(f"{g:+.3f}" for g in goal))
        return RobotInterface.manager.execute_joint_goal(group_name, list(goal), options=options)

    @staticmethod
    def execute_pose_goal(group_name: str, goal: RobotPose,
                          options: TrajectoryMovementOptions = None) -> bool:
        assert group_name in AVAILABLE_GROUPS, f"Group {group_name} not in {AVAILABLE_GROUPS}"
        return RobotInterface.manager.execute_pose_goal(group_name, goal, options=options)


__all__ = ['RobotInterface']
