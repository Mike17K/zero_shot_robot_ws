"""Higher-level moves onto graph nodes: go to a joint configuration, or to
the graph node closest to the current state."""
from typing import List, Optional

from ..robots.interface import RobotPose
from ..robots.utils import current_robot_pose, determine_start_node, find_and_move_to_node_logic
from ..robots.wrapper import RobotInterface
from ..shared.constants import AVAILABLE_GROUPS, CANONICAL_EE_GROUP
from ..shared.graph import Graph
from ..shared.logger import Logger
from ..shared.types import TrajectoryMovementOptions


def go_to_pose(
    robot: RobotInterface,
    graph: Graph[RobotPose],
    target_joints: List[float],
    direct: bool = False,
    options: Optional[TrajectoryMovementOptions] = None,
) -> bool:
    """Move to target_joints. Unless direct, first snap onto the closest
    graph node so the final move starts from a known state."""
    if not direct and not go_to_closest_pose(robot, graph, options):
        Logger.WARN("Could not reach the closest graph node first - moving directly.")
    return robot.execute_joint_goal(CANONICAL_EE_GROUP, target_joints, options=options)


def go_to_closest_pose(
    robot: RobotInterface,
    graph: Graph[RobotPose],
    options: Optional[TrajectoryMovementOptions] = None,
) -> bool:
    """Move to the graph node the planner would start a path from."""
    current = current_robot_pose(robot, AVAILABLE_GROUPS)
    if current is None:
        return False
    node = determine_start_node(robot, current, graph, 0.5)
    if node is None:
        Logger.WARN("No suitable node in the graph.")
        return False
    return robot.execute_joint_goal(CANONICAL_EE_GROUP, node.canonical.joints, options=options)


__all__ = ["go_to_pose", "go_to_closest_pose", "find_and_move_to_node_logic"]
