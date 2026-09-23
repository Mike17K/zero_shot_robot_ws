from typing import List, Optional

from ..shared.constants import CANONICAL_EE_GROUP
from ..shared.graph import Graph
from ..shared.logger import Logger
from ..shared.types import Point, Quaternion
from .interface import RobotPose
from .wrapper import RobotInterface, TrajectoryMovementOptions

AT_NODE_TOLERANCE = 0.01  # rad, per joint: "already at this node"


def current_robot_pose(robot: RobotInterface, selected_groups: List[str]) -> Optional[RobotPose]:
    poses = robot.collect_all_ee_poses(selected_groups)
    if CANONICAL_EE_GROUP not in poses:
        Logger.ERROR(f"Could not read the current '{CANONICAL_EE_GROUP}' state (joint states / TF).")
        return None
    return RobotPose(all_ee_poses=poses)


def determine_start_node(
    robot: RobotInterface,
    current_robot_state_pose: RobotPose,
    graph: Graph,
    threshold: float = None,
    current_all_joints: List[float] = None,
) -> Optional[RobotPose]:
    """The node to start a path from: the closest connected node if it is
    within `threshold` (RMS joint difference, rad), otherwise the closest node
    marked is_free_to_move_from (by tool distance)."""
    threshold = 1.0 if threshold is None else threshold
    if current_all_joints is None:
        current_all_joints = current_robot_state_pose.canonical.joints
    nearby = graph.find_nearby_nodes(current_robot_state_pose, current_all_joints=current_all_joints)
    if nearby and nearby[0][1] <= threshold:
        return nearby[0][0]

    free_nodes = [n for n in graph._pose_to_id.keys() if n.is_free_to_move_from]
    if not free_nodes:
        Logger.ERROR("Robot is not near any graph node and no node is marked 'free to move from'.")
        return None
    here = current_robot_state_pose.canonical.pos
    return min(free_nodes, key=lambda n: n.canonical.pos.distance_to(here))


def _is_at(node: RobotPose, joints) -> bool:
    return all(abs(a - b) < AT_NODE_TOLERANCE for a, b in zip(node.canonical.joints, joints))


def _describe_path(graph: Graph, path) -> str:
    out = ""
    for i, (pose, move_group, options) in enumerate(path):
        node_id = graph._get_or_assign_id(pose)
        out += f"[{node_id}]" if i == 0 else f" --('{move_group}')--> [{node_id}]"
        if options:
            out += f"({options})"
    return out


def find_and_move_to_node_logic(
    robot: RobotInterface,
    node_id: int,
    graph: Graph,
    selected_groups: List[str],
    interactive: bool = False,
) -> bool:
    """Move from the current state onto the graph (if not on a node already)
    and along the shortest path to node_id, one joint goal per edge."""
    if not graph.adjacency:
        Logger.WARN("Graph is empty. Cannot execute path.")
        return False
    target_node = graph._id_to_pose.get(node_id)
    if not target_node:
        Logger.ERROR(f"Node with ID {node_id} not found in graph.")
        return False

    current = current_robot_pose(robot, selected_groups)
    if current is None:
        return False
    current_joints = current.canonical.joints
    start_node = determine_start_node(robot, current, graph, 0.3, current_joints)
    if not start_node:
        Logger.ERROR("Failed to determine a starting node for path execution. Aborting.")
        return False

    path = graph.shortest_path(start_node, target_node)
    if not path:
        Logger.WARN(f"No path between node ID {graph._get_or_assign_id(start_node)} and node ID {node_id}.")
        return False

    at_start = _is_at(start_node, current_joints)
    Logger.INFO("--- Execution Plan ---")
    if not at_start:
        Logger.INFO(f"1. Move from the current state to start node ID {graph._get_or_assign_id(start_node)}.")
    Logger.INFO(f"{'2' if not at_start else '1'}. Path: {_describe_path(graph, path)}")

    if interactive and input("Proceed with execution? (y/N): ").strip().lower() != 'y':
        Logger.INFO("Execution cancelled by user.")
        return False

    if not at_start and not robot.execute_joint_goal(CANONICAL_EE_GROUP, start_node.canonical.joints):
        Logger.ERROR(f"Failed to reach start node ID {graph._get_or_assign_id(start_node)}.")
        return False

    for i, (pose, move_group, options) in enumerate(path[1:], start=1):
        group = move_group or CANONICAL_EE_GROUP
        Logger.INFO(f"Segment {i}/{len(path) - 1} -> node ID {graph._get_or_assign_id(pose)} via '{group}'")
        ok = (robot.execute_joint_goal(group, pose.all_ee_poses[group].joints, options=options)
              if pose.is_joint_goal else robot.execute_pose_goal(group, pose, options=options))
        if not ok:
            Logger.ERROR(f"Failed at node ID {graph._get_or_assign_id(pose)}. Stopping path execution.")
            return False
    Logger.INFO("Path execution complete.")
    return True


def find_and_move_to_target_logic(
    robot: RobotInterface,
    target_point: Point,
    target_orientation: Quaternion,
    graph: Graph,
    selected_groups: List[str],
    end_node_options: TrajectoryMovementOptions = None,
) -> bool:
    """Plan straight to the tool pose; if that fails, go through the graph to
    the node closest to the target and plan the last step from there."""
    current = current_robot_pose(robot, selected_groups)
    if current is None:
        return False
    state = current.canonical.model_copy(update={'pos': target_point, 'rot': target_orientation})
    target = RobotPose(all_ee_poses={CANONICAL_EE_GROUP: state})
    if robot.execute_pose_goal(CANONICAL_EE_GROUP, target, options=end_node_options):
        return True

    candidates = [n for n in graph._id_to_pose.values() if graph.adjacency.get(n.id)]
    if not candidates:
        Logger.WARN("Direct plan failed and the graph has no connected nodes to route through.")
        return False
    closest = min(candidates, key=lambda n: n.canonical.pos.distance_to(target_point))
    Logger.INFO(f"Direct plan failed - routing via closest node ID {closest.id}.")
    if not find_and_move_to_node_logic(robot, closest.id, graph, selected_groups):
        return False
    return robot.execute_pose_goal(CANONICAL_EE_GROUP, target, options=end_node_options)
