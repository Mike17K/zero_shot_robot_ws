from typing import Any, Dict, List

from rclpy.node import Node

from ..robots.interface import RobotPose
from ..robots.utils import (current_robot_pose, determine_start_node, find_and_move_to_node_logic,
                            find_and_move_to_target_logic)
from ..robots.wrapper import RobotInterface
from ..shared.constants import AVAILABLE_GROUPS, CANONICAL_EE_GROUP
from ..shared.graph import Graph
from ..shared.logger import Logger
from ..shared.types import Point, Quaternion, TrajectoryMovementOptions
from .interface import CLIFunctionBase


AT_NODE_RMS = 0.02  # rad, RMS joint difference to count as "at" a node

FLAGS = {
    'd': 'direct joint move to the node, ignoring the graph',
    'p': 'show which node the robot is at',
    't': 'move the tool to a pose you type in',
}


class GraphNavigationCLIFunction(CLIFunctionBase):
    name = "e"

    def __init__(self, node: Node, robot: RobotInterface, graph: Graph[RobotPose]):
        super().__init__(name=self.name)
        self._node = node
        self.robot = robot
        self.graph = graph
        self.selected_groups = list(AVAILABLE_GROUPS)

    @staticmethod
    def __str__():
        n = GraphNavigationCLIFunction.name
        return (f"Move along the pose graph. Usage: {n} <node_id|label> (shortest path) | "
                f"{n} <node_id|label> -d (direct joint move) | {n} -p (where am I in the graph) | "
                f"{n} -t (move the tool to a pose you type in)")

    def execute(self, args: List[str] = None, kwargs: Dict[str, Any] = None):
        kwargs = kwargs or {}
        unknown = [k for k in kwargs if k not in FLAGS]
        if unknown:
            return self._print_options(f"Unknown option(s): {', '.join('-' + k for k in unknown)}")
        if 'p' in kwargs:
            return self._where_am_i()
        if 't' in kwargs:
            return self._move_to_typed_target()

        if not args:
            return self._print_options("No target node given.")
        target_node = self._resolve_target(args[0])
        if target_node is None:
            return self._print_options(f"No node with ID or label '{args[0]}'.")
        target_id = self.graph._get_or_assign_id(target_node)

        if 'd' in kwargs:
            ok = self.robot.execute_joint_goal(CANONICAL_EE_GROUP, target_node.canonical.joints,
                                               TrajectoryMovementOptions())
            (Logger.INFO if ok else Logger.ERROR)(
                f"Direct joint move to node ID {target_id} {'done' if ok else 'failed'}.")
            return

        find_and_move_to_node_logic(robot=self.robot, node_id=target_id, graph=self.graph,
                                    selected_groups=self.selected_groups, interactive=True)

    def _resolve_target(self, token: str):
        """Node by integer ID, else by label (case-insensitive)."""
        try:
            return self.graph._id_to_pose.get(int(token))
        except ValueError:
            pass
        matches = [n for n in self.graph._id_to_pose.values()
                   if n.label and n.label.lower() == token.lower()]
        return matches[0] if matches else None

    def _print_options(self, reason: str):
        """Explain what went wrong, then list the usage and every target."""
        n = self.name
        Logger.WARN(reason)
        Logger.INFO("Usage:")
        Logger.INFO(f"  {n} <id|label>       go along the shortest graph path (asks y/N first)")
        for flag, text in FLAGS.items():
            usage = f"{n} <id|label> -{flag}" if flag == 'd' else f"{n} -{flag}"
            Logger.INFO(f"  {usage:18s} {text}")

        nodes = sorted(self.graph._id_to_pose.items())
        if not nodes:
            Logger.INFO("The graph is empty - add nodes with 'a' (graph add) first.")
            return
        # "you are here" only when the robot actually sits on a node - quietly
        # skipped if joint states / TF are not available yet.
        here = None
        poses = self.robot.collect_all_ee_poses(self.selected_groups)
        if CANONICAL_EE_GROUP in poses:
            current = RobotPose(all_ee_poses=poses)
            nearby = self.graph.find_nearby_nodes(current, current.canonical.joints)
            if nearby and nearby[0][1] <= AT_NODE_RMS:
                here = nearby[0][0]
        Logger.INFO("Available targets:")
        for node_id, node in nodes:
            p = node.canonical.pos
            edges = len(self.graph.adjacency.get(node_id, ()))
            tags = [t for t, on in (("you are here", here is not None and here.id == node_id),
                                    ("free start", node.is_free_to_move_from),
                                    ("no edges - only -d reaches it", edges == 0)) if on]
            label = f" '{node.label}'" if node.label else ""
            Logger.INFO(f"  [{node_id}]{label}  x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}  "
                        f"edges={edges}" + (f"  ({', '.join(tags)})" if tags else ""))

    def _where_am_i(self):
        current = current_robot_pose(self.robot, self.selected_groups)
        if current is None:
            return
        node = determine_start_node(self.robot, current, self.graph, 0.3)
        if node:
            Logger.INFO(f"Current position in graph is node ID {self.graph._get_or_assign_id(node)} "
                        f"with label '{node.label}'")
        else:
            Logger.WARN("Could not determine current position in graph.")

    def _move_to_typed_target(self):
        current = self.robot.get_ee_pose_for_group(CANONICAL_EE_GROUP)
        if not current:
            Logger.ERROR("Could not read the current tool pose.")
            return
        values = {}
        try:
            for axis, default in (('x', current.pos.x), ('y', current.pos.y), ('z', current.pos.z),
                                  ('qx', current.rot.x), ('qy', current.rot.y),
                                  ('qz', current.rot.z), ('qw', current.rot.w)):
                raw = input(f"Enter target {axis} (default {default:.3f}): ").strip()
                values[axis] = float(raw) if raw else default
        except ValueError:
            Logger.ERROR("Invalid number.")
            return
        ok = find_and_move_to_target_logic(
            self.robot, Point(values['x'], values['y'], values['z']),
            Quaternion(values['qx'], values['qy'], values['qz'], values['qw']),
            self.graph, self.selected_groups)
        (Logger.INFO if ok else Logger.ERROR)(
            "Moved to target pose." if ok else "Failed to move to target pose.")
