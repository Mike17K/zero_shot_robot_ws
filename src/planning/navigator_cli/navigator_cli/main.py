#!/usr/bin/env python3
"""Interactive pose-graph navigator for group_a (GP70L). Teach poses into a
graph, connect them, then move along shortest paths - planned by cuMotion,
executed on the joint_trajectory_controller.

  ros2 run navigator_cli navigator_cli --ros-args -p namespace:=robot_1
"""
import threading
import traceback
from typing import Dict

import rclpy
from rclpy.node import Node

from .cli_functions import (CLIFunctionBase, GraphEditCLIFunction, GraphNavigationCLIFunction,
                            SuctionCLIFunction, TargetPositionCLIFunction)
from .robots.interface import RobotPose
from .robots.utils import current_robot_pose, determine_start_node
from .robots.wrapper import RobotInterface
from .shared.constants import AVAILABLE_GROUPS, CANONICAL_EE_GROUP
from .shared.graph import Graph
from .shared.logger import Logger

# XRDF default_joint_positions (planning_bringup/config/group_a/group_a.xrdf).
HOME_JOINTS = [0.0, -0.349066, 0.349066, 0.0, -0.523599, 0.0]

# Single-letter shortcuts for graph subcommands.
GRAPH_ALIASES = {
    'a': 'add', 'p': 'set-parent', 'r': 'reload', 'd': 'delete-edge',
    'x': 'delete-node', 'j': 'add-edge', 'l': 'list', 'save': 'save',
}


class InteractivePoseGraph:
    def __init__(self, node: Node):
        self._node = node
        self.robot = RobotInterface()
        self.robot.SetMode(node, isReal=True)
        self.selected_groups = list(AVAILABLE_GROUPS)

        self.graph: Graph[RobotPose] = Graph[RobotPose](node_cls=RobotPose)
        self.graph.load_from_file()

        cli_functions = [
            GraphNavigationCLIFunction(node, self.robot, self.graph),
            GraphEditCLIFunction(node, self.robot, self.graph),
            TargetPositionCLIFunction(node, self.robot),
            SuctionCLIFunction(node, self.robot),
        ]
        self.cli_function_dict: Dict[str, CLIFunctionBase] = {f.name: f for f in cli_functions}
        self._print_help()

        self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.cli_thread.start()

    def _print_help(self):
        Logger.INFO("Commands:")
        for func in self.cli_function_dict.values():
            Logger.INFO(f"  {func.name:8s} {func}")
        Logger.INFO("  a/p/r/d/x/j/l/save  shortcuts for 'graph add/set-parent/reload/delete-edge/"
                    "delete-node/add-edge/list/save'")
        Logger.INFO("  s <id>   show the path to node <id> without moving")
        Logger.INFO("  t        move the tool to a typed pose (same as 'e -t')")
        Logger.INFO("  home     move to the XRDF default joint position")
        Logger.INFO("  on/off   suction on/off")
        Logger.INFO("  help, exit")

    def cli_loop(self):
        while rclpy.ok():
            try:
                self.run()
            except (KeyboardInterrupt, EOFError):
                break
            except Exception as e:
                Logger.ERROR(f"CLI loop error: {e}\n{traceback.format_exc()}")
        rclpy.try_shutdown()

    def run(self):
        graph_cli = self.cli_function_dict[GraphEditCLIFunction.name]
        if graph_cli.previous_node:
            p = graph_cli.previous_node.canonical.pos
            Logger.INFO(f"Parent node for 'a': ID {self.graph._get_or_assign_id(graph_cli.previous_node)} "
                        f"(x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f})")

        cmd = input("navigator> ").strip()
        if not cmd:
            return
        first, _, rest = cmd.partition(" ")

        if first == 'exit':
            raise KeyboardInterrupt
        if first == 'help':
            return self._print_help()
        if first == 'home':
            return self._report(self.robot.execute_joint_goal(CANONICAL_EE_GROUP, HOME_JOINTS), "home")
        if first in ('on', 'off'):
            return self.cli_function_dict[SuctionCLIFunction.name].execute(args=[first])
        if first == 't':
            return self.cli_function_dict[GraphNavigationCLIFunction.name].execute(args=[], kwargs={'t': True})
        if first == 's':
            return self._show_path(rest)
        if first in GRAPH_ALIASES:
            return graph_cli.execute(args=[GRAPH_ALIASES[first]] + rest.split())

        func = self.cli_function_dict.get(first)
        if func is None:
            Logger.WARN(f"Invalid command: {cmd} (type 'help')")
            return
        # Tokens starting with '-' are kwargs (-key=value, or -flag -> True)
        # for commands that accept them; everything else is a positional arg.
        args, kwargs = [], {}
        for token in rest.split():
            if func.accept_kwargs and token.startswith('-'):
                key, _, value = token[1:].partition('=')
                kwargs[key] = value if value else True
            else:
                args.append(token)
        try:
            func.execute(args=args, kwargs=kwargs)
        except Exception as e:
            Logger.ERROR(f"Error executing command '{cmd}': {e}")

    @staticmethod
    def _report(ok: bool, what: str):
        (Logger.INFO if ok else Logger.ERROR)(f"{what}: {'done' if ok else 'failed'}")

    def _show_path(self, rest: str):
        try:
            target_id = int(rest.split()[0])
        except (IndexError, ValueError):
            Logger.ERROR("Usage: s <target_node_id>")
            return
        target = self.graph._id_to_pose.get(target_id)
        current = current_robot_pose(self.robot, self.selected_groups)
        if target is None or current is None:
            Logger.ERROR(f"Unknown node {target_id} or no robot state.")
            return
        start = determine_start_node(self.robot, current, self.graph, 0.3)
        path = self.graph.shortest_path(start, target) if start else []
        if not path:
            Logger.WARN("No path found.")
            return
        Logger.INFO(" -> ".join(f"[{self.graph._get_or_assign_id(p)}]" for p, _, _ in path))


def main(args=None):
    from .robots.ROS2.setup import runNode, setupNode

    node = setupNode(args)
    executor = runNode(node)
    try:
        InteractivePoseGraph(node).cli_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
