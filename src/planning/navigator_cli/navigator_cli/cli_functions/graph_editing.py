from .interface import CLIFunctionBase

from typing import Dict, List, Optional, Any
from rclpy.node import Node

from ..robots.interface import RobotPose
from ..robots.wrapper import RobotInterface
from ..robots.utils import determine_start_node
from ..shared.constants import AVAILABLE_GROUPS, CANONICAL_EE_GROUP
from ..shared.graph import Graph, Edge
from ..shared.logger import Logger
import traceback


class GraphEditCLIFunction(CLIFunctionBase):
    name = "graph"
    accept_kwargs = False

    def __init__(self, node: Node, robot: RobotInterface, graph: Graph[RobotPose]):
        super().__init__(name=self.name)
        self._node = node
        self.robot = robot
        self.graph = graph
        self.selected_groups = list(AVAILABLE_GROUPS)
        self.previous_node: Optional[RobotPose] = None

        self.alias_map = {
            'a': 'add',
            'p': 'set-parent',
            'r': 'reload',
            'd': 'delete-edge',
            'x': 'delete-node',
            'j': 'add-edge',
            'l': 'list',
            's': 'save',
            'c': 'current',
        }
        
    @staticmethod
    def __str__():
        return (
            "Graph editing command. Usage: graph <subcommand> [args]\n"
            "Subcommands:\n"
            "  (a) add              - capture current robot pose and add it to the graph\n"
            "  (p) set-parent       - set the closest existing graph node as the parent for next added node\n"
            "  (r) reload           - reload graph from file\n"
            "  (d) delete-edge      - remove an existing directed edge by source, target and move group\n"
            "  (x) delete-node      - remove a node and its incident edges\n"
            "  (j) add-edge         - create a directed edge between two existing nodes\n"
            "  (l) list             - list all nodes and outgoing edges\n"
            "  (s) save [filename]  - save graph to file, optional filename override\n"
            "  (c) current          - show the current previous node"
        )

    def execute(self, args: List[str] = None, kwargs: Dict[str, Any] = None):
        if not args or len(args) == 0:
            Logger.INFO(str(self))
            return

        command = args[0].lower()
        command = self.alias_map.get(command, command)

        handlers = {
            'add': self._add_node,
            'set-parent': self._set_parent,
            'reload': self._reload,
            'delete-edge': self._delete_edge,
            'delete-node': self._delete_node,
            'add-edge': self._add_edge,
            'list': self._list,
            'save': self._save,
            'current': self._show_current,
            'help': lambda _: Logger.INFO(str(self)),
        }

        handler = handlers.get(command)
        if handler is None:
            Logger.ERROR(f"Unknown graph command: {command}")
            Logger.INFO(str(self))
            return

        try:
            handler(args[1:])
        except Exception as e:
            Logger.ERROR(f"Graph command '{command}' failed: {e}\n{traceback.format_exc()}")

    def _add_node(self, args: List[str]):
        all_ee_poses_for_new_node = self.robot.collect_all_ee_poses(self.selected_groups)
        if not all_ee_poses_for_new_node:
            Logger.ERROR("Could not collect all end-effector poses for new node. Node not added.")
            return

        canonical_ee_pose_for_new_node_tuple = all_ee_poses_for_new_node.get(CANONICAL_EE_GROUP)
        if not canonical_ee_pose_for_new_node_tuple:
            Logger.ERROR(f"Could not get canonical EE pose for new node from '{CANONICAL_EE_GROUP}'. Node not added.")
            return

        free_to_move_input = input("Can the planner freely move from this point? (y/N, default No): ").strip().lower()
        is_free_to_move = (free_to_move_input == 'y')
        label_input = input("Enter an optional label for this pose (e.g., 'Home', 'Pick_Point_A', or leave empty): ").strip()
        label = label_input if label_input else None

        on_demand_pose = RobotPose(
            all_ee_poses=all_ee_poses_for_new_node,
            is_free_to_move_from=is_free_to_move,
            label=label,
            is_joint_goal=True,
        )

        current_pose_in_graph = self.graph.add_node(on_demand_pose, current_all_joints=all_ee_poses_for_new_node[CANONICAL_EE_GROUP].joints)
        Logger.INFO(f"Pose added to self.graph. ID: {self.graph._get_or_assign_id(current_pose_in_graph)}")
        Logger.INFO(f"  Canonical EE Pos: x={current_pose_in_graph.all_ee_poses[CANONICAL_EE_GROUP].pos.x:.3f}, y={current_pose_in_graph.all_ee_poses[CANONICAL_EE_GROUP].pos.y:.3f}, z={current_pose_in_graph.all_ee_poses[CANONICAL_EE_GROUP].pos.z:.3f}")
        Logger.INFO(f"  Is Free To Move From: {current_pose_in_graph.is_free_to_move_from}")
        if current_pose_in_graph.label:
            Logger.INFO(f"  Label: '{current_pose_in_graph.label}'")

        parent_for_edge: Optional[RobotPose] = self.previous_node
        parentID = None
        parentIDResp = input("Enter the ID of the parent node to connect to (or leave empty to use previous node): ").strip()
        if parentIDResp:
            try:
                parentID = int(parentIDResp)
                parent_for_edge = self.graph._id_to_pose.get(parentID)
                if not parent_for_edge:
                    Logger.WARN(f"No node found with ID {parentID}. Will attempt to find a nearby node instead.")
                    parent_for_edge = None
            except ValueError:
                Logger.WARN("Invalid input for parent node ID. Will attempt to find a nearby node instead.")
                parent_for_edge = None

        if parent_for_edge:
            common_move_group = self._ask_move_group(
                f"MoveGroup for transition between ID {self.graph._get_or_assign_id(parent_for_edge)} "
                f"and ID {self.graph._get_or_assign_id(current_pose_in_graph)}")

            self.graph.add_edge(Edge(
                source_id=self.graph._get_or_assign_id(parent_for_edge),
                target_id=self.graph._get_or_assign_id(current_pose_in_graph),
                move_group_for_transition=common_move_group,
            ))
            self.graph.add_edge(Edge(
                source_id=self.graph._get_or_assign_id(current_pose_in_graph),
                target_id=self.graph._get_or_assign_id(parent_for_edge),
                move_group_for_transition=common_move_group,
            ))
        else:
            Logger.INFO("No suitable parent node found, so no edge was created. This pose is a new starting point.")

        self.previous_node = current_pose_in_graph
        Logger.INFO(f"Current pose (ID: {self.graph._get_or_assign_id(current_pose_in_graph)}) set as the new previous node for future additions.")
        self.graph.save_to_file()

    @staticmethod
    def _ask_move_group(prompt: str) -> str:
        value = input(f"{prompt} ({', '.join(AVAILABLE_GROUPS)}, empty for '{CANONICAL_EE_GROUP}'): ").strip()
        if value and value not in AVAILABLE_GROUPS:
            raise ValueError(f"unknown move group {value!r}")
        return value or CANONICAL_EE_GROUP

    def _set_parent(self, args: List[str]):
        if not self.graph.adjacency:
            Logger.WARN("Graph is empty. Cannot set previous node. No nodes to connect to.")
            self.previous_node = None
            return

        current_all_ee_poses = self.robot.collect_all_ee_poses(self.selected_groups)
        if not current_all_ee_poses:
            Logger.ERROR("Could not collect current end-effector poses. Cannot set previous node.")
            return

        current_canonical_ee_pose_tuple = current_all_ee_poses.get(CANONICAL_EE_GROUP)
        if not current_canonical_ee_pose_tuple:
            Logger.ERROR(f"Could not get current '{CANONICAL_EE_GROUP}' EE pose. Cannot set previous node.")
            return

        current_robot_state_pose = RobotPose(all_ee_poses=current_all_ee_poses)
        start_node_candidate = determine_start_node(
            self.robot,
            current_robot_state_pose,
            self.graph,
            0.5,
            current_robot_state_pose.canonical.joints,
        )
        if not start_node_candidate:
            Logger.WARN("No existing nodes in the graph. Cannot set previous node.")
            self.previous_node = None
            return

        self.previous_node = start_node_candidate
        Logger.INFO(f"Previous node (parent) set to closest node in graph (ID: {self.graph._get_or_assign_id(start_node_candidate)}).")
        Logger.INFO(
            f"  Canonical EE Pos: x={start_node_candidate.all_ee_poses[CANONICAL_EE_GROUP].pos.x:.3f}, y={start_node_candidate.all_ee_poses[CANONICAL_EE_GROUP].pos.y:.3f}, z={start_node_candidate.all_ee_poses[CANONICAL_EE_GROUP].pos.z:.3f}"
        )

    def _reload(self, args: List[str]):
        Logger.INFO("Reloading graph from file...")
        self.graph.load_from_file()
        self.previous_node = None
        Logger.INFO("Graph reloaded. Previous node reset.")

    def _delete_edge(self, args: List[str]):
        try:
            source_id = int(input("Enter the SOURCE node ID of the edge to delete: ").strip())
            target_id = int(input("Enter the TARGET node ID of the edge to delete: ").strip())
            move_group_for_transition = self._ask_move_group("MoveGroup of the edge to delete")
            self.graph.delete_edge(source_id, target_id, move_group_for_transition)
            self.graph.save_to_file()
        except ValueError:
            Logger.ERROR("Invalid input. Please enter numbers for node IDs.")
        except Exception as e:
            Logger.ERROR(f"Error during edge deletion: {e}")

    def _delete_node(self, args: List[str]):
        try:
            node_id = int(input("Enter the ID of the NODE to delete: ").strip())
            self.graph.delete_node(node_id)
            self.graph.save_to_file()
            if self.previous_node and self.graph._id_to_pose.get(self.graph._get_or_assign_id(self.previous_node)) is None:
                self.previous_node = None
                Logger.INFO("Previous node was deleted, resetting it to None.")
        except ValueError:
            Logger.ERROR("Invalid input. Please enter a number for the node ID.")
        except Exception as e:
            Logger.ERROR(f"Error during node deletion: {e}")

    def _add_edge(self, args: List[str]):
        if not self.graph.adjacency:
            Logger.WARN("Graph is empty. Cannot add edge between nodes.")
            return

        Logger.INFO("Available nodes in graph:")
        node_list = list(self.graph._pose_to_id.keys())
        for id, node in enumerate(node_list):
            node_id = self.graph._get_or_assign_id(node)
            node_label = f" ('{node.label}')" if node.label else ""
            Logger.INFO(
                f"  [{node_id}]{node_label} Canonical EE Pos: x={node.all_ee_poses[CANONICAL_EE_GROUP].pos.x:.3f}, y={node.all_ee_poses[CANONICAL_EE_GROUP].pos.y:.3f}, z={node.all_ee_poses[CANONICAL_EE_GROUP].pos.z:.3f}"
            )

        try:
            source_id = int(input("Enter the SOURCE node ID for the new edge: ").strip())
            target_id = int(input("Enter the TARGET node ID for the new edge: ").strip())

            source_node = self.graph._id_to_pose.get(source_id)
            target_node = self.graph._id_to_pose.get(target_id)

            if not source_node:
                Logger.ERROR(f"Source node with ID {source_id} not found.")
                return
            if not target_node:
                Logger.ERROR(f"Target node with ID {target_id} not found.")
                return

            move_group = self._ask_move_group(f"MoveGroup for transition from ID {source_id} to ID {target_id}")
            self.graph.add_edge(Edge(source_id=source_node.id, target_id=target_node.id, move_group_for_transition=move_group))
            self.graph.save_to_file()
        except ValueError:
            Logger.ERROR("Invalid input. Please enter numbers for node IDs.")
        except Exception as e:
            Logger.ERROR(f"Error adding edge: {e}")

    def _list(self, args: List[str]):
        if not self.graph._id_to_pose:
            Logger.INFO("Graph is empty. No nodes to list.")
            return

        Logger.INFO("--- Current Graph Nodes and Connections ---")
        for node_id, pose in self.graph._id_to_pose.items():
            node_label = f" ('{pose.label}')" if pose.label else ""
            free_to_move_flag = " (Free to Move From)" if pose.is_free_to_move_from else ""
            Logger.INFO(f"Node ID: {node_id}{node_label}{free_to_move_flag}")
            Logger.INFO(
                f"  Canonical EE Pos: x={pose.all_ee_poses[CANONICAL_EE_GROUP].pos.x:.3f}, y={pose.all_ee_poses[CANONICAL_EE_GROUP].pos.y:.3f}, z={pose.all_ee_poses[CANONICAL_EE_GROUP].pos.z:.3f}"
            )

            outgoing_edges = self.graph.adjacency.get(node_id, set())
            if outgoing_edges:
                Logger.INFO("  Outgoing Edges:")
                for edge in outgoing_edges:
                    target_node = self.graph._id_to_pose.get(edge.target_id)
                    target_label = f" ('{target_node.label}')" if target_node and target_node.label else ""
                    Logger.INFO(f"    -> ID {edge.target_id}{target_label} via '{edge.move_group_for_transition}'")
            else:
                Logger.INFO("  No outgoing edges.")
        Logger.INFO("-----------------------------------------")

    def _save(self, args: List[str]):
        if args and len(args) > 0 and args[0].strip():
            self.graph.set_file(filename=args[0].strip())
        self.graph.save_to_file()
        Logger.INFO("Graph saved successfully.")

    def _show_current(self, args: List[str]):
        if self.previous_node:
            Logger.INFO(f"Current previous node (parent): ID {self.graph._get_or_assign_id(self.previous_node)}")
        else:
            Logger.INFO("No previous node (parent) currently set.")
