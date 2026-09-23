import os
from .types import TrajectoryMovementOptions
from .constants import CANONICAL_EE_GROUP, CONFIG_FOLDER, FLOAT_TOLERANCE
from .logger import Logger
from .model import FrozenModel
from collections import deque
from typing import Any, Protocol, Set, List, Tuple, Optional, Dict, Type, TypeVar, Generic
import json
import math
import traceback

# --------------- Types ----------------



class Edge(FrozenModel):
    """Graph edge representation with source, target, move group, and options."""
    source_id: int
    target_id: int
    move_group_for_transition: str
    options: Optional[TrajectoryMovementOptions] = None

class NodeProtocol(Protocol):
    id: int
    label: Optional[str]

    def model_dump(self) -> Dict[str, Any]: ...
    @classmethod
    def model_validate_json(cls, data: str) -> "NodeProtocol": ...

    def get_similarity(self, other: 'NodeProtocol') -> float: ...

    def copy_with(self, **kwargs) -> 'NodeProtocol': ...

# --------------- Functionality ----------------

T = TypeVar("T", bound=NodeProtocol)

class Graph(Generic[T]):
    """Graph representation for pose planning."""
    DEFAULT_GRAPH_FILE = os.path.join(CONFIG_FOLDER,"pose_graph.json")
    
    def __init__(self, node_cls: Type[T]):
        self.node_cls = node_cls
        self.adjacency: Dict[int, Set[Edge]] = {}
        self._pose_to_id: Dict[T, int] = {}
        self._id_to_pose: Dict[int, T] = {}
        self._next_id: int = 0

    def clear(self):
        self.adjacency.clear()
        self._pose_to_id.clear()
        self._id_to_pose.clear()
        self._next_id = 0

    def get_by_label(self, label: str) -> T:
        for p in self._pose_to_id.keys():
            if p.label == label:
                return p
        return None

    def _get_or_assign_id(self, pose: T, force: bool = False) -> int:
        if pose not in self._pose_to_id:
            assigned_id = self._next_id if not force else pose.id
            self._pose_to_id[pose] = assigned_id
            self._id_to_pose[assigned_id] = pose
            self._next_id += 1
        return self._pose_to_id[pose]

    def delete_edge(self, source_id: int, target_id: int, move_group_for_transition: str) -> bool:
        if source_id in self.adjacency:
            edge_to_delete = Edge(
    source_id=source_id,
    target_id=target_id,
    move_group_for_transition=move_group_for_transition
)

            if edge_to_delete in self.adjacency[source_id]:
                self.adjacency[source_id].remove(edge_to_delete)
                Logger.INFO(f"Deleted edge from ID {source_id} to ID {target_id} using '{move_group_for_transition}'.")
                return True
        Logger.WARN(f"Edge from ID {source_id} to ID {target_id} using '{move_group_for_transition}' not found.")
        return False

    def delete_node(self, node_id: int) -> bool:
        if node_id not in self._id_to_pose:
            Logger.WARN(f"Node with ID {node_id} not found in graph.")
            return False

        pose_to_delete = self._id_to_pose[node_id]

        del self._id_to_pose[node_id]
        del self._pose_to_id[pose_to_delete]

        if node_id in self.adjacency:
            num_outgoing = len(self.adjacency[node_id])
            del self.adjacency[node_id]
            Logger.INFO(f"Removed node ID {node_id} and its {num_outgoing} outgoing edges.")

        edges_to_remove = []
        for source_id, edges_set in self.adjacency.items():
            for edge in edges_set:
                if edge.target_id == node_id:
                    edges_to_remove.append((source_id, edge))
        
        for source_id, edge in edges_to_remove:
            self.adjacency[source_id].remove(edge)
            Logger.INFO(f"Removed incoming edge from ID {source_id} to deleted node ID {node_id}.")

        Logger.INFO(f"Node ID {node_id} successfully deleted from graph.")
        return True

    def add_node(self, node: T, force: bool = False, current_all_joints: List[float]= None) -> T:
        existing_nearby_nodes_results = self.find_nearby_nodes(node,current_all_joints) if not force else []

        for existing_node, distance in existing_nearby_nodes_results:
            if distance <= FLOAT_TOLERANCE and existing_node == node:
                Logger.INFO(f"Found existing node (ID: {self._get_or_assign_id(existing_node, force)}) for current pose. Using existing node.")
                return existing_node

        if node in self._pose_to_id:
            return self._id_to_pose[self._pose_to_id[node]]
        # Store the copy that carries its id - saved nodes need it to reload.
        node_id = node.id if force and node.id is not None else self._next_id
        node = node.copy_with(id=node_id)
        self._pose_to_id[node] = node_id
        self._id_to_pose[node_id] = node
        self._next_id = max(self._next_id, node_id + 1)
        self.adjacency[node_id] = set()
        Logger.INFO(f"Added new node (ID: {node_id}).")
        return node

    def add_edge(self, new_edge: Edge):
        source_id = new_edge.source_id
        target_id = new_edge.target_id

        # validate source and target IDs
        if source_id not in self._id_to_pose:
            Logger.ERROR(f"Source ID {source_id} not found in graph. Cannot add edge.")
            return

        if target_id not in self._id_to_pose:
            Logger.ERROR(f"Target ID {target_id} not found in graph. Cannot add edge.")
            return

        if source_id not in self._id_to_pose or target_id not in self._id_to_pose:
            Logger.ERROR(f"Source ID {source_id} or target ID {target_id} not found in graph. Cannot add edge.")
            return
        
        if source_id not in self.adjacency:
            self.adjacency[source_id] = set()
        
        if new_edge not in self.adjacency[source_id]:
            self.adjacency[source_id].add(new_edge)
            Logger.INFO(f"Added directed edge from ID {source_id} to ID {target_id} using '{new_edge.move_group_for_transition}'.")
        else:
            Logger.INFO(f"Edge from ID {source_id} to ID {target_id} with '{new_edge.move_group_for_transition}' already exists.")

    def neighbors(self, node: T) -> Set[Edge]:
        node_id = self._pose_to_id.get(node)
        if node_id is None:
            return set()
        return self.adjacency.get(node_id, set())

    def shortest_path(self, start: T, goal: T) -> List[Tuple[T, str, TrajectoryMovementOptions]]:
        start_id = self._pose_to_id.get(start)
        goal_id = self._pose_to_id.get(goal)

        if start_id is None or goal_id is None or start_id not in self.adjacency or goal_id not in self.adjacency:
            return []

        queue = deque([(start_id, [(start, "", "")])])
        visited = {start_id}

        while queue:
            current_id, path = queue.popleft()

            if current_id == goal_id:
                return path

            for edge in self.adjacency.get(current_id, set()):
                neighbor_id = edge.target_id
                # ignore ids above 1000 cause they are temp
                if neighbor_id > 1000:
                    continue
                if neighbor_id not in visited:
                    neighbor_pose = self._id_to_pose.get(neighbor_id)
                    if neighbor_pose:
                        visited.add(neighbor_id)
                        queue.append((neighbor_id, path + [(neighbor_pose, edge.move_group_for_transition, edge.options)]))
        return []

    def find_nearby_nodes(self, target_pose: T, current_all_joints: List[float] = None) -> List[Tuple[T, float]]:
        """Connected nodes sorted by closeness to target_pose: RMS joint
        difference of the canonical group, against current_all_joints if
        given (the robot's live joints), else target_pose's own joints."""
        assert isinstance(target_pose, self.node_cls), (
            f"target_pose of type {type(target_pose)}, expected {self.node_cls}")
        reference = list(current_all_joints) if current_all_joints is not None else \
            list(target_pose.all_ee_poses[CANONICAL_EE_GROUP].joints)

        def score(node: T) -> float:
            joints = node.all_ee_poses[CANONICAL_EE_GROUP].joints
            if len(joints) != len(reference):
                return float('inf')
            return math.sqrt(sum((a - b) ** 2 for a, b in zip(joints, reference)) / len(joints))

        results = [(node, score(node)) for node in self._pose_to_id.keys()
                   if node.id is not None and node.id < 1000]  # ids >= 1000 are temporary
        results.sort(key=lambda x: x[1])
        return [r for r in results if self.adjacency.get(r[0].id)]

    def get_all_data(self) -> Tuple[List[T], List[Edge]]:
        return list(self._pose_to_id.keys()) , [edge for edges in self.adjacency.values() for edge in edges]
    
    def set_all_data(self, nodes: List[T], edges: List[Edge]):
        self.clear()
        for node in nodes:
            self.add_node(node, force=True)
        for edge in edges:
            self.add_edge(edge)
            
    def set_file(self, filename: str):
        self.DEFAULT_GRAPH_FILE = filename
        Logger.INFO(f"Graph file set to: {filename}")

    def save_to_file(self):
        serializable_data = {
            "nodes": [],
            "edges": []
        }

        for pose in self._pose_to_id.keys():
            serializable_data["nodes"].append(pose.model_dump(exclude_none=True))

        for source_id, edges_set in self.adjacency.items():
            for edge in edges_set:
                serializable_data["edges"].append(edge.model_dump(exclude_none=True))

        with open(self.DEFAULT_GRAPH_FILE, "w") as f:
            json.dump(serializable_data, f, indent=2)
        Logger.INFO(f"Graph saved to {self.DEFAULT_GRAPH_FILE} with {len(serializable_data['nodes'])} nodes and {len(serializable_data['edges'])} edges.")

    def load_from_file(self):
        self.adjacency.clear()
        self._pose_to_id.clear()
        self._id_to_pose.clear()
        self._next_id = 0

        try:
            with open(self.DEFAULT_GRAPH_FILE, "r") as f:
                raw_data = json.load(f)

            if not isinstance(raw_data, dict) or "nodes" not in raw_data or "edges" not in raw_data:
                Logger.ERROR(f"Invalid graph file format in {self.DEFAULT_GRAPH_FILE}. Expected 'nodes' and 'edges' keys.")
                return

            for node_data in raw_data["nodes"]:
                node_id = node_data["id"]
                
                pose = self.node_cls.model_validate_json(json.dumps(node_data))
                
                self._pose_to_id[pose] = node_id
                self._id_to_pose[node_id] = pose
                self.adjacency[node_id] = set()
                if node_id >= self._next_id:
                    self._next_id = node_id + 1

            for edge_data in raw_data["edges"]:
                if len(edge_data) < 3: 
                    continue

                edge = Edge.model_validate_json(json.dumps(edge_data))
                
                self.add_edge(new_edge=edge)

            Logger.INFO(f"Loaded {len(self._pose_to_id)} poses and {sum(len(v) for v in self.adjacency.values())} edges from {self.DEFAULT_GRAPH_FILE}.")

        except FileNotFoundError:
            Logger.WARN(f"No existing graph file found: {self.DEFAULT_GRAPH_FILE}. Starting with an empty graph.")
        except json.JSONDecodeError as e:
            Logger.ERROR(f"Error decoding JSON from {self.DEFAULT_GRAPH_FILE}: {e}")
        except Exception as e:
            Logger.ERROR(f"An unexpected error occurred during graph loading: {e}")
            traceback.print_exc()
            self.adjacency.clear()
            self._pose_to_id.clear()
            self._id_to_pose.clear()
            self._next_id = 0
