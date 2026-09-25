"""Client for navigator_cli's navigator_server (and the robot's gripper):
the ROS 2 message interface to the navigation layer, for the demo and any
other script.

    from shared_utils.navigation import NavigatorClient
    nav = NavigatorClient(node, namespace='robot_1')   # node must be spinning
    nav.navigate('6')                    # graph path to node 6 (or a label)
    nav.cartesian((0.0, 0.0, -0.10))     # 10 cm straight down (world frame)
    nav.cartesian((0.0, 0.0, -0.30), stop_force=5.0)   # down until the pad touches something
    nav.move_to_pose(pose_stamped)       # straight tool line to an absolute gripper_tcp pose
    nav.gripper(True)                    # suction on
    nav.holding()                        # models currently held

Interfaces (all under /<namespace>/):
    navigator/navigate_to_node  custom_msgs/action/NavigateToNode
    navigator/cartesian_move    custom_msgs/action/CartesianMove
    gripper/set_suction         std_srvs/srv/SetBool  (group_a_bringup gripper_manager)
    gripper/state               custom_msgs/msg/GripperState (latched)
"""
import threading
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool

from custom_msgs.action import CartesianMove, NavigateToNode
from custom_msgs.msg import GripperState

from .ros_helpers import call_service, send_action_goal


@dataclass
class NavResult:
    success: bool
    message: str = ''
    reached_node: int = -1   # navigate(): node reached
    fraction: float = 0.0    # cartesian(): part of the line followed
    contact: bool = False    # cartesian(stop_force>0): stopped on gripper-pad contact
    force: float = 0.0       # cartesian(stop_force>0): force change at the stop, N
    distance: float = 0.0    # cartesian(): distance the tool actually travelled, m


class NavigatorClient:
    def __init__(self, node: Node, namespace: str = 'robot_1'):
        self._node = node
        ns = namespace.strip('/')
        self.namespace = ns
        self._nav = ActionClient(node, NavigateToNode, f'/{ns}/navigator/navigate_to_node')
        self._cart = ActionClient(node, CartesianMove, f'/{ns}/navigator/cartesian_move')
        self._suction = node.create_client(SetBool, f'/{ns}/gripper/set_suction')
        self._lock = threading.Lock()
        self._gripper_state: Optional[GripperState] = None
        node.create_subscription(
            GripperState, f'/{ns}/gripper/state', self._on_gripper_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_gripper_state(self, msg: GripperState) -> None:
        with self._lock:
            self._gripper_state = msg

    # ── Availability ────────────────────────────────────────────────────────

    def wait_until_ready(self, timeout_sec: float = 10.0) -> bool:
        """True once both navigator actions are available."""
        return (self._nav.wait_for_server(timeout_sec=timeout_sec)
                and self._cart.wait_for_server(timeout_sec=timeout_sec))

    # ── Navigation ──────────────────────────────────────────────────────────

    def navigate(self, target, direct: bool = False, speed: float = 0.0,
                 timeout_sec: Optional[float] = None,
                 feedback: Optional[Callable[[NavigateToNode.Feedback], None]] = None) -> NavResult:
        """Move to a graph node by id or label. Blocking (no timeout by default:
        the server itself fails a stalled trajectory)."""
        goal = NavigateToNode.Goal(target=str(target), direct=bool(direct), speed=float(speed))
        response, _ = send_action_goal(
            self._node, self._nav, goal, timeout_sec=timeout_sec,
            feedback_callback=(lambda m: feedback(m.feedback)) if feedback else None)
        if response is None:
            return NavResult(False, 'navigate_to_node unavailable, rejected or timed out')
        r = response.result
        return NavResult(bool(r.success), r.message, reached_node=r.reached_node)

    def cartesian(self, offset: Sequence[float], frame: str = 'world', speed: float = 0.0,
                  stop_force: float = 0.0, timeout_sec: Optional[float] = None) -> NavResult:
        """Straight-line tool move by offset (x, y, z) meters. Blocking.
        stop_force > 0 makes it a guarded move: it stops as soon as the gripper
        force pad reads that much (N) more than at the start, and succeeds only
        on contact (offset is then the furthest it searches)."""
        goal = CartesianMove.Goal(offset=Vector3(x=float(offset[0]), y=float(offset[1]),
                                                 z=float(offset[2])),
                                  frame=frame, speed=float(speed), stop_force=float(stop_force))
        response, _ = send_action_goal(self._node, self._cart, goal, timeout_sec=timeout_sec)
        if response is None:
            return NavResult(False, 'cartesian_move unavailable, rejected or timed out')
        r = response.result
        return NavResult(bool(r.success), r.message, fraction=r.fraction,
                         contact=bool(r.contact), force=float(r.force), distance=float(r.distance))

    def move_to_pose(self, pose: PoseStamped, speed: float = 0.0, stop_force: float = 0.0,
                     timeout_sec: Optional[float] = None) -> NavResult:
        """Straight-line tool move to an absolute gripper_tcp pose (position and
        orientation interpolated along the line). Blocking; stop_force as in
        cartesian()."""
        goal = CartesianMove.Goal(use_target_pose=True, target_pose=pose, speed=float(speed),
                                  stop_force=float(stop_force))
        response, _ = send_action_goal(self._node, self._cart, goal, timeout_sec=timeout_sec)
        if response is None:
            return NavResult(False, 'cartesian_move unavailable, rejected or timed out')
        r = response.result
        return NavResult(bool(r.success), r.message, fraction=r.fraction,
                         contact=bool(r.contact), force=float(r.force), distance=float(r.distance))

    # ── Gripper ─────────────────────────────────────────────────────────────

    def gripper(self, on: bool) -> NavResult:
        """Suction on (latching capture) / off (release everything)."""
        response = call_service(self._node, self._suction, SetBool.Request(data=bool(on)))
        if response is None:
            return NavResult(False, f'{self._suction.srv_name} unavailable')
        return NavResult(bool(response.success), response.message)

    @property
    def gripper_state(self) -> Optional[GripperState]:
        with self._lock:
            return self._gripper_state

    def holding(self) -> list[str]:
        """Models currently held ([] if none, or no state received yet)."""
        state = self.gripper_state
        return list(state.grasped) if state else []
