"""Latest-joint-state cache with name-based reordering."""
import threading
from typing import Optional, Sequence

from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateCache:
    """Keeps the latest JointState from `topic` (e.g. /robot_1/joint_states).
    Everything is looked up by joint NAME - joint_state_broadcaster does not
    guarantee any particular order."""

    def __init__(self, node: Node, topic: str = 'joint_states'):
        self._lock = threading.Lock()
        self._msg: Optional[JointState] = None
        self._received = threading.Event()
        node.create_subscription(JointState, topic, self._on_msg, 10)

    def _on_msg(self, msg: JointState) -> None:
        with self._lock:
            self._msg = msg
        self._received.set()

    def wait(self, timeout_sec: float = 2.0) -> bool:
        """Block until the first message arrives (the node must be spinning)."""
        return self._received.wait(timeout=timeout_sec)

    @property
    def latest(self) -> Optional[JointState]:
        with self._lock:
            return self._msg

    def positions(self, joint_names: Sequence[str]) -> Optional[list[float]]:
        """Positions in joint_names order, or None if no message yet or any
        joint is missing from it."""
        msg = self.latest
        if msg is None:
            return None
        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index for name in joint_names):
            return None
        return [msg.position[index[name]] for name in joint_names]


def reorder(names: Sequence[str], values: Sequence[float], target_names: Sequence[str]) -> list[float]:
    """values (given in `names` order) rearranged into target_names order.
    Raises KeyError on a missing joint."""
    lookup = dict(zip(names, values))
    return [lookup[name] for name in target_names]
