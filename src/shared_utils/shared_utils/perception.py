"""Client for vision's box_pose_estimator: trigger one estimate and get the
detected box top faces back (see docs/VISION.md).

    from shared_utils.perception import BoxDetectionClient
    boxes = BoxDetectionClient(node, namespace='robot_1')   # node must be spinning
    r = boxes.detect()
    if r.success and r.boxes:
        biggest = max(r.boxes, key=lambda b: b.area)
        biggest.pose      # PoseStamped, top-face centre; +Z = face normal (towards the camera)
        biggest.size      # (long, short) edge lengths, m; long edge = the pose's +X

The estimator's ~/detect (std_srvs/Trigger) only answers with a summary; the
faces themselves come on its ~/markers (one 'box_top' cube per face, scale =
face size), published just before the reply. detect() records how many marker
messages it has seen, calls the service and waits for the next one.
"""
import threading
from dataclasses import dataclass, field
from typing import Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .ros_helpers import call_service


@dataclass
class BoxDetection:
    pose: PoseStamped              # top-face centre; +X long edge, +Z face normal (towards the camera)
    size: tuple                    # (long, short) edge lengths, m

    @property
    def area(self) -> float:
        return self.size[0] * self.size[1]


@dataclass
class DetectionResult:
    success: bool
    message: str = ''
    boxes: list = field(default_factory=list)   # BoxDetection, closest to the camera first


class BoxDetectionClient:
    def __init__(self, node: Node, namespace: str = 'robot_1', estimator: str = 'box_pose_estimator'):
        self._node = node
        prefix = f"/{namespace.strip('/')}/{estimator}"
        self._detect = node.create_client(Trigger, f'{prefix}/detect')
        self._cond = threading.Condition()
        self._count = 0
        self._latest: Optional[MarkerArray] = None
        node.create_subscription(MarkerArray, f'{prefix}/markers', self._on_markers, 10)

    def _on_markers(self, msg: MarkerArray) -> None:
        with self._cond:
            self._latest = msg
            self._count += 1
            self._cond.notify_all()

    def detect(self, timeout_sec: float = 15.0) -> DetectionResult:
        """Run one estimate on the latest camera frames. Blocking."""
        with self._cond:
            seen = self._count
        response = call_service(self._node, self._detect, Trigger.Request(), timeout_sec=timeout_sec)
        if response is None:
            return DetectionResult(False, f'{self._detect.srv_name} unavailable or timed out '
                                          f'(ros2 launch vision box_pose.launch.py running?)')
        if not response.success:
            return DetectionResult(False, response.message)
        with self._cond:
            if not self._cond.wait_for(lambda: self._count > seen, timeout=2.0):
                return DetectionResult(False, f'{response.message} - but no markers received')
            msg = self._latest
        boxes = []
        for m in msg.markers:
            if m.ns != 'box_top' or m.action != Marker.ADD:
                continue
            pose = PoseStamped()
            pose.header, pose.pose = m.header, m.pose
            boxes.append(BoxDetection(pose, (m.scale.x, m.scale.y)))
        return DetectionResult(True, response.message, boxes)
