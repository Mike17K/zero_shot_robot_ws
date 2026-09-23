"""Pose / transform construction and conversion, plus a TF helper.

Quaternions are (x, y, z, w) everywhere, matching geometry_msgs.
"""
import math
from typing import Optional, Sequence

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


# ── Pure conversions ──────────────────────────────────────────────────────────

def quaternion_from_euler(roll: float, pitch: float, yaw: float, degrees: bool = False):
    """Static-axis XYZ (ROS rpy convention) -> (x, y, z, w)."""
    return tuple(Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees).as_quat())


def euler_from_quaternion(q: Sequence[float], degrees: bool = False):
    """(x, y, z, w) -> (roll, pitch, yaw), static-axis XYZ."""
    return tuple(Rotation.from_quat(q).as_euler('xyz', degrees=degrees))


def make_pose(position: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0),
              quat: Optional[Sequence[float]] = None, degrees: bool = False) -> Pose:
    """Pose from a position and either rpy or an explicit quat (x, y, z, w)."""
    q = quat if quat is not None else quaternion_from_euler(*rpy, degrees=degrees)
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in position)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def make_pose_stamped(frame_id: str, position: Sequence[float],
                      rpy: Sequence[float] = (0.0, 0.0, 0.0),
                      quat: Optional[Sequence[float]] = None, degrees: bool = False,
                      stamp=None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose = make_pose(position, rpy, quat, degrees)
    return ps


def make_transform_stamped(parent_frame: str, child_frame: str, position: Sequence[float],
                           rpy: Sequence[float] = (0.0, 0.0, 0.0),
                           quat: Optional[Sequence[float]] = None, degrees: bool = False,
                           stamp=None) -> TransformStamped:
    pose = make_pose(position, rpy, quat, degrees)
    t = TransformStamped()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    if stamp is not None:
        t.header.stamp = stamp
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    return t


def transform_from_pose_stamped(pose: PoseStamped, child_frame: str) -> TransformStamped:
    p, q = pose.pose.position, pose.pose.orientation
    return make_transform_stamped(pose.header.frame_id, child_frame, (p.x, p.y, p.z),
                                  quat=(q.x, q.y, q.z, q.w), stamp=pose.header.stamp)


def pose_to_matrix(pose: Pose) -> np.ndarray:
    p, q = pose.position, pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def transform_to_matrix(transform: Transform) -> np.ndarray:
    t, q = transform.translation, transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = (t.x, t.y, t.z)
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    return make_pose(T[:3, 3], quat=Rotation.from_matrix(T[:3, :3]).as_quat())


def offset_pose(pose: Pose, offset: Pose, in_pose_frame: bool = True) -> Pose:
    """Compose offset onto pose: in pose's own frame (pose * offset) or in
    the parent frame (offset * pose)."""
    a, b = pose_to_matrix(pose), pose_to_matrix(offset)
    return matrix_to_pose(a @ b if in_pose_frame else b @ a)


def pose_distance(a: Pose, b: Pose) -> tuple[float, float]:
    """(translation distance in m, rotation angle in rad) between two poses."""
    dp = math.dist((a.position.x, a.position.y, a.position.z),
                   (b.position.x, b.position.y, b.position.z))
    qa = Rotation.from_quat([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w])
    qb = Rotation.from_quat([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w])
    return dp, (qa.inv() * qb).magnitude()


# ── TF ────────────────────────────────────────────────────────────────────────

class TfHelper:
    """TF buffer fed from configurable topics - robots in this workspace
    publish on their own /<namespace>/tf, which tf2_ros.TransformListener
    (hardcoded to /tf) cannot follow without node-level remaps."""

    def __init__(self, node: Node, tf_topic: str = '/tf', tf_static_topic: Optional[str] = None,
                 cache_time_sec: float = 10.0):
        self._node = node
        self.buffer = Buffer(cache_time=Duration(seconds=cache_time_sec))
        tf_static_topic = tf_static_topic or f'{tf_topic}_static'
        qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        static_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(TFMessage, tf_topic, self._on_tf, qos)
        node.create_subscription(TFMessage, tf_static_topic, self._on_tf_static, static_qos)

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.buffer.set_transform(t, 'shared_utils.TfHelper')

    def _on_tf_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.buffer.set_transform_static(t, 'shared_utils.TfHelper')

    def lookup(self, target_frame: str, source_frame: str,
               timeout_sec: float = 1.0) -> Optional[TransformStamped]:
        """Latest transform source -> target, or None (logged) on failure."""
        try:
            return self.buffer.lookup_transform(target_frame, source_frame, Time(),
                                                timeout=Duration(seconds=timeout_sec))
        except TransformException as exc:
            self._node.get_logger().warn(f'TF {source_frame} -> {target_frame} failed: {exc}')
            return None

    def frame_pose(self, target_frame: str, frame: str,
                   timeout_sec: float = 1.0) -> Optional[PoseStamped]:
        """Pose of `frame` expressed in target_frame."""
        t = self.lookup(target_frame, frame, timeout_sec)
        if t is None:
            return None
        ps = PoseStamped()
        ps.header = t.header
        ps.header.frame_id = target_frame
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation
        return ps

    def transform_pose(self, pose: PoseStamped, target_frame: str,
                       timeout_sec: float = 1.0) -> Optional[PoseStamped]:
        """pose re-expressed in target_frame, or None on TF failure."""
        if pose.header.frame_id == target_frame:
            return pose
        t = self.lookup(target_frame, pose.header.frame_id, timeout_sec)
        if t is None:
            return None
        out = PoseStamped()
        out.header.frame_id = target_frame
        out.header.stamp = pose.header.stamp
        out.pose = matrix_to_pose(transform_to_matrix(t.transform) @ pose_to_matrix(pose.pose))
        return out
