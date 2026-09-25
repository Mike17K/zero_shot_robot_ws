#!/usr/bin/python3
# Pinned to the SYSTEM interpreter: the gz-transport13 / gz-msgs10 Python
# bindings live in /usr/lib/python3/dist-packages (same as group_a_bringup's
# gripper_manager.py).
"""Accuracy check for box_pose_estimator against Gazebo ground truth.

Ground truth: box sizes from /box_factory/spawned (custom_msgs/SpawnedBox) and
live box poses read natively from gz-transport /world/<world>/dynamic_pose/info
(ros_gz_bridge drops entity names, so it can't go through the bridge - same as
gripper_manager.py). The expected top-face centre is the box centre plus half
its height along the box's +Z.

For every detected face (box_pose_estimator's ~/markers, which carry the face
size) it finds the nearest ground-truth box and logs position, yaw (mod 90 deg)
and size errors, plus running mean / max. With trigger_period > 0 it calls
~/detect itself, so it runs hands-off:

    ros2 run vision box_pose_check --ros-args -p namespace:=robot_1 -p trigger_period:=5.0
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

from custom_msgs.msg import SpawnedBox


def quat_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class BoxPoseCheck(Node):
    def __init__(self):
        super().__init__('box_pose_check')
        ns = self.declare_parameter('namespace', 'robot_1').value.strip('/')
        world = self.declare_parameter('world_name', 'default').value
        self.max_match = float(self.declare_parameter('max_match_distance', 0.25).value)
        period = float(self.declare_parameter('trigger_period', 0.0).value)
        prefix = self.declare_parameter('box_prefix', 'box').value

        self._lock = threading.Lock()
        self._gt_pose = {}   # name -> (position (3,), rotation (3, 3)), world frame
        self._gt_size = {}   # name -> (x, y, z)
        self._prefix = prefix
        self._errors = []    # (position m, yaw deg, size m)

        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self._on_gz_poses):
            self.get_logger().error(f'failed to subscribe to /world/{world}/dynamic_pose/info')
        self.create_subscription(SpawnedBox, '/box_factory/spawned', self._on_spawned, 50)
        self.create_subscription(MarkerArray, f'/{ns}/box_pose_estimator/markers', self._on_markers, 10)
        self._detect = self.create_client(Trigger, f'/{ns}/box_pose_estimator/detect')
        if period > 0.0:
            self.create_timer(period, self._trigger)
        self.get_logger().info(f'checking /{ns}/box_pose_estimator against Gazebo world {world!r}')

    def _on_gz_poses(self, msg: Pose_V) -> None:
        poses = {}
        for p in msg.pose:
            if p.name.startswith(self._prefix):
                q = p.orientation
                poses[p.name] = (np.array([p.position.x, p.position.y, p.position.z]),
                                 quat_matrix(q.x, q.y, q.z, q.w))
        with self._lock:
            self._gt_pose = poses

    def _on_spawned(self, msg: SpawnedBox) -> None:
        self._gt_size[msg.name] = (msg.size.x, msg.size.y, msg.size.z)

    def _trigger(self) -> None:
        if self._detect.service_is_ready():
            self._detect.call_async(Trigger.Request())

    def _on_markers(self, msg: MarkerArray) -> None:
        faces = [m for m in msg.markers if m.ns == 'box_top' and m.action == Marker.ADD]
        if not faces:
            return
        with self._lock:
            gt = {n: pose for n, pose in self._gt_pose.items() if n in self._gt_size}
        if not gt:
            self.get_logger().warn('no ground-truth boxes with a known size yet '
                                   '(box_factory running? boxes spawned before this node started are unknown)')
            return
        for m in faces:
            p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            q = m.pose.orientation
            x_axis = quat_matrix(q.x, q.y, q.z, q.w)[:, 0]
            best = None
            for name, (c, R) in gt.items():
                size = self._gt_size[name]
                top = c + R[:, 2] * size[2] / 2.0
                d = float(np.linalg.norm(top - p))
                if best is None or d < best[0]:
                    best = (d, name, R, size)
            d, name, R, size = best
            if d > self.max_match:
                self.get_logger().info(f'face #{m.id}: no ground-truth box within {self.max_match:.2f} m')
                continue
            yaw_det = math.atan2(x_axis[1], x_axis[0])
            yaw_gt = math.atan2(R[1, 0], R[0, 0])
            yaw_err = abs((math.degrees(yaw_det - yaw_gt) + 45.0) % 90.0 - 45.0)
            size_err = float(np.max(np.abs(np.sort([m.scale.x, m.scale.y]) - np.sort(size[:2]))))
            self._errors.append((d, yaw_err, size_err))
            e = np.array(self._errors)
            self.get_logger().info(
                f'face #{m.id} vs {name}: position {d * 1000:.1f} mm, yaw {yaw_err:.1f} deg, '
                f'size {size_err * 1000:.1f} mm | n={len(e)} mean {e[:, 0].mean() * 1000:.1f} mm / '
                f'{e[:, 1].mean():.1f} deg / {e[:, 2].mean() * 1000:.1f} mm, max '
                f'{e[:, 0].max() * 1000:.1f} mm / {e[:, 1].max():.1f} deg / {e[:, 2].max() * 1000:.1f} mm')


def main():
    rclpy.init()
    node = BoxPoseCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
