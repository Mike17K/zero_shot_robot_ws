#!/usr/bin/python3
# Pinned to the SYSTEM interpreter for the same reason as group_a_bringup's
# gripper_manager.py: the gz-transport13 / gz-msgs10 Python bindings live in
# /usr/lib/python3/dist-packages, which the workspace .venv does not see.
"""Random box factory for the infeed conveyor: spawns a stream of boxes with
random sizes, poses and textures at the spawn area - the domain-randomization
source for the zero-shot pick-and-place work.

Every box is spawned through a robot's gripper/spawn_box service (see
group_a_bringup/scripts/gripper_manager.py), NOT straight through Gazebo, so it
gets a graspable name and gripper_manager knows its real height.

Randomization (all live-tunable ROS parameters, driven by workcell_teleop's
Box Factory panel or `ros2 param set /box_factory ...`):
  enabled              start/stop the stream
  period_sec           mean time between boxes
  time_randomness      0..1, each gap = period_sec * (1 + r * U(-1, 1))
  shape_randomness     0..1, blends every size/pose value from "always the
                       midpoint/centered/aligned" (0) to its full range (1)
  size_min/size_max    [width, depth, height] in meters. Each edge is drawn
                       from mid +- shape_randomness * (max - min) / 2
Pose randomization is a yaw of up to +-180 deg plus an offset across the lane
(limited so the rotated box still fits between the rails) and along it.

Textures are procedural PNGs (cardboard grain, tape stripes, checker, label
patch, gradient), each with its own random base color, generated once at
startup into a temp directory and referenced by absolute path. Every box picks
one at random.

No box is spawned while another box is still inside the spawn area (tracked
natively from /world/<world>/dynamic_pose/info) - that pauses the stream while
the belt is stopped instead of stacking boxes into each other. Timing uses wall
time, not sim time.
"""
import math
import os
import random
import struct
import tempfile
import threading
import time
import zlib

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

from custom_msgs.srv import SpawnBox

TICK_SEC = 0.1
MIN_PERIOD_SEC = 0.2
MIN_EDGE_M = 0.02
TEXTURE_PX = 64


# ── Procedural textures ───────────────────────────────────────────────────────

def _write_png(path: str, px: int, rows: list[bytes]) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))
    raw = b''.join(b'\x00' + row for row in rows)  # filter type 0 per row
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', px, px, 8, 2, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(raw)))
        f.write(chunk(b'IEND', b''))


def _random_base_color(rng: random.Random) -> tuple[float, float, float]:
    # Mostly cardboard-like browns, sometimes a saturated retail color.
    if rng.random() < 0.6:
        v = rng.uniform(0.45, 0.85)
        return (v, v * rng.uniform(0.72, 0.85), v * rng.uniform(0.45, 0.62))
    return (rng.random(), rng.random(), rng.random())


def _make_texture(path: str, rng: random.Random) -> None:
    base = _random_base_color(rng)
    accent = _random_base_color(rng)
    pattern = rng.choice(('grain', 'stripes', 'checker', 'label', 'gradient'))
    stripe_w = rng.randint(4, 12)
    cells = rng.choice((4, 8))
    lx0, ly0 = rng.randint(4, 24), rng.randint(4, 24)
    lx1, ly1 = lx0 + rng.randint(16, 32), ly0 + rng.randint(12, 24)
    horizontal = rng.random() < 0.5
    px = TEXTURE_PX
    rows = []
    for y in range(px):
        row = bytearray()
        for x in range(px):
            if pattern == 'stripes':
                use_accent = abs((y if horizontal else x) - px // 2) < stripe_w // 2
            elif pattern == 'checker':
                use_accent = ((x * cells // px) + (y * cells // px)) % 2 == 1
            elif pattern == 'label':
                use_accent = lx0 <= x < lx1 and ly0 <= y < ly1
            else:
                use_accent = False
            color = accent if use_accent else base
            if pattern == 'gradient':
                t = (y if horizontal else x) / (px - 1)
                color = tuple(b + (a - b) * t for a, b in zip(accent, base))
            noise = rng.uniform(0.9, 1.1)  # grain on every pattern
            row.extend(int(max(0.0, min(1.0, c * noise)) * 255) for c in color)
        rows.append(bytes(row))
    _write_png(path, px, rows)


# ── Node ──────────────────────────────────────────────────────────────────────

class BoxFactory(Node):
    def __init__(self):
        super().__init__('box_factory')

        self.declare_parameter('world_name', 'default')
        self.declare_parameter('spawn_service', '/robot_1/gripper/spawn_box')
        # Spawn area on the infeed belt (world frame). The lane runs along
        # lane_yaw; spawn_x/spawn_y is its center line at the spawn point.
        self.declare_parameter('spawn_x', -1.00)
        self.declare_parameter('spawn_y', 3.2)
        self.declare_parameter('lane_yaw', math.pi / 2.0)
        self.declare_parameter('lane_width', 0.5)
        self.declare_parameter('along_jitter', 0.15)
        self.declare_parameter('belt_top_z', 0.4)
        self.declare_parameter('drop_height', 0.03)
        # No spawn while any box center is closer than this (XY) to spawn_x/y.
        self.declare_parameter('spawn_clearance', 0.45)
        self.declare_parameter('box_prefix', 'box')
        self.declare_parameter('texture_pool_size', 32)
        self.declare_parameter('seed', -1)  # -1 = random every run
        # Live-tunable from the teleop UI.
        self.declare_parameter('enabled', False)
        self.declare_parameter('period_sec', 3.0)
        self.declare_parameter('time_randomness', 0.3)
        self.declare_parameter('shape_randomness', 0.5)
        self.declare_parameter('size_min', [0.10, 0.10, 0.08])
        self.declare_parameter('size_max', [0.30, 0.35, 0.25])
        self.add_on_set_parameters_callback(self._validate_params)

        seed = int(self.get_parameter('seed').value)
        self._rng = random.Random(None if seed < 0 else seed)
        self._textures = self._generate_textures(int(self.get_parameter('texture_pool_size').value))

        self._spawn_client = self.create_client(SpawnBox, self.get_parameter('spawn_service').value)
        self._pending = None
        self._next_spawn = 0.0
        self._was_enabled = False
        self._count = 0

        world = self.get_parameter('world_name').value
        self._pose_topic = f'/world/{world}/dynamic_pose/info'
        self._prefix = self.get_parameter('box_prefix').value
        self._lock = threading.Lock()
        self._box_xy = []
        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, self._pose_topic, self._on_gz_poses):
            self.get_logger().error(f'failed to subscribe to {self._pose_topic} - spawn area check disabled')

        self.create_timer(TICK_SEC, self._on_tick)
        self.get_logger().info(
            f'box_factory ready: {len(self._textures)} textures, spawning via '
            f'{self._spawn_client.srv_name} at ({self.get_parameter("spawn_x").value:.2f}, '
            f'{self.get_parameter("spawn_y").value:.2f}) - enabled={self.get_parameter("enabled").value}')

    def _generate_textures(self, count: int) -> list[str]:
        if count <= 0:
            return []
        out_dir = tempfile.mkdtemp(prefix='box_factory_textures_')
        paths = []
        for i in range(count):
            path = os.path.join(out_dir, f'box_texture_{i:03d}.png')
            _make_texture(path, self._rng)
            paths.append(path)
        return paths

    def _validate_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'period_sec' and p.value <= 0.0:
                return SetParametersResult(successful=False, reason='period_sec must be > 0')
            if p.name in ('time_randomness', 'shape_randomness') and not 0.0 <= p.value <= 1.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be in [0, 1]')
            if p.name in ('size_min', 'size_max'):
                if len(p.value) != 3 or min(p.value) < MIN_EDGE_M:
                    return SetParametersResult(
                        successful=False, reason=f'{p.name} must be 3 values >= {MIN_EDGE_M} m')
        return SetParametersResult(successful=True)

    def _p(self, name):
        return self.get_parameter(name).value

    # ── Spawn area tracking (gz-transport thread) ──────────────────────────

    def _on_gz_poses(self, msg: Pose_V) -> None:
        xy = [(p.position.x, p.position.y) for p in msg.pose if p.name.startswith(self._prefix)]
        with self._lock:
            self._box_xy = xy

    def _spawn_area_blocked(self) -> bool:
        sx, sy, r = self._p('spawn_x'), self._p('spawn_y'), self._p('spawn_clearance')
        with self._lock:
            return any(math.hypot(x - sx, y - sy) < r for x, y in self._box_xy)

    # ── Scheduling ──────────────────────────────────────────────────────────

    def _next_gap(self) -> float:
        r = self._p('time_randomness')
        return max(MIN_PERIOD_SEC, self._p('period_sec') * (1.0 + r * self._rng.uniform(-1.0, 1.0)))

    def _on_tick(self) -> None:
        enabled = bool(self._p('enabled'))
        now = time.monotonic()
        if enabled and not self._was_enabled:
            self._next_spawn = now  # first box right away
            self.get_logger().info('factory ON')
        elif not enabled and self._was_enabled:
            self.get_logger().info(f'factory OFF ({self._count} boxes spawned so far)')
        self._was_enabled = enabled
        if not enabled or self._pending is not None or now < self._next_spawn:
            return
        if not self._spawn_client.service_is_ready():
            self.get_logger().warn(f'{self._spawn_client.srv_name} not available yet',
                                   throttle_duration_sec=5.0)
            return
        if self._spawn_area_blocked():
            self.get_logger().info('spawn area occupied - waiting', throttle_duration_sec=5.0)
            return
        self._pending = self._spawn_client.call_async(self._random_request())
        self._pending.add_done_callback(self._on_spawned)

    def _on_spawned(self, future) -> None:
        self._pending = None
        self._next_spawn = time.monotonic() + self._next_gap()
        result = future.result()
        if result is None or not result.success:
            msg = result.message if result is not None else future.exception()
            self.get_logger().warn(f'spawn failed: {msg}')
            return
        self._count += 1
        self.get_logger().info(result.message)

    # ── Sampling ────────────────────────────────────────────────────────────

    def _sample_size(self, s: float) -> list[float]:
        size = []
        for lo, hi in zip(self._p('size_min'), self._p('size_max')):
            lo, hi = min(lo, hi), max(lo, hi)
            size.append((lo + hi) / 2.0 + s * self._rng.uniform(-1.0, 1.0) * (hi - lo) / 2.0)
        return size

    @staticmethod
    def _lateral_extent(w: float, d: float, rel_yaw: float) -> float:
        """Footprint across the lane of a w x d box rotated rel_yaw away from
        the lane's axes (box width along the lane, depth across it at 0)."""
        return abs(w * math.sin(rel_yaw)) + abs(d * math.cos(rel_yaw))

    def _random_request(self) -> SpawnBox.Request:
        s = self._p('shape_randomness')
        lane_w = self._p('lane_width')
        w, d, h = self._sample_size(s)

        # Keep the rotated box between the rails: halve the yaw until it fits,
        # then fall back to a quarter turn, then to shrinking the box.
        rel_yaw = s * self._rng.uniform(-math.pi, math.pi)
        for _ in range(6):
            if self._lateral_extent(w, d, rel_yaw) <= lane_w:
                break
            rel_yaw /= 2.0
        else:
            rel_yaw = 0.0 if d <= w else math.pi / 2.0
            extent = self._lateral_extent(w, d, rel_yaw)
            if extent > lane_w:
                scale = lane_w * 0.95 / extent
                w, d = w * scale, d * scale
        extent = self._lateral_extent(w, d, rel_yaw)

        across = s * self._rng.uniform(-1.0, 1.0) * max(0.0, lane_w - extent) / 2.0
        along = s * self._rng.uniform(-1.0, 1.0) * self._p('along_jitter')
        lane_yaw = self._p('lane_yaw')
        # Lane axis (along) is lane_yaw; across is +90 deg from it.
        ca, sa = math.cos(lane_yaw), math.sin(lane_yaw)

        req = SpawnBox.Request()
        # SpawnBox width is world X / depth world Y at yaw=0; the box's own
        # yaw is lane_yaw + rel_yaw, so width lies along the lane at rel_yaw=0.
        req.width, req.depth, req.height = w, d, h
        req.x = self._p('spawn_x') + along * ca - across * sa
        req.y = self._p('spawn_y') + along * sa + across * ca
        req.z = self._p('belt_top_z') + self._p('drop_height') + h / 2.0
        req.yaw = math.atan2(math.sin(lane_yaw + rel_yaw), math.cos(lane_yaw + rel_yaw))
        if self._textures:
            req.albedo_map = self._rng.choice(self._textures)
            req.rgb = [1.0, 1.0, 1.0]  # color lives in the texture itself
        else:
            req.rgb = list(_random_base_color(self._rng))
        return req

    def destroy_node(self):
        self._gz.unsubscribe(self._pose_topic)
        return super().destroy_node()


def main():
    rclpy.init()
    node = BoxFactory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
