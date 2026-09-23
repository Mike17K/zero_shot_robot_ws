#!/usr/bin/python3
# Pinned to the SYSTEM interpreter for the same reason as group_a_bringup's
# gripper_manager.py: the gz-transport13 / gz-msgs10 Python bindings live in
# /usr/lib/python3/dist-packages, which the workspace .venv does not see.
"""Box factory node - the operating node of the box_factory "robot" (see
box_factory_description): spawns a stream of boxes with random sizes, poses
and textures at the factory's outlet on the infeed belt - the
domain-randomization source for the zero-shot pick-and-place work.

All Gazebo traffic is this node's own, over gz-transport:
  request    /world/<world>/create             (EntityFactory -> Boolean)
  subscribe  /world/<world>/dynamic_pose/info  (spawn-area occupancy)
ROS interface (under the factory namespace, e.g. /box_factory):
  spawned  custom_msgs/SpawnedBox        every box: name, size, pose, look -
                                         gripper_manager learns box heights here
  status   custom_msgs/BoxFactoryStatus  latched, on change + 2 Hz
  parameters below (teleop Box Factory panel / `ros2 param set`)
Box names start with box_prefix ("box") so the gripper treats them as graspable.

Randomization (all live-tunable ROS parameters, driven by workcell_teleop's
Box Factory panel or `ros2 param set /box_factory/factory ...`):
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

Feeding line: with follow_line (default) the stream only runs while
workcell_bringup's infeed_line_controller reports the line RUNNING or
RELEASED on line_state_topic - a stopped (beam-blocked) or switched-off line
pauses the factory, and it resumes by itself when the line restarts. With no
state message (controller not launched, or silent for
line_state_timeout_sec) the factory runs as before.

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
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

from geometry_msgs.msg import Pose, Vector3

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory

from custom_msgs.msg import BoxFactoryStatus, InfeedLineState, SpawnedBox

TICK_SEC = 0.1
MIN_PERIOD_SEC = 0.2
MIN_EDGE_M = 0.02
TEXTURE_PX = 64
# The first textured box can take Gazebo several seconds to load before it
# replies (the box is created regardless) - see _spawn / _pending.
GZ_SERVICE_TIMEOUT_MS = 8000
PENDING_TIMEOUT_SEC = 20.0
STATUS_PERIOD_SEC = 0.5
BOX_DENSITY_KG_M3 = 300.0  # cardboard-ish, a plausible mass from size alone
BOX_MIN_MASS_KG = 0.05

_BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <static>false</static>
    <link name="{link}">
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{w} {d} {h}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>{pbr}
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{w} {d} {h}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
_PBR = """
          <pbr><metal>
            <albedo_map>{albedo_map}</albedo_map>
            <roughness>0.9</roughness><metalness>0.0</metalness>
          </metal></pbr>"""


def _box_sdf(box: SpawnedBox, link: str) -> str:
    w, d, h = box.size.x, box.size.y, box.size.z
    mass = max(BOX_MIN_MASS_KG, BOX_DENSITY_KG_M3 * w * d * h)
    r, g, b = box.rgb if len(box.rgb) == 3 else (1.0, 1.0, 1.0)
    return _BOX_SDF.format(
        name=box.name, link=link, w=w, d=d, h=h, mass=mass,
        ixx=mass * (d * d + h * h) / 12.0, iyy=mass * (w * w + h * h) / 12.0,
        izz=mass * (w * w + d * d) / 12.0, r=r, g=g, b=b,
        pbr=_PBR.format(albedo_map=box.albedo_map) if box.albedo_map else '')


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
        super().__init__('factory')

        self.declare_parameter('world_name', 'default')
        # Link name inside each spawned box model - must match what
        # gripper_manager attaches to (its spawn_link_name, default "link").
        self.declare_parameter('box_link_name', 'link')
        # Spawn area = the factory outlet on the belt (world frame, set by
        # box_factory_bringup from workcell_bringup/layout.py). The lane runs
        # along lane_yaw; spawn_x/spawn_y is its centre line at the outlet.
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
        self.declare_parameter('follow_line', True)
        self.declare_parameter('line_state_topic', '/conveyor_package_infeed/line/state')
        self.declare_parameter('line_state_timeout_sec', 3.0)  # wall time
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

        self._next_spawn = 0.0
        self._was_enabled = False
        self._count = 0
        self._name_counter = 0
        self._last_box = ''
        self._pending = {}  # name -> (SpawnedBox, wall time) awaiting confirmation
        self._gazebo_ok = True
        self._area_blocked = False
        self._last_status = None
        self._last_status_pub = 0.0

        world = self.get_parameter('world_name').value
        self._pose_topic = f'/world/{world}/dynamic_pose/info'
        self._create_service = f'/world/{world}/create'
        self._prefix = self.get_parameter('box_prefix').value
        self._lock = threading.Lock()
        self._box_xy = []
        self._box_names = set()
        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, self._pose_topic, self._on_gz_poses):
            self.get_logger().error(f'failed to subscribe to {self._pose_topic} - spawn area check disabled')

        self._line_state = None
        self._line_state_wall = 0.0
        self._line_paused = False
        self.create_subscription(
            InfeedLineState, self.get_parameter('line_state_topic').value, self._on_line_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._spawned_pub = self.create_publisher(SpawnedBox, 'spawned', 10)
        self._status_pub = self.create_publisher(BoxFactoryStatus, 'status', latched)

        # Wall-clock timer: scheduling is wall time anyway, and a sim running
        # far below real time (or no /clock yet) must not stall the factory.
        self.create_timer(TICK_SEC, self._on_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'box factory ready: {len(self._textures)} textures, spawning into {self._create_service} '
            f'at ({self.get_parameter("spawn_x").value:.2f}, {self.get_parameter("spawn_y").value:.2f}) '
            f'- enabled={self.get_parameter("enabled").value}')

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
        boxes = [p for p in msg.pose if p.name.startswith(self._prefix)]
        with self._lock:
            self._box_xy = [(p.position.x, p.position.y) for p in boxes]
            self._box_names = {p.name for p in boxes}

    def _spawn_area_blocked(self) -> bool:
        sx, sy, r = self._p('spawn_x'), self._p('spawn_y'), self._p('spawn_clearance')
        with self._lock:
            return any(math.hypot(x - sx, y - sy) < r for x, y in self._box_xy)

    # ── Feeding line gate ───────────────────────────────────────────────────

    def _on_line_state(self, msg: InfeedLineState) -> None:
        self._line_state = msg
        self._line_state_wall = time.monotonic()

    def _line_allows_spawn(self) -> bool:
        if not self._p('follow_line') or self._line_state is None:
            return True
        if time.monotonic() - self._line_state_wall > self._p('line_state_timeout_sec'):
            return True  # controller gone: behave as without it
        return self._line_state.mode in (InfeedLineState.MODE_RUNNING, InfeedLineState.MODE_RELEASED)

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

        allowed = self._line_allows_spawn()
        if allowed == self._line_paused:  # state change
            self._line_paused = not allowed
            if enabled:
                self.get_logger().info('paused - feeding line stopped' if self._line_paused
                                       else 'resumed - feeding line running')
            if allowed:
                self._next_spawn = now + self._next_gap()
        self._area_blocked = self._spawn_area_blocked()
        self._confirm_pending()

        if enabled and allowed and now >= self._next_spawn:
            if self._area_blocked:
                self.get_logger().info('spawn area occupied - waiting', throttle_duration_sec=5.0)
            else:
                self._spawn(self._random_box())
                self._next_spawn = time.monotonic() + self._next_gap()
        self._publish_status(enabled)

    def _next_box_name(self) -> str:
        with self._lock:
            existing = set(self._box_names) | set(self._pending)
        # Skip names already in the world (boxes left from an earlier run) -
        # /create does not fail loudly on a duplicate.
        while True:
            self._name_counter += 1
            name = f'{self._prefix}_f{self._name_counter}'
            if name not in existing:
                return name

    def _spawn(self, box: SpawnedBox) -> None:
        box.name = self._next_box_name()
        req = EntityFactory()
        req.sdf = _box_sdf(box, self._p('box_link_name'))
        req.name = box.name
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = (
            box.pose.position.x, box.pose.position.y, box.pose.position.z)
        req.pose.orientation.z, req.pose.orientation.w = box.pose.orientation.z, box.pose.orientation.w
        ok, rep = self._gz.request(self._create_service, req, EntityFactory, Boolean, GZ_SERVICE_TIMEOUT_MS)
        self._gazebo_ok = bool(ok)
        if not ok:
            # No reply in time - Gazebo may still be loading it. Count it once
            # it shows up in the pose stream (see _confirm_pending).
            self.get_logger().warn(f'{self._create_service} did not answer for {box.name} - '
                                   f'waiting for it to appear')
            self._pending[box.name] = (box, time.monotonic())
            return
        if not rep.data:
            self.get_logger().warn(f'{self._create_service} refused {box.name}', throttle_duration_sec=5.0)
            return
        self._announce(box)

    def _confirm_pending(self) -> None:
        if not self._pending:
            return
        with self._lock:
            present = set(self._box_names)
        now = time.monotonic()
        for name, (box, since) in list(self._pending.items()):
            if name in present:
                del self._pending[name]
                self._gazebo_ok = True
                self._announce(box)
            elif now - since > PENDING_TIMEOUT_SEC:
                del self._pending[name]
                self.get_logger().error(f'{name} never appeared in Gazebo - dropped')

    def _announce(self, box: SpawnedBox) -> None:
        self._count += 1
        self._last_box = box.name
        self._spawned_pub.publish(box)
        yaw = 2.0 * math.atan2(box.pose.orientation.z, box.pose.orientation.w)
        self.get_logger().info(
            f'spawned {box.name} ({box.size.x:.2f}x{box.size.y:.2f}x{box.size.z:.2f}m) at '
            f'({box.pose.position.x:.2f}, {box.pose.position.y:.2f}) yaw={math.degrees(yaw):.0f}deg')

    def _publish_status(self, enabled: bool) -> None:
        wait = self._next_spawn - time.monotonic()
        msg = BoxFactoryStatus(
            enabled=enabled, paused_by_line=self._line_paused, spawn_area_blocked=self._area_blocked,
            gazebo_ok=self._gazebo_ok, boxes_spawned=self._count, last_box=self._last_box,
            next_spawn_in=float(max(0.0, wait)) if enabled and not self._line_paused else -1.0)
        key = (msg.enabled, msg.paused_by_line, msg.spawn_area_blocked, msg.gazebo_ok, msg.boxes_spawned)
        now = time.monotonic()
        if key != self._last_status or now - self._last_status_pub >= STATUS_PERIOD_SEC:
            self._status_pub.publish(msg)
            self._last_status, self._last_status_pub = key, now

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

    def _random_box(self) -> SpawnedBox:
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

        box = SpawnedBox()
        # size.x lies along the box's own X; its yaw is lane_yaw + rel_yaw, so
        # at rel_yaw=0 the width runs along the lane.
        box.size = Vector3(x=float(w), y=float(d), z=float(h))
        yaw = math.atan2(math.sin(lane_yaw + rel_yaw), math.cos(lane_yaw + rel_yaw))
        box.pose = Pose()
        box.pose.position.x = self._p('spawn_x') + along * ca - across * sa
        box.pose.position.y = self._p('spawn_y') + along * sa + across * ca
        box.pose.position.z = self._p('belt_top_z') + self._p('drop_height') + h / 2.0
        box.pose.orientation.z, box.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        if self._textures:
            box.albedo_map = self._rng.choice(self._textures)  # color lives in the texture
        else:
            box.rgb = list(_random_base_color(self._rng))
        return box

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
