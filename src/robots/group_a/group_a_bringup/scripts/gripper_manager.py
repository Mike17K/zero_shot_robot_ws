#!/usr/bin/python3
# Pinned to the SYSTEM interpreter on purpose: the gz-transport13 / gz-msgs10
# Python bindings (apt: python3-gz-transport13, python3-gz-msgs10) live in
# /usr/lib/python3/dist-packages, which the workspace .venv does not see.
# Everything else this node imports (rclpy, tf2_ros, *_msgs) is a system/ROS
# package too, so nothing here needs the venv.
"""Per-robot suction-gripper manager: decides WHAT to grasp, WHEN, and makes
Gazebo actually hold it - with no fixed object pool and no per-object setup
anywhere in the URDF, launch files or bridge config.

How grasping works
------------------
gz-sim has no vacuum model, so a suction grasp is a rigid joint. Declaring a
DetachableJoint in the URDF would force naming the child model at build time
(a fixed object pool) and would weld each object on spawn. Instead this node
installs a DetachableJoint onto the robot model ON DEMAND through Gazebo's
runtime /world/<world>/entity/system/add service, configured for the one object
being grasped. Installing it IS the grasp: DetachableJoint attaches as soon as
it resolves its child model.

Every instance shares ONE detach topic, so a single publish releases everything
("suction off"). Re-grasping installs a fresh instance; idle instances are
detached no-ops, and they accumulate per grasp EVENT, not per object.

What gets grasped
-----------------
Suction is latching: while ON, every qualifying object entering the footprint
is captured (several at once is fine); OFF releases all. In the gripper TCP
frame (+Z = direction the cups point) an object qualifies when

    -suction_back_tol <= z <= suction_reach + (object half-height)
    |x| <= suction_half_width,  |y| <= suction_half_length

Poses come from /world/<world>/dynamic_pose/info (non-static entities only),
subscribed to NATIVELY via gz-transport. It can't go through ros_gz_bridge:
the Pose_V -> TFMessage conversion drops entity names (child_frame_id comes out
empty), and names are how objects are identified. Only entities whose name
starts with one of graspable_prefixes are considered, which keeps the arm, the
conveyors and all their links out.

Transport
---------
All Gazebo traffic uses the gz-transport13 Python bindings directly:
  subscribe  /world/<world>/dynamic_pose/info    (gz.msgs.Pose_V)
  request    /world/<world>/entity/system/add    (EntityPlugin_V -> Boolean)
  request    /world/<world>/create               (EntityFactory  -> Boolean)
  publish    /grasp/<robot>/detach_all           (gz.msgs.Empty)
No bridge entries and no subprocesses are involved.

ROS interface (namespaced per robot):
  gripper/set_suction (std_srvs/srv/SetBool)     - suction on/off
  gripper/spawn_box   (custom_msgs/srv/SpawnBox) - spawn one dynamic box
  gripper/state       (custom_msgs/msg/GripperState, latched) - suction on/off, held objects
  spawned_box_topics  (custom_msgs/msg/SpawnedBox) - sizes of boxes spawned elsewhere
"""
import math
import threading

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool

from gz.transport13 import Node as GzNode
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty as GzEmpty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.pose_v_pb2 import Pose_V

from custom_msgs.msg import GripperState, SpawnedBox
from rclpy.qos import DurabilityPolicy, QoSProfile
from custom_msgs.srv import SpawnBox

BOX_DENSITY_KG_M3 = 300.0  # cardboard-ish, for a plausible mass from size alone
BOX_MIN_MASS_KG = 0.05
BOX_COLOR_RGB = (0.76, 0.60, 0.42)  # cardboard tan
GZ_SERVICE_TIMEOUT_MS = 3000

_BOX_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{model_name}">
    <static>false</static>
    <link name="{link_name}">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
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


_PBR_TEMPLATE = """
          <pbr><metal>
            <albedo_map>{albedo_map}</albedo_map>
            <roughness>0.9</roughness><metalness>0.0</metalness>
          </metal></pbr>"""


def _box_sdf(model_name: str, link_name: str, w: float, d: float, h: float,
             rgb=BOX_COLOR_RGB, albedo_map: str = '') -> str:
    # Pose is NOT in the SDF: EntityFactory.pose places the model instead.
    mass = max(BOX_MIN_MASS_KG, BOX_DENSITY_KG_M3 * w * d * h)
    r, g, b = rgb
    pbr = _PBR_TEMPLATE.format(albedo_map=albedo_map) if albedo_map else ''
    return _BOX_SDF_TEMPLATE.format(
        model_name=model_name, link_name=link_name,
        w=w, d=d, h=h, mass=mass,
        ixx=mass * (d * d + h * h) / 12.0,
        iyy=mass * (w * w + h * h) / 12.0,
        izz=mass * (w * w + d * d) / 12.0,
        r=r, g=g, b=b, pbr=pbr,
    )


def _quat_conj_rotate(q, v):
    """Rotate v by the INVERSE of quaternion q=(x,y,z,w), i.e. express a
    world-frame vector in the frame q describes."""
    qx, qy, qz, qw = q
    tx = 2.0 * (-qy * v[2] + qz * v[1])
    ty = 2.0 * (-qz * v[0] + qx * v[2])
    tz = 2.0 * (-qx * v[1] + qy * v[0])
    return (
        v[0] + qw * tx + (-qy * tz + qz * ty),
        v[1] + qw * ty + (-qz * tx + qx * tz),
        v[2] + qw * tz + (-qx * ty + qy * tx),
    )


class GripperManager(Node):
    def __init__(self):
        super().__init__('gripper_manager')

        self.declare_parameter('world_name', 'default')
        # Empty -> fall back to this node's namespace, which is the name
        # workcell.launch.py spawns the robot model as.
        self.declare_parameter('robot_model_name', '')
        # gp_link_6, not gripper_plate_link: sdformat merges fixed-jointed child
        # links into their parent during URDF->SDF, so the plate, cups and
        # camera_link physically ARE gp_link_6 in the spawned model.
        self.declare_parameter('gripper_parent_link', 'gp_link_6')
        self.declare_parameter('gripper_frame', 'gripper_tcp')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('suction_half_width', 0.17)
        self.declare_parameter('suction_half_length', 0.22)
        self.declare_parameter('suction_reach', 0.01)
        self.declare_parameter('suction_back_tol', 0.02)
        # Used for objects this node did not spawn (size unknown). The zone test
        # is on the object's CENTER, so 0 would make any real box ungraspable.
        self.declare_parameter('unknown_object_half_height', 0.10)
        self.declare_parameter('graspable_prefixes', ['box'])
        self.declare_parameter('spawn_link_name', 'link')
        self.declare_parameter('update_rate_hz', 30.0)
        # Boxes spawned by other nodes (box_factory_bringup) are announced
        # here with their size, so the suction zone test uses their real
        # height instead of unknown_object_half_height.
        self.declare_parameter('spawned_box_topics', ['/box_factory/spawned'])

        self.world_name = self.get_parameter('world_name').value
        self.robot_model_name = (self.get_parameter('robot_model_name').value
                                 or self.get_namespace().strip('/'))
        self.parent_link = self.get_parameter('gripper_parent_link').value
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.half_width = float(self.get_parameter('suction_half_width').value)
        self.half_length = float(self.get_parameter('suction_half_length').value)
        self.reach = float(self.get_parameter('suction_reach').value)
        self.back_tol = float(self.get_parameter('suction_back_tol').value)
        self.unknown_half_h = float(self.get_parameter('unknown_object_half_height').value)
        self.graspable_prefixes = tuple(self.get_parameter('graspable_prefixes').value)
        self.spawn_link_name = self.get_parameter('spawn_link_name').value
        update_rate = float(self.get_parameter('update_rate_hz').value)

        if not self.robot_model_name:
            self.get_logger().error(
                'robot_model_name is empty and this node has no namespace - '
                'cannot identify the robot model in Gazebo, grasping disabled')

        self._svc_system_add = f'/world/{self.world_name}/entity/system/add'
        self._svc_create = f'/world/{self.world_name}/create'
        self._pose_topic = f'/world/{self.world_name}/dynamic_pose/info'
        self._detach_topic = f'/grasp/{self.robot_model_name}/detach_all'

        # ── State ────────────────────────────────────────────────────────────
        # _poses is written from a gz-transport thread; everything else is only
        # touched from ROS callbacks on the single-threaded executor.
        self._lock = threading.Lock()
        self._poses = {}
        self._suction_on = False
        self._grasped = set()
        self._install_failed = set()
        self._object_half_height = {}
        self._spawn_counter = 0
        # Gazebo entity id of this robot's model, learned from dynamic_pose/info.
        # entity/system/add targets entities by ID - a name-only request lands
        # the plugin on an invalid entity and DetachableJoint refuses to load
        # ("should be attached to a model entity"). 0 = not seen yet.
        self._robot_entity_id = 0

        # ── Gazebo side ──────────────────────────────────────────────────────
        self._gz = GzNode()
        self._detach_pub = self._gz.advertise(self._detach_topic, GzEmpty)
        if not self._gz.subscribe(Pose_V, self._pose_topic, self._on_gz_poses):
            self.get_logger().error(f'failed to subscribe to {self._pose_topic}')

        # ── ROS side ─────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_service(SetBool, 'gripper/set_suction', self._on_set_suction)
        self._state_pub = self.create_publisher(
            GripperState, 'gripper/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publish_state()
        self.create_service(SpawnBox, 'gripper/spawn_box', self._on_spawn_box)
        for topic in self.get_parameter('spawned_box_topics').value:
            self.create_subscription(SpawnedBox, topic, self._on_spawned_box, 50)
        self._timer = self.create_timer(1.0 / max(1.0, update_rate), self._on_timer)

        self.get_logger().info(
            f'gripper_manager ready: world={self.world_name!r} robot={self.robot_model_name!r} '
            f'parent_link={self.parent_link!r} footprint='
            f'{2 * self.half_width:.2f}x{2 * self.half_length:.2f}m reach={self.reach:.2f}m '
            f'prefixes={list(self.graspable_prefixes)}')

    def _on_spawned_box(self, msg: SpawnedBox) -> None:
        self._object_half_height[msg.name] = msg.size.z / 2.0

    # ── Object tracking (gz-transport thread) ──────────────────────────────

    def _on_gz_poses(self, msg: Pose_V) -> None:
        """Each dynamic_pose/info message carries EVERY dynamic entity, so the
        tracked set is replaced rather than merged - deleted models drop out
        on their own. The same stream also yields this robot's entity id."""
        poses = {}
        robot_id = 0
        for p in msg.pose:
            if not robot_id and p.name == self.robot_model_name:
                robot_id = p.id
            if not p.name.startswith(self.graspable_prefixes):
                continue
            poses[p.name] = (
                (p.position.x, p.position.y, p.position.z),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            )
        with self._lock:
            self._poses = poses
            if robot_id:
                self._robot_entity_id = robot_id

    # ── Geometry ────────────────────────────────────────────────────────────

    def _tcp_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(self.world_frame, self.gripper_frame, Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'{self.gripper_frame} TF lookup failed: {exc}',
                                   throttle_duration_sec=5.0)
            return None
        p, q = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    def _to_tcp_frame(self, tcp, obj_pos):
        (tx, ty, tz), q = tcp
        return _quat_conj_rotate(q, (obj_pos[0] - tx, obj_pos[1] - ty, obj_pos[2] - tz))

    def _in_suction_zone(self, local, model_name: str) -> bool:
        half_h = self._object_half_height.get(model_name, self.unknown_half_h)
        return (abs(local[0]) <= self.half_width
                and abs(local[1]) <= self.half_length
                and -self.back_tol <= local[2] <= self.reach + half_h)

    # ── Grasp control loop ──────────────────────────────────────────────────

    def _on_timer(self) -> None:
        if not self._suction_on:
            return
        tcp = self._tcp_pose()
        if tcp is None:
            return
        with self._lock:
            candidates = {n: p for n, p in self._poses.items()
                          if n not in self._grasped and n not in self._install_failed}
        if not candidates:
            self.get_logger().debug('suction on, no candidate objects tracked',
                                    throttle_duration_sec=2.0)
            return
        for name, (pos, _quat) in candidates.items():
            if not self._suction_on:  # turned off mid-loop
                break
            local = self._to_tcp_frame(tcp, pos)
            self.get_logger().debug(
                f'{name}: tcp-frame x={local[0]:+.3f} y={local[1]:+.3f} z={local[2]:+.3f}',
                throttle_duration_sec=1.0)
            if self._in_suction_zone(local, name):
                self._grasp(name)

    def _grasp(self, model_name: str) -> bool:
        with self._lock:
            robot_id = self._robot_entity_id
        if not robot_id:
            self.get_logger().warn(
                f'robot model {self.robot_model_name!r} not seen in {self._pose_topic} yet - '
                f'cannot grasp {model_name}', throttle_duration_sec=2.0)
            return False  # not marked failed: retried on the next tick
        if not self._install_detachable_joint(model_name, robot_id):
            self._install_failed.add(model_name)
            return False
        self._grasped.add(model_name)
        self.get_logger().info(f'grasped {model_name}')
        self._publish_state()
        return True

    def _publish_state(self) -> None:
        self._state_pub.publish(GripperState(suction_on=self._suction_on, grasped=sorted(self._grasped)))

    def _release_all(self) -> int:
        count = len(self._grasped)
        if count:
            self._detach_pub.publish(GzEmpty())  # every instance listens here
            self.get_logger().info(f'released {count} object(s)')
        self._grasped.clear()
        self._publish_state()
        # A fresh suction cycle retries objects whose install failed before.
        self._install_failed.clear()
        return count

    def _install_detachable_joint(self, model_name: str, robot_id: int) -> bool:
        req = EntityPlugin_V()
        req.entity.id = robot_id
        req.entity.name = self.robot_model_name
        req.entity.type = Entity.MODEL
        plugin = req.plugins.add()
        plugin.name = 'gz::sim::systems::DetachableJoint'
        plugin.filename = 'gz-sim-detachable-joint-system'
        plugin.innerxml = (
            f'<parent_link>{self.parent_link}</parent_link>'
            f'<child_model>{model_name}</child_model>'
            f'<child_link>{self.spawn_link_name}</child_link>'
            f'<detach_topic>{self._detach_topic}</detach_topic>'
            f'<suppress_child_warning>false</suppress_child_warning>'
        )
        ok, rep = self._gz.request(self._svc_system_add, req, EntityPlugin_V, Boolean,
                                   GZ_SERVICE_TIMEOUT_MS)
        if not ok or not rep.data:
            self.get_logger().error(
                f'{self._svc_system_add} failed for {model_name} '
                f'(transport_ok={ok}, reply={rep.data if ok else None})')
            return False
        return True

    # ── ROS services ────────────────────────────────────────────────────────

    def _on_set_suction(self, request, response):
        self._suction_on = bool(request.data)
        self._publish_state()
        if self._suction_on:
            response.success = True
            response.message = 'suction ON - capturing objects in the gripper footprint'
        else:
            released = self._release_all()
            response.success = True
            response.message = f'suction OFF - released {released} object(s)'
        return response

    def _next_box_name(self) -> str:
        prefix = self.graspable_prefixes[0] if self.graspable_prefixes else 'box'
        with self._lock:
            existing = set(self._poses)
        # Skip names already in the world (e.g. boxes left over from a previous
        # run of this node) - /create does not fail loudly on a duplicate.
        while True:
            self._spawn_counter += 1
            name = f'{prefix}_{self._spawn_counter}'
            if name not in existing:
                return name

    def _on_spawn_box(self, request, response):
        if request.rgb and len(request.rgb) != 3:
            response.success = False
            response.message = f'rgb must be empty or have 3 values, got {len(request.rgb)}'
            return response
        rgb = (tuple(min(1.0, max(0.0, c)) for c in request.rgb)
               if request.rgb else BOX_COLOR_RGB)
        model_name = self._next_box_name()
        req = EntityFactory()
        req.sdf = _box_sdf(model_name, self.spawn_link_name,
                           request.width, request.depth, request.height,
                           rgb=rgb, albedo_map=request.albedo_map)
        req.name = model_name
        req.allow_renaming = False
        req.pose.position.x = request.x
        req.pose.position.y = request.y
        req.pose.position.z = request.z
        req.pose.orientation.z = math.sin(request.yaw / 2.0)
        req.pose.orientation.w = math.cos(request.yaw / 2.0)

        ok, rep = self._gz.request(self._svc_create, req, EntityFactory, Boolean,
                                   GZ_SERVICE_TIMEOUT_MS)
        if not ok or not rep.data:
            response.success = False
            response.message = (f'{self._svc_create} failed '
                                f'(transport_ok={ok}, reply={rep.data if ok else None})')
            return response

        self._object_half_height[model_name] = request.height / 2.0
        response.success = True
        response.message = (
            f'spawned {model_name} ({request.width:.2f}x{request.depth:.2f}x'
            f'{request.height:.2f}m) at ({request.x:.2f}, {request.y:.2f}, {request.z:.2f}) '
            f'yaw={math.degrees(request.yaw):.0f}deg')
        return response

    def destroy_node(self):
        self._gz.unsubscribe(self._pose_topic)
        return super().destroy_node()


def main():
    rclpy.init()
    node = GripperManager()
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