"""Static collision scene for cuMotion, generated from layout.py.

cuMotion runs with read_esdf_world: false (no nvblox ESDF - too much VRAM on
the 4GB card), so its whole world is this handful of boxes plus whatever a
MotionPlan request adds in `world` (e.g. perceived pallets/boxes).

Written in MoveIt's .scene text format, which isaac_ros_cumotion's
StaticPlanningSceneServer parses (utils.cpp ParseMoveItSceneFile). That
parser IGNORES frames and cuMotion uses the poses as-is in its own world,
which is the robot's base frame (gp_base_link) - so everything here is
converted from world into the robot's base frame first.
"""
import math
from typing import List, Tuple

from workcell_bringup import layout

# Pedestal under the robot (workcell_description/models/robot_pedestal):
# 1.12 x 1.12, floor to the robot base.
PEDESTAL_SIZE = 1.12
# The pedestal box's top is kept this far BELOW the base frame: gp_base_link's
# and gp_link_1's XRDF spheres (r 0.24 / 0.22 + 0.02 buffer) reach ~0.24m below
# the base and gp_link_1's sweep covers the whole 1.12m top, so a full-height
# pedestal would put every configuration in world collision.
PEDESTAL_TOP_CLEARANCE = 0.27
# Floor slab, top at z=0 in world.
FLOOR_SIZE = 8.0
FLOOR_THICKNESS = 0.02

Box = Tuple[str, Tuple[float, float, float], float, Tuple[float, float, float]]  # id, center, yaw, size


def _world_boxes() -> List[Box]:
    boxes: List[Box] = [
        ("floor", (0.0, 0.0, -FLOOR_THICKNESS / 2.0), 0.0, (FLOOR_SIZE, FLOOR_SIZE, FLOOR_THICKNESS)),
    ]
    for c in layout.CONVEYORS:
        x, y, top, yaw = layout.belt_frame(c)
        # Solid from the floor to the top of its side rails - conservative
        # (the real belt only has legs at its ends).
        h = top + float(c["side_rail_height"])
        boxes.append((c["name"], (x, y, h / 2.0), yaw, (float(c["length"]), float(c["width"]), h)))
    return boxes


def _robot(namespace: str) -> dict:
    return next(r for r in layout.ROBOTS if r["name"] == namespace)


def _yaw_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


BaseBox = Tuple[str, Tuple[float, float, float], Tuple[float, float, float, float], Tuple[float, float, float]]


def base_frame_boxes(namespace: str) -> List[BaseBox]:
    """(id, center, quaternion xyzw, size) of every box, in `namespace`'s base frame."""
    robot = _robot(namespace)
    bx, by, bz = (float(v) for v in robot["xyz"].split())
    roll, pitch, byaw = (float(v) for v in robot["rpy"].split())
    if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
        raise ValueError(f"{namespace}: only yaw-mounted robots are supported (rpy={robot['rpy']})")
    cos_y, sin_y = math.cos(byaw), math.sin(byaw)

    def to_base(p: Tuple[float, float, float]) -> Tuple[float, float, float]:
        dx, dy = p[0] - bx, p[1] - by
        return (cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy, p[2] - bz)

    pedestal_h = bz - PEDESTAL_TOP_CLEARANCE
    boxes = _world_boxes() + [
        ("pedestal", (bx, by, pedestal_h / 2.0), byaw, (PEDESTAL_SIZE, PEDESTAL_SIZE, pedestal_h)),
    ]

    return [(name, to_base(center), _yaw_quat(yaw - byaw), size) for name, center, yaw, size in boxes]


def collision_scene(namespace: str) -> str:
    """MoveIt .scene text of the static workcell, in `namespace`'s base frame."""
    lines = [f"{namespace}_workcell +"]
    for name, (cx, cy, cz), (qx, qy, qz, qw), size in base_frame_boxes(namespace):
        lines += [
            f"* {name}",
            f"{cx:.4f} {cy:.4f} {cz:.4f}",
            f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}",
            "1",
            "box",
            f"{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}",
            "0 0 0",
            "0 0 0 1",
            "0.5 0.5 0.5 1",
            "0",
        ]
    lines.append(".")
    return "\n".join(lines) + "\n"


def main():
    """Publish the boxes cuMotion plans against as RViz markers.

    cuMotion itself only visualizes the robot side (cumotion/collision_spheres)
    and the ESDF (cumotion/voxels); the static scene server's /planning_scene
    is stamped "world" although cuMotion uses those poses in the base frame,
    so RViz would draw it 0.4m low. These markers are stamped with the base
    frame, i.e. exactly where cuMotion sees them.

        ros2 run workcell_bringup cumotion_world_markers --ros-args -p namespace:=robot_1
    """
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node("cumotion_world_markers")
    ns = node.declare_parameter("namespace", "robot_1").value
    base_frame = node.declare_parameter("base_frame", "gp_base_link").value
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, f"/{ns}/cumotion/world_boxes", qos)

    markers = MarkerArray()
    for i, (name, center, quat, size) in enumerate(base_frame_boxes(ns)):
        m = Marker()
        m.header.frame_id = base_frame
        m.ns = name
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = center
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quat
        m.scale.x, m.scale.y, m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.35
        m.frame_locked = True
        markers.markers.append(m)
    pub.publish(markers)
    node.get_logger().info(
        f"Published {len(markers.markers)} cuMotion world boxes on /{ns}/cumotion/world_boxes ({base_frame})")
    rclpy.spin(node)


if __name__ == "__main__":
    import sys
    print(collision_scene(sys.argv[1] if len(sys.argv) > 1 else "robot_1"), end="")
