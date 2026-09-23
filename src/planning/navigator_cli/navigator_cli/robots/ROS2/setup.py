import rclpy
from rclpy.node import Node

from shared_utils.ros_helpers import spin_in_background


def setupNode(args=None, name: str = 'navigator_cli') -> Node:
    """The CLI's (or navigator_server's) node. Parameters: namespace
    (robot_1), world_frame (world), tool_frame (gripper_tcp), controller
    (gp70l_joint_trajectory_controller)."""
    rclpy.init(args=args)
    node = Node(name)
    node.declare_parameter('namespace', 'robot_1')
    node.declare_parameter('world_frame', 'world')
    node.declare_parameter('tool_frame', 'gripper_tcp')
    node.declare_parameter('controller', 'gp70l_joint_trajectory_controller')
    return node


def runNode(node: Node):
    """Spin on background threads so the CLI thread can block on planning and
    execution calls; returns the executor."""
    return spin_in_background(node)
