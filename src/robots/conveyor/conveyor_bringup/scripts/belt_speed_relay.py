#!/usr/bin/env python3
"""Fans a single target speed out to belt_velocity_controller's full
Float64MultiArray command.

belt_velocity_controller (forward_command_controller/ForwardCommandController)
expects one command value per roller joint, and the roller COUNT varies per
belt instance (see conveyor_bringup/launch/bringup.launch.py's _num_rollers -
derived from length/roller_radius/roller_gap). Anything that wants to drive a
belt - a teleop UI, a test script - would otherwise have to know that count
itself. This node is the one place that does: it's launched with the actual
num_rollers for its own belt instance (a launch-time parameter, computed the
same place the controller's own joint list is), and turns a plain
std_msgs/Float64 "target_speed" into the correctly-sized array.

Also replaces the previous one-shot `ros2 topic pub` bootstrap
(belt_on_bootstrap in bringup.launch.py) - this node publishes the initial
speed itself on startup, then keeps running to relay future commands, so a
teleop UI (or anything else) publishing to ~/target_speed takes effect
immediately without needing its own knowledge of the roller count.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class BeltSpeedRelay(Node):
    def __init__(self):
        super().__init__('belt_speed_relay')
        self.declare_parameter('num_rollers', 4)
        self.declare_parameter('initial_speed', 0.0)
        self._num_rollers = int(self.get_parameter('num_rollers').value)

        # Relative topics - resolve under this node's own namespace, same
        # convention as every other topic in this workspace (see
        # group_a_description/urdf/group_a_macro.xacro's namespace-strategy
        # comment): namespace for topic scoping, nothing baked into names.
        self._pub = self.create_publisher(Float64MultiArray, 'belt_velocity_controller/commands', 10)
        self.create_subscription(Float64, 'target_speed', self._on_target_speed, 10)

        initial_speed = float(self.get_parameter('initial_speed').value)
        self._publish_speed(initial_speed)
        self.get_logger().info(
            f'belt_speed_relay ready: {self._num_rollers} rollers, initial_speed={initial_speed}'
        )

    def _on_target_speed(self, msg: Float64) -> None:
        self._publish_speed(msg.data)

    def _publish_speed(self, speed: float) -> None:
        out = Float64MultiArray()
        out.data = [float(speed)] * self._num_rollers
        self._pub.publish(out)


def main():
    rclpy.init()
    node = BeltSpeedRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
