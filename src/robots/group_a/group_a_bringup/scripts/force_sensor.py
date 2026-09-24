#!/usr/bin/python3
"""Gripper force pad -> proper ROS topics.

Gazebo's force_torque sensor on gripper_force_sensor_joint (group_a_macro.xacro)
comes over ros_gz_bridge as gripper/force_torque_raw, without a usable
frame_id. This node re-publishes it stamped in the pad's own frame and derives
a contact flag from it.

The wrench is the one the pad exerts on the gripper plate, in the pad's axes
(+Z out of the gripper face) - pressing the face onto something shows up as a
force along -Z. At rest it reads only the pad's own weight (0.05 kg, ~0.5 N),
removed by taring.

ROS interface (namespaced per robot):
  gripper/wrench             geometry_msgs/WrenchStamped   frame gripper_force_sensor_link
  gripper/contact            std_msgs/Bool (latched)        |force - tare| > contact_force
  gripper/force_sensor/tare  std_srvs/Trigger               tare = mean of the last samples

Parameters: contact_force (N, default 5.0), tare_samples (default 20),
frame_id (default gripper_force_sensor_link).
"""
from collections import deque
import math

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ForceSensor(Node):
    def __init__(self):
        super().__init__('force_sensor')
        self.contact_force = float(self.declare_parameter('contact_force', 5.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'gripper_force_sensor_link').value
        self._recent = deque(maxlen=int(self.declare_parameter('tare_samples', 20).value))
        self._tare = (0.0, 0.0, 0.0)
        self._contact = None

        self._wrench_pub = self.create_publisher(WrenchStamped, 'gripper/wrench', 10)
        self._contact_pub = self.create_publisher(
            Bool, 'gripper/contact', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(WrenchStamped, 'gripper/force_torque_raw', self._on_raw,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, 'gripper/force_sensor/tare', self._on_tare)
        self.get_logger().info(f'force pad -> gripper/wrench ({self.frame_id}), '
                               f'gripper/contact above {self.contact_force:.1f} N')

    def _on_raw(self, msg: WrenchStamped) -> None:
        msg.header.frame_id = self.frame_id
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()
        self._wrench_pub.publish(msg)

        f = msg.wrench.force
        self._recent.append((f.x, f.y, f.z))
        contact = math.dist((f.x, f.y, f.z), self._tare) > self.contact_force
        if contact != self._contact:
            self._contact = contact
            self._contact_pub.publish(Bool(data=contact))

    def _on_tare(self, request, response):
        if not self._recent:
            response.success, response.message = False, 'no force samples yet'
            return response
        n = len(self._recent)
        self._tare = tuple(sum(s[i] for s in self._recent) / n for i in range(3))
        response.success = True
        response.message = 'tared at ({:.2f}, {:.2f}, {:.2f}) N'.format(*self._tare)
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = ForceSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
