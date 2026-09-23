"""Infeed feeding line: a laser beam across conveyor_package_infeed in front
of the robot, and the controller that stops the belt while a box blocks it.

  ros2 launch workcell_bringup infeed_line.launch.py

Start it after workcell.launch.py (it spawns into the running Gazebo). The
infeed belt's relay listens to line_speed (layout.py's "speed_topic"), so
without this launch the infeed belt ignores speed requests.

Pieces (all under the infeed namespace):
  - infeed_beam_sensor: static model with a 3-ray gpu_lidar, mounted just
    inside the belt's left rail and shooting across the belt, beam_height
    above the conveying surface and beam_offset from the robot-side end
  - ros_gz_bridge: gz LaserScan -> /<ns>/line/beam
  - infeed_line_controller (workcell_bringup/infeed_line_controller.py)
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from workcell_bringup import layout

_SENSOR_SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <!-- Emitter housing sits behind the sensor origin (towards the rail) so
           the rays start in free space. No collision: boxes pass under it. -->
      <visual name="emitter">
        <pose>-0.012 0 0 0 0 0</pose>
        <geometry><box><size>0.02 0.03 0.03</size></box></geometry>
        <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="beam" type="gpu_lidar">
        <topic>{topic}</topic>
        <update_rate>{rate}</update_rate>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <lidar>
          <scan>
            <horizontal>
              <samples>3</samples>
              <resolution>1</resolution>
              <min_angle>-0.01</min_angle>
              <max_angle>0.01</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.005</min>
            <max>{max_range}</max>
            <resolution>0.001</resolution>
          </range>
        </lidar>
      </sensor>
    </link>
  </model>
</sdf>
"""


def launch_setup(context):
    ns = LaunchConfiguration("conveyor").perform(context)
    belt = layout.conveyor(ns)
    width, length = float(belt["width"]), float(belt["length"])
    _, _, z_top, yaw = layout.belt_frame(belt)
    beam_offset = float(LaunchConfiguration("beam_offset").perform(context))
    beam_height = float(LaunchConfiguration("beam_height").perform(context))
    use_sim_time = LaunchConfiguration("sim_gazebo").perform(context) == "true"

    # The robot-side end is the belt's -X end (boxes are spawned at +X by
    # box_factory and travel towards the robot). Emitter 1 cm inside the
    # left rail, rays pointing across to the right rail.
    along = -length / 2.0 + beam_offset
    across = width / 2.0 - 0.01
    x, y = layout.belt_point(belt, along, across)
    sensor_yaw = yaw - math.pi / 2.0
    topic = f"/{ns}/line/beam"

    sdf = _SENSOR_SDF.format(name="infeed_beam_sensor", topic=topic,
                             rate=LaunchConfiguration("beam_rate").perform(context),
                             max_range=f"{width + 0.2:.3f}")
    spawn_sensor = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-string", sdf, "-name", "infeed_beam_sensor",
                   "-x", f"{x:.4f}", "-y", f"{y:.4f}", "-z", f"{z_top + beam_height:.4f}",
                   "-Y", f"{sensor_yaw:.4f}"],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="infeed_beam_bridge",
        output="screen",
        arguments=[f"{topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    controller = Node(
        package="workcell_bringup",
        executable="infeed_line_controller",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            # Opposite rail is `width` away; anything clearly closer is a box.
            "beam_clear_range": width - 0.06,
            "running_speed": float(LaunchConfiguration("running_speed").perform(context)),
            "start_enabled": LaunchConfiguration("start_enabled").perform(context) == "true",
        }],
    )
    return [spawn_sensor, bridge, controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("conveyor", default_value="conveyor_package_infeed",
                              description="Namespace (layout.py name) of the feeding belt"),
        DeclareLaunchArgument("beam_offset", default_value="1.60",
                              description="Beam distance from the robot-side belt end, m - where boxes stop for picking"),
        DeclareLaunchArgument("beam_height", default_value="0.04",
                              description="Beam height above the conveying surface, m (below the smallest box)"),
        DeclareLaunchArgument("beam_rate", default_value="20", description="Beam scan rate, Hz"),
        DeclareLaunchArgument("running_speed", default_value="6.0",
                              description="Belt speed (rad/s) until the first target_speed request, e.g. from the teleop slider"),
        DeclareLaunchArgument("start_enabled", default_value="true", description="Start with the line switched on"),
        DeclareLaunchArgument("sim_gazebo", default_value="true", description="Use sim time"),
        OpaqueFunction(function=launch_setup),
    ])
