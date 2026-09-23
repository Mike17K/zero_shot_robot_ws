"""One box factory instance, same structure as group_a_bringup /
conveyor_bringup: description -> robot_state_publisher (own /<namespace>/tf)
-> Gazebo spawn, plus the factory's operating node.

xyz / rpy place the factory OUTLET on the conveying surface of the belt it
feeds (yaw = the belt's travel direction); workcell.launch.py derives them
from workcell_bringup/layout.py.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument("namespace", default_value="box_factory", description="Namespace for this factory's nodes and topics, and its Gazebo model name"),
        DeclareLaunchArgument("parent_link", default_value="world"),
        DeclareLaunchArgument("xyz", default_value="-1.0 3.2 0.4", description="Outlet position (x y z) - on the conveying surface"),
        DeclareLaunchArgument("rpy", default_value="0.0 0.0 1.5708", description="Outlet orientation - yaw = the fed belt's travel direction"),
        DeclareLaunchArgument("lane_width", default_value="0.5", description="Width of the belt between its rails, meters"),
        DeclareLaunchArgument("clearance", default_value="0.45", description="Chute height above the belt, meters (above the tallest box)"),
        DeclareLaunchArgument("sim_gazebo", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            get_package_share_directory("box_factory_bringup"), "config", "box_factory.yaml")),
    ]


def launch_setup(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)
    lane_width = float(LaunchConfiguration("lane_width").perform(context))
    clearance = LaunchConfiguration("clearance").perform(context)
    sim_time = {"use_sim_time": LaunchConfiguration("sim_gazebo").perform(context) == "true"}
    x, y, z = (float(v) for v in xyz.split())
    yaw = float(rpy.split()[2])

    robot_desc = (
        ParameterBuilder("box_factory_description")
        .xacro_parameter(
            "robot_description",
            "urdf/box_factory.urdf.xacro",
            mappings={
                "parent": LaunchConfiguration("parent_link").perform(context),
                "xyz": xyz,
                "rpy": rpy,
                "chute_width": f"{lane_width - 0.04:.3f}",
                "clearance": clearance,
                "namespace": namespace,
            },
        )
        .to_dict()
    )
    # Own /<namespace>/tf, like the robot and the conveyors.
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_desc, sim_time],
        remappings=tf_remappings,
    )
    gazebo_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=namespace,
        arguments=["-topic", "robot_description", "-name", namespace],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    factory = Node(
        package="box_factory_bringup",
        executable="box_factory_node.py",
        name="factory",
        namespace=namespace,
        output="screen",
        parameters=[
            LaunchConfiguration("params_file").perform(context),
            {
                "spawn_x": x,
                "spawn_y": y,
                "belt_top_z": z,
                "lane_yaw": math.atan2(math.sin(yaw), math.cos(yaw)),
                "lane_width": lane_width,
            },
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    return [robot_state_publisher, gazebo_spawn, factory]


def generate_launch_description():
    return LaunchDescription([*get_launch_arguments(), OpaqueFunction(function=launch_setup)])
