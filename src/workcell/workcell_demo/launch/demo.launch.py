"""Workcell demo: the navigator action server + the pick and place orchestrator.

  ros2 launch workcell_demo demo.launch.py

Start it once the workcell, the cuMotion planner and the feeding line
(infeed_line.launch.py) are up. Parameters: config/pick_place.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("namespace", default_value="robot_1"),
        DeclareLaunchArgument("start_navigator", default_value="true",
                              description="Also start navigator_cli's navigator_server"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            get_package_share_directory("workcell_demo"), "config", "pick_place.yaml")),
    ]
    navigator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("navigator_cli"), "launch", "navigator_server.launch.py")),
        launch_arguments={"namespace": LaunchConfiguration("namespace")}.items(),
        condition=IfCondition(LaunchConfiguration("start_navigator")),
    )
    demo = Node(
        package="workcell_demo",
        executable="pick_place_demo",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"),
                    {"robot_namespace": LaunchConfiguration("namespace")}],
    )
    return LaunchDescription(args + [navigator, demo])
