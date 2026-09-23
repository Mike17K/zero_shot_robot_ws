"""Navigator action server (navigator_cli/navigator_server.py) for one robot.

  ros2 launch navigator_cli navigator_server.launch.py namespace:=robot_1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("namespace", default_value="robot_1"),
        DeclareLaunchArgument("tool_frame", default_value="gripper_tcp"),
        DeclareLaunchArgument("cartesian_speed", default_value="0.05", description="Default straight-line tool speed, m/s"),
    ]
    server = Node(
        package="navigator_cli",
        executable="navigator_server",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "namespace": LaunchConfiguration("namespace"),
            "tool_frame": LaunchConfiguration("tool_frame"),
            "cartesian_speed": LaunchConfiguration("cartesian_speed"),
        }],
    )
    return LaunchDescription(args + [server])
