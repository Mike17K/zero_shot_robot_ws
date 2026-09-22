from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Defaults mirror workcell_bringup/launch/workcell.launch.py's own
    # robots_config/conveyors_config namespace lists - keep these two files
    # in sync by hand if the workcell layout changes (entity added/removed/
    # renamed there).
    robot_namespaces_arg = DeclareLaunchArgument(
        "robot_namespaces",
        default_value="['robot_1']",
        description="YAML list of robot namespaces to show joint sliders for",
    )
    robot_joint_names_arg = DeclareLaunchArgument(
        "robot_joint_names",
        default_value="['gp_joint_1','gp_joint_2','gp_joint_3','gp_joint_4','gp_joint_5','gp_joint_6']",
        description="YAML list of joint names, in trajectory order",
    )
    joint_limits_package_arg = DeclareLaunchArgument(
        "joint_limits_package",
        default_value="group_a_moveit_config",
        description="Package whose config/joint_limits.yaml supplies slider min/max (radians)",
    )
    conveyor_namespaces_arg = DeclareLaunchArgument(
        "conveyor_namespaces",
        default_value="['conveyor_pallet_left','conveyor_pallet_right','conveyor_pallet_mid','conveyor_package_infeed']",
        description="YAML list of conveyor namespaces to show speed sliders for",
    )
    conveyor_max_speed_arg = DeclareLaunchArgument(
        "conveyor_max_speed",
        default_value="10.0",
        description="Each conveyor slider's range is +/- this, rad/s",
    )

    teleop_node = Node(
        package="workcell_teleop",
        executable="teleop_gui",
        output="screen",
        parameters=[{
            "robot_namespaces": LaunchConfiguration("robot_namespaces"),
            "robot_joint_names": LaunchConfiguration("robot_joint_names"),
            "joint_limits_package": LaunchConfiguration("joint_limits_package"),
            "conveyor_namespaces": LaunchConfiguration("conveyor_namespaces"),
            "conveyor_max_speed": LaunchConfiguration("conveyor_max_speed"),
        }],
    )

    return LaunchDescription([
        robot_namespaces_arg,
        robot_joint_names_arg,
        joint_limits_package_arg,
        conveyor_namespaces_arg,
        conveyor_max_speed_arg,
        teleop_node,
    ])
