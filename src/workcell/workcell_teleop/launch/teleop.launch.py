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

    # Gripper/Spawn Box logic (which objects are graspable, the per-object
    # runtime joint, gz spawning) lives entirely in group_a_bringup's
    # gripper_manager.py node, one instance per robot - this UI is just a
    # service client (see teleop_gui.py's module docstring), so none of that
    # needs configuring here.
    # Default spawn point (also editable live from the Spawn Box panel's own
    # X/Y/Z spinboxes - these are just the seed values): on top of
    # conveyor_package_infeed's footprint (xyz="-1.00 1.5 0.0", yaw=90deg,
    # length=4.0 in workcell_bringup/launch/workcell.launch.py's
    # conveyors_config), at the belt's far end - away from the robot/
    # pedestal at world origin. After the belt's 90deg yaw, its local travel
    # axis (+X) maps to world_y = spawn_y(1.5) + local_x, so larger local_x
    # (up to ~1.93 at the last roller) is larger world_y, farther from the
    # robot at y=0; Y=3.2 sits near that far end with a safety margin off
    # the very edge. X stays centered on the belt's width (matches the
    # belt's own spawn_x, -1.00). Z is a short drop above its z-top=0.4
    # conveying surface. Keep in sync with that file by hand.
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="-1.00", description="Spawn Box default X position, meters")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="3.2", description="Spawn Box default Y position, meters")
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.55", description="Spawn Box default Z position, meters (above conveyor_package_infeed's z-top=0.4, for a short drop)")

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
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
        }],
    )

    return LaunchDescription([
        robot_namespaces_arg,
        robot_joint_names_arg,
        joint_limits_package_arg,
        conveyor_namespaces_arg,
        conveyor_max_speed_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        teleop_node,
    ])
