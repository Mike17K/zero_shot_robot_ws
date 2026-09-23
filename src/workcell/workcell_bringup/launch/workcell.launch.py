import math
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace

from workcell_bringup import layout


def generate_launch_description():
    ld = LaunchDescription()

    # 1. Εντοπισμός Πακέτων για το Gazebo Global Environment
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_gp70l_desc = get_package_share_directory("motoman_gp70l_support")
    pkg_workcell_bringup = get_package_share_directory("workcell_bringup")
    pkg_workcell_description = get_package_share_directory("workcell_description")

    # 2. Global Gazebo Resource Paths (GZ_SIM_RESOURCE_PATH)
    # workcell_description/models is a separate entry (not just the package's
    # own share dir): model:// URIs (robot_pedestal, ...) resolve by scanning
    # GZ_SIM_RESOURCE_PATH for a directory that itself contains a matching
    # model.config - i.e. the *models* directory, not workcell_description's
    # share dir one level up. Same reasoning for pkg_gp70l_desc: its meshes
    # are package://motoman_gp70l_support/meshes/... in the xacro, which
    # sdformat rewrites to model://motoman_gp70l_support/... - resolved by
    # scanning for a directory named motoman_gp70l_support, i.e. one level
    # above that package's own share dir (motoman_resources has no meshes,
    # so it doesn't need an entry here).
    gz_resource_paths = (
        os.path.dirname(pkg_gp70l_desc) + ":"
        + os.path.dirname(pkg_workcell_bringup) + ":"
        + os.path.dirname(pkg_workcell_description) + ":"
        + os.path.join(pkg_workcell_description, "models")
    )
    set_gz_resource_path = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_paths)
    ld.add_action(set_gz_resource_path)

    # 3. Global Launch Arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="True for mock components (RViz only). False for Gazebo or Real Hardware.",
    )

    sim_gazebo_arg = DeclareLaunchArgument(
        "sim_gazebo",
        default_value="false",
        description="True to launch Gazebo Simulator.",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_workcell_description, "worlds", "workcell_world.sdf"]),
        description="Gazebo world file to load",
    )

    # 4. Εκκίνηση Global Gazebo Instance
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),  # trailing space
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    ld.add_action(gazebo)

    # 5. Global Clock Bridge (ROS 2 <-> Gazebo time synchronization)
    bridge_params = os.path.join(pkg_workcell_bringup, "config", "gz_bridge.yaml")
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    ld.add_action(clock_bridge)

    # 6. Global Static Transform Publisher για το World Frame
    world_node = Node(package="tf2_ros", executable="static_transform_publisher", arguments=["0", "0", "0", "0", "0", "0", "world", "map"])
    ld.add_action(world_node)

    # 7. Robots - layout (and its rationale) in workcell_bringup/layout.py.
    robots_config = layout.ROBOTS

    pkg_group_a_bringup_share = get_package_share_directory("group_a_bringup")
    group_a_launch_path = os.path.join(pkg_group_a_bringup_share, "launch", "bringup.launch.py")

    # 8. Ένα GroupAction/IncludeLaunchDescription ανά ρομπότ (still built per-
    # entry here; WHEN they actually start is decided once, below, together
    # with the conveyors).
    all_stacks = []
    for robot in robots_config:
        all_stacks.append(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(group_a_launch_path),
                        launch_arguments={
                            "parent_link": "world",
                            "xyz": robot["xyz"],
                            "rpy": robot["rpy"],
                            "sim_gazebo": LaunchConfiguration("sim_gazebo"),
                            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                            "namespace": robot["name"],
                            "world_name": "default",
                        }.items(),
                    ),
                ]
            )
        )

    # 9. Conveyor fleet around the pedestal - layout (and its rationale) in
    # workcell_bringup/layout.py. Each belt is its own namespaced instance of
    # conveyor_description / conveyor_bringup.
    conveyors_config = layout.CONVEYORS

    pkg_conveyor_bringup_share = get_package_share_directory("conveyor_bringup")
    conveyor_launch_path = os.path.join(pkg_conveyor_bringup_share, "launch", "bringup.launch.py")

    for conveyor in conveyors_config:
        all_stacks.append(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(conveyor_launch_path),
                        launch_arguments={
                            "parent_link": "world",
                            "xyz": conveyor["xyz"],
                            "rpy": conveyor["rpy"],
                            "width": conveyor["width"],
                            "length": conveyor["length"],
                            "height": conveyor["height"],
                            # Optional per-instance overrides - roller count is
                            # derived from length/roller_radius/roller_gap (see
                            # conveyor_bringup/launch/bringup.launch.py's
                            # _num_rollers), so a long belt at the 0.05/0.005
                            # defaults below can end up with 30+ driven joints;
                            # bump roller_radius here (fewer, bigger rollers) per
                            # belt if that's too slow in Gazebo.
                            "roller_radius": conveyor.get("roller_radius", "0.05"),
                            "roller_gap": conveyor.get("roller_gap", "0.005"),
                            "side_rail_height": conveyor.get("side_rail_height", "0.03"),
                            "controllers_spawn_delay": conveyor.get("controllers_spawn_delay", "4.0"),
                            "sim_gazebo": LaunchConfiguration("sim_gazebo"),
                            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                            "namespace": conveyor["name"],
                        }.items(),
                    ),
                ]
            )
        )

    # 9b. Random box factory on package_infeed (see conveyor_bringup/scripts/
    # box_factory.py) - starts disabled, switched on from workcell_teleop's
    # Box Factory panel. The spawn area is derived from the infeed entry
    # above: its center line, 0.3m in from the far end (the end away from the
    # robot, +local X after its 90deg yaw - same point as teleop's Spawn Box
    # default), on top of its conveying surface.
    infeed = next(c for c in conveyors_config if c["name"] == "conveyor_package_infeed")
    infeed_x, infeed_y, _ = (float(v) for v in infeed["xyz"].split())
    infeed_yaw = float(infeed["rpy"].split()[2])
    infeed_far = float(infeed["length"]) / 2.0 - 0.3
    box_factory = Node(
        package="conveyor_bringup",
        executable="box_factory.py",
        name="box_factory",
        output="screen",
        parameters=[{
            "world_name": "default",
            "spawn_service": f"/{robots_config[0]['name']}/gripper/spawn_box",
            "spawn_x": infeed_x + infeed_far * math.cos(infeed_yaw),
            "spawn_y": infeed_y + infeed_far * math.sin(infeed_yaw),
            "lane_yaw": infeed_yaw,
            "lane_width": float(infeed["width"]),
            "belt_top_z": float(infeed["height"]),
        }],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    all_stacks.append(box_factory)

    # 10. Spawn everything together, in parallel. Previously this staggered
    # every entity by a growing i*0.5s (0, 0.5, 1.0, ...) specifically to
    # avoid literally simultaneous `ros_gz_sim create` requests - real for a
    # handful of entities, but it serializes startup and gets worse the more
    # robots/belts are added, which is the opposite of what a scalable fleet
    # wants. Each entity's bringup pipeline is self-contained and per-
    # namespace (own controller_manager, own spawner, own internal
    # TimerActions for its controller/bootstrap sequencing - see
    # group_a_bringup's and conveyor_bringup's own bringup.launch.py), so
    # there's no real ordering dependency BETWEEN entities - only each one's
    # own internal steps need to happen in order, which they still do. One
    # shared, fixed delay (not growing with fleet size) gives Gazebo itself a
    # moment to finish coming up, then launches every entity at once.
    ld.add_action(TimerAction(period=1.0, actions=all_stacks))

    return LaunchDescription([use_fake_hardware_arg, sim_gazebo_arg, world_arg, ld])
