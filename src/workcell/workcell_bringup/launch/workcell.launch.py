import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace


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

    # 7. Ρομπότ στην Κυψέλη Εργασίας
    # Single arm (Yaskawa GP70L), mounted on top of the robot_pedestal static
    # prop (see workcell_description/worlds/workcell_world.sdf and models/
    # robot_pedestal) - z=0.4 matches the pedestal's own height (shorter than
    # the old UR10-era pedestal: GP70L's own base+joint_1 already adds 0.54m
    # of height before the arm starts moving - see group_a_description/urdf/
    # group_a_macro.xacro). This workcell is now a specific zero-shot
    # pick/place cell (arm + conveyor fleet below), not a generic multi-arm
    # layout - add more entries here (each with its own pedestal + conveyor
    # cell) to scale to a real fleet.
    robots_config = [
        {"name": "robot_1", "xyz": "0.0 0.0 0.4", "rpy": "0.0 0.0 0.0"},
    ]

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

    # 9. Conveyor fleet around the pedestal (see conveyor_description/
    # conveyor_bringup - each is its own namespaced instance, same
    # description/bringup-pair pattern as group_a and the sibling
    # internal_delivery_system_ws AGV fleet).
    #
    # Layout (world frame, robot_1 base at the origin on top of the
    # pedestal): xyz below is each belt's own footprint CENTER, at floor
    # level (see conveyor_description/urdf/conveyor_macro.xacro's origin
    # convention - it also only frames the two ends with thin legs, not a
    # solid wall the full length, precisely so a crossing belt's own legs
    # have a clear gap to land in).
    #
    # A first pass here had the 3 pallet lanes flush against each other
    # (shared y boundaries, zero gap) and the crossing belt's clearance from
    # pallet_mid's own leg down to ~0.025m - both individually "correct" on
    # paper but together read as one overlapping mess in Gazebo. Real gaps
    # everywhere below.
    #
    # Rescaled by ~1.6x (GP70L's ~2.05m reach vs. the earlier UR10's ~1.3m)
    # from the original UR10-era layout when the arm was swapped for a
    # Yaskawa GP70L - lane/gap DISTANCES scaled, belt WIDTHS left as-is
    # (pallet/package size is independent of which arm is servicing them).
    # pallet heights (0.45) also left as-is (floor-clearance choice, not
    # reach-related). Still only worked out analytically (see the
    # verification script this was computed with), not simulation-verified -
    # recheck in Gazebo.
    #
    # package_infeed is the ONE exception: its xyz/length are pinned by
    # direct instruction, NOT part of this rescale - see its own entry below.
    #
    #   y
    #   ^  pallet_left  (long,  1.0 x 5.76, z-top 0.45)   y in [0.8, 1.8]
    #   |  pallet_mid   (short, 0.8 x 4.96, z-top 0.45)   y in [-0.4, 0.4],
    #   |                 starts at x=0.80 (0.24m clear of the pedestal's
    #   |                 x=0.56 edge) - the pedestal blocks the lane near
    #   |                 the robot, so this middle lane is shorter and
    #   |                 starts further out than the two long side lanes,
    #   |                 flush with them at the far end (x=5.76) - "the 2nd
    #   |                 is shorter and in the edge of the 2 big ones".
    #   |  pallet_right (long,  1.0 x 5.76, z-top 0.45)   y in [-1.8, -0.8]
    #   +------------------------------------------------------------> x
    #  (pedestal + robot_1 base, centered at the origin)
    #
    # package_infeed (0.5 x 4.0, z-top 0.4, rotated 90 deg - "length" runs
    # along Y) is fixed at xyz=(-0.75, 1.5, 0), NOT auto-rescaled - footprint
    # works out to x:[-1.0,-0.5], y:[-0.5,3.5]. It no longer crosses
    # pallet_mid at all (pallet_mid's own x-span is [0.8,5.76], entirely
    # positive-x; package_infeed sits on the negative-x side of the pedestal
    # instead). It DOES have a small bounding-box overlap with the pedestal's
    # own corner (pedestal x:[-0.56,0.56] y:[-0.56,0.56]) - roughly a
    # 0.06m x 1.06m sliver where package_infeed's near-end leg lands inside
    # the pedestal's footprint. Left as-is per instruction; flagging it here
    # rather than silently fixing it - check in Gazebo, nudge by hand if it's
    # a real collision and not just close.
    # side_rail_height: guide-rail height ABOVE the conveying surface (see
    # conveyor_description/urdf/conveyor_macro.xacro). Low on the 3 pallet
    # belts - a low rail still guides pallets in transit but stays well below
    # a pallet's own height, so the robot can still lift one off from above
    # without the rail being in the way. Tall on package_infeed - it's
    # carrying loose, unpalletized items with nothing else holding them in
    # place, so it needs actual containment walls.
    # controllers_spawn_delay: staggered 0.5s apart across the 4 belts, and
    # offset from robot_1's own fixed 4.0s (see group_a_bringup/launch/
    # bringup.launch.py) - all 5 controller_managers used to fire their
    # switch_controller call at the exact same instant (every bringup.
    # launch.py hardcoded the same 4.0s delay), which was real startup
    # contention on a machine without GPU passthrough into the container
    # (CPU rendering fallback). See conveyor_bringup/launch/bringup.launch.py
    # and group_a_bringup/launch/bringup.launch.py's own comments on this -
    # also paired there with a raised --switch-timeout as a second line of
    # defense for whatever contention staggering doesn't fully avoid.
    conveyors_config = [
        {"name": "conveyor_pallet_left", "width": "1.1", "length": "5.76", "height": "0.45", "xyz": "2.88 1.3 0.0", "rpy": "0.0 0.0 0.0", "side_rail_height": "0.01", "controllers_spawn_delay": "4.5"},
        {"name": "conveyor_pallet_right", "width": "1.1", "length": "5.76", "height": "0.45", "xyz": "2.88 -1.3 0.0", "rpy": "0.0 0.0 0.0", "side_rail_height": "0.01", "controllers_spawn_delay": "5.0"},
        {"name": "conveyor_pallet_mid", "width": "1.1", "length": "4.96", "height": "0.45", "xyz": "3.28 0.0 0.0", "rpy": "0.0 0.0 0.0", "side_rail_height": "0.01", "controllers_spawn_delay": "5.5"},
        # xyz/length here are intentionally NOT auto-rescaled with the rest of
        # this layout - left exactly as manually placed, per direct
        # instruction. If you reposition pallet_mid/the pedestal later, this
        # one won't automatically stay clear of them - recheck by hand.
        {"name": "conveyor_package_infeed", "width": "0.5", "length": "4.0", "height": "0.4", "xyz": "-1.00 1.5 0.0", "rpy": "0.0 0.0 1.5708", "side_rail_height": "0.12", "controllers_spawn_delay": "6.0"},
    ]

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
