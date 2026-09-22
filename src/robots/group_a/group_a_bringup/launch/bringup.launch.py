import os
from typing import Any, cast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use mock_components/GenericSystem (true) or real hardware drivers (false)"))
    args.append(DeclareLaunchArgument("sim_gazebo", default_value="false", description="Switch to true if launching inside a Gazebo Simulation environment"))
    args.append(DeclareLaunchArgument("parent_link", default_value="world", description="Parent link in the workcell"))
    args.append(DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0", description="Robot spawn position"))
    args.append(DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0", description="Robot spawn orientation"))
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Namespace for this robot's nodes and topics, including its own /<namespace>/tf"))
    # Suction footprint, in the gripper TCP frame - what gripper_manager.py
    # counts as "in front of the plate and close enough to suck up". Defaults
    # cover the 0.30x0.40m plate (group_a_macro.xacro) plus a small margin.
    # Nothing here caps HOW MANY objects can be grasped or spawned.
    args.append(DeclareLaunchArgument("suction_half_width", default_value="0.17", description="Half-width (X) of the suction footprint in the gripper TCP frame, meters"))
    args.append(DeclareLaunchArgument("suction_half_length", default_value="0.22", description="Half-length (Y) of the suction footprint in the gripper TCP frame, meters"))
    args.append(DeclareLaunchArgument("suction_reach", default_value="0.06", description="How far in front of the cups (+Z in the TCP frame) an object can be and still be sucked up, meters"))
    args.append(DeclareLaunchArgument("graspable_prefixes", default_value="['box']", description="YAML list of model-name prefixes gripper_manager treats as graspable - keeps the arm's own links and the conveyor rollers out of consideration"))
    args.append(DeclareLaunchArgument("world_name", default_value="default", description="Gazebo world name (see workcell_description/worlds/workcell_world.sdf's <world name=...>) - used by gripper_manager's spawn_box for the /world/<name>/create and /world/<name>/remove services"))
    return args


_param_file_refs: list[Any] = []


def _make_param_file(path, context):
    pf = ParameterFile(path, allow_substs=True)
    _param_file_refs.append(pf)  # prevent garbage collection / temp-file deletion
    return pf.evaluate(context)


def launch_setup(context):
    pkg_description = get_package_share_directory("group_a_description")
    pkg_moveit = get_package_share_directory("group_a_moveit_config")
    pkg_bringup = get_package_share_directory("group_a_bringup")

    # ── Runtime values ───────────────────────────────────────────────────────
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
    parent_link = LaunchConfiguration("parent_link").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    suction_half_width = LaunchConfiguration("suction_half_width").perform(context)
    suction_half_length = LaunchConfiguration("suction_half_length").perform(context)
    suction_reach = LaunchConfiguration("suction_reach").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)

    # ── Controllers YAML (namespace-substituted) ─────────────────────────────────
    # DUBUGGING TIP! we need to keep the parameter file in an instance! it creates the tmp file when we call evaluate() on it
    # if we don't keep the instance, it will be garbage collected and the tmp file will be deleted before the node can read it
    joint_limits_file_path = _make_param_file(os.path.join(pkg_moveit, "config", "joint_limits.yaml"), context)
    controllers_file_path = _make_param_file(os.path.join(pkg_bringup, "config", "controllers.yaml"), context)
    # sensors_3d_file_path = _make_param_file(os.path.join(pkg_bringup, "config", "sensors_3d.yaml"), context)
    moveit_controllers_file_path = _make_param_file(os.path.join(pkg_moveit, "config", "moveit_controllers.yaml"), context)

    robot_desc = (
        ParameterBuilder("group_a_description")
        .xacro_parameter(
            "robot_description",
            "urdf/group_a.urdf.xacro",
            mappings={
                "parent": parent_link,
                "xyz": xyz,
                "rpy": rpy,
                "sim_gazebo": sim_gazebo,
                "use_fake_hardware": use_fake_hardware,
                "simulation_controllers": str(controllers_file_path),
                "namespace": namespace,
            },
        )
        .to_dict()
    )

    gz_bridge_yaml_path = _make_param_file(os.path.join(pkg_bringup, "config", "gz_bridge.yaml"), context)

    sim_time_param = {"use_sim_time": LaunchConfiguration("sim_gazebo")}

    # tf2_ros hardcodes an absolute "/tf"/"/tf_static" internally, which a
    # Node's own `namespace=` does NOT touch (an already-absolute topic name
    # is never re-namespaced) - this explicit remap to the relative "tf"/
    # "tf_static" is what actually makes this robot's transforms land on its
    # own /<namespace>/tf instead of the global /tf. Applied to every node
    # here that publishes or looks up transforms (robot_state_publisher,
    # move_group) - same strategy as agv_bringup/launch/bringup.launch.py in
    # the sibling internal_delivery_system_ws workspace. controller_manager
    # deliberately does NOT get this remap: it never publishes tf itself
    # (joint_trajectory_controller doesn't broadcast odom tf the way
    # diff_drive_controller does), so there is nothing to redirect.
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    moveit_config = (
        MoveItConfigsBuilder(namespace, package_name="group_a_moveit_config")
        .robot_description(
            file_path=os.path.join(pkg_description, "urdf", "group_a.urdf.xacro"),
            mappings={
                "parent": parent_link,
                "xyz": xyz,
                "rpy": rpy,
                "sim_gazebo": sim_gazebo,
                "use_fake_hardware": use_fake_hardware,
                "simulation_controllers": str(controllers_file_path),
                "namespace": namespace,
            },
        )
        .robot_description_semantic(file_path=os.path.join(pkg_description, "config", "combined_system.srdf.xacro"))
        .robot_description_kinematics(os.path.join(pkg_moveit, "config", "kinematics.yaml"))
        .joint_limits(str(joint_limits_file_path))
        .trajectory_execution(str(controllers_file_path))
        .planning_scene_monitor(
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_planning_scene=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        # .sensors_3d(str(sensors_3d_file_path))
        .planning_pipelines("isaac_ros_cumotion", ["ompl", "isaac_ros_cumotion", "chomp", "stomp", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(os.path.join(pkg_moveit, "config", "pilz_cartesian_limits.yaml"))
        .to_moveit_configs()
        .to_dict()
    )

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_desc,
            sim_time_param,
        ],
        remappings=tf_remappings,
    )

    # ── 2. Standalone Controller Manager (real hardware only) ─────────────────
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_desc,
            controllers_file_path,
            sim_time_param,
        ],
        condition=UnlessCondition(LaunchConfiguration("sim_gazebo")),
        remappings=[("/robot_description", f"{namespace}/robot_description")],
    )

    # ── 3. Gazebo Spawner ────────────────────────────────────────────────────
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=namespace,
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            namespace,
        ],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    # ── 4. Camera Bridge ─────────────────────────────────────────────────────
    gz_default_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[sim_time_param],
        namespace=namespace,
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_yaml_path}"],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    # ── 4b. Depth-to-Pointcloud (Bypasses buggy Gazebo RGBD pointcloud) ──────
    # https://docs.ros.org/en/rolling/p/depth_image_proc/doc/components.html
    depth_to_pointcloud_container = ComposableNodeContainer(
        name="depth_to_pointcloud_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            # 1. Ευθυγράμμιση του Raw Depth με την RGB Κάμερα
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="depth_register_node",
                parameters=[sim_time_param],
                namespace=namespace,
                remappings=[
                    ("depth/image_rect", "camera/depth"),
                    ("depth/camera_info", "camera/camera_info"),
                    ("rgb/camera_info", "camera/camera_info"),
                    ("depth_registered/camera_info", "camera/depth_registered/camera_info"),
                    ("depth_registered/image_rect", "camera/depth_registered/image_rect"),
                ],
            ),
            # 2. Δημιουργία του XYZRGB Point Cloud από τα ευθυγραμμισμένα δεδομένα
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb_node",
                parameters=[sim_time_param],
                namespace=namespace,
                remappings=[
                    ("depth_registered/image_rect", "camera/depth_registered/image_rect"),
                    ("rgb/image_rect_color", "camera/color"),
                    ("rgb/camera_info", "camera/camera_info"),
                    ("points", "camera/depth_registered/points"),
                ],
            ),
        ],
    )

    # ── 5. Controller Spawners ───────────────────────────────────────────────
    motion_default_active_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "gp70l_joint_trajectory_controller",
            "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout",
            "30",
            # --controller-manager-timeout only bounds waiting for the
            # controller_manager SERVICE to exist - the actual activation
            # (switch_controller) has its own separate, shorter internal
            # wait (5s default in ros2_control's spawner). workcell.launch.py
            # starts this robot and all 4 conveyors together, each with its
            # own controller_manager racing to activate around the same
            # moment - on a machine falling back to CPU rendering (no GPU
            # passthrough into the container), that's enough contention to
            # occasionally blow through the 5s default and kill the spawner
            # outright. --switch-timeout is spawner's own documented knob for
            # exactly this ("switching cannot be performed immediately, e.g.
            # paused simulations at startup") - it waits instead of dying.
            "--switch-timeout",
            "20",
        ],
        parameters=[sim_time_param],
    )

    # ── 6. MoveIt move_group ─────────────────────────────────────────────────
    # Parameter loading order matters: last entry wins on key conflicts.
    #   - planning_params (dict, scalars only) → overrides any autogen pipeline keys
    #   - sensors_tmp_path (file path string)  → ROS 2 reads list params from file
    #   - octomap scalars dict                 → simple key/value, safe as dict

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=namespace,
        parameters=[
            moveit_config,
            moveit_controllers_file_path,
            sim_time_param,
            {
                "octomap_frame": "world",
                "octomap_resolution": 0.05,
                "max_range": 3.0,
                "workspace_bounds": {
                    "min_x": -5.0,
                    "min_y": -5.0,
                    "min_z": -2.0,
                    "max_x": 5.0,
                    "max_y": 5.0,
                    "max_z": 5.0,
                },
            },
        ],
        remappings=[
            ("/robot_description", f"{namespace}/robot_description"),
            ("/robot_description_semantic", f"{namespace}/robot_description_semantic"),
            *tf_remappings,
        ],
    )

    # ── 7. Gripper manager ───────────────────────────────────────────────────
    # Decides what this arm grasps and makes Gazebo hold it, by installing a
    # DetachableJoint per object at RUNTIME (Gazebo's entity/system/add
    # service) instead of anything being declared in the URDF - which is what
    # makes the object count unbounded. See scripts/gripper_manager.py's own
    # docstring. Gazebo-only: it talks to gz services that don't exist
    # outside sim. robot_model_name is left empty so the node falls back to
    # its own namespace, which is exactly the name workcell.launch.py spawns
    # this robot's Gazebo model as.
    gripper_manager_node = Node(
        package="group_a_bringup",
        executable="gripper_manager.py",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                # These are declared in gripper_manager.py with Python
                # float/list defaults, which fixes each one's ROS parameter
                # TYPE. LaunchConfiguration(...).perform(context) above always
                # returns a plain string, and handing a raw string straight
                # into this dict makes launch_ros set a STRING-typed override
                # - a real type mismatch (InvalidParameterTypeException),
                # confirmed against a live run, not a hypothetical. Cast to
                # the node's actual expected type here.
                "world_name": world_name,
                "suction_half_width": float(suction_half_width),
                "suction_half_length": float(suction_half_length),
                "suction_reach": float(suction_reach),
                # Already a YAML list literal, so let launch_ros resolve and
                # type it rather than .perform()-ing it into a string.
                "graspable_prefixes": LaunchConfiguration("graspable_prefixes"),
            },
            sim_time_param,
        ],
        remappings=tf_remappings,
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        gazebo_spawn_robot,
        gz_default_bridge,
        depth_to_pointcloud_container,
        move_group_node,
        gripper_manager_node,
        TimerAction(
            period=4.0,
            actions=[motion_default_active_controllers_spawner],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )
