import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use mock_components/GenericSystem (true) or real hardware drivers (false)"))
    args.append(DeclareLaunchArgument("sim_gazebo", default_value="false", description="Switch to true if launching inside a Gazebo Simulation environment"))
    args.append(DeclareLaunchArgument("parent_link", default_value="world", description="Parent link in the workcell"))
    args.append(DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0", description="Belt spawn position (x y z, space-separated) - center of footprint, at floor level"))
    args.append(DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0", description="Belt spawn orientation (roll pitch yaw, space-separated)"))
    args.append(DeclareLaunchArgument("width", default_value="1.0", description="Belt width (across the travel direction), meters"))
    args.append(DeclareLaunchArgument("length", default_value="2.0", description="Belt length (along the travel direction), meters"))
    args.append(DeclareLaunchArgument("height", default_value="0.45", description="Conveying-surface height off the floor, meters"))
    args.append(DeclareLaunchArgument("roller_radius", default_value="0.05", description="Roller cylinder radius, meters"))
    args.append(DeclareLaunchArgument("roller_gap", default_value="0.005", description="Target clearance between adjacent roller SURFACES (not centers), meters"))
    args.append(DeclareLaunchArgument("side_rail_height", default_value="0.03", description="Side guide-rail height ABOVE the conveying surface, meters - keep low on pallet belts (so the robot can still lift a pallet off from above), tall on a loose-item infeed belt (so items don't fall off the sides)"))
    args.append(DeclareLaunchArgument("belt_speed", default_value="6.0", description="Target roller angular velocity, rad/s (bootstrap-published once the belt controller is active)"))
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Namespace for this belt's nodes and topics, including its own /<namespace>/tf"))
    args.append(DeclareLaunchArgument("speed_topic", default_value="target_speed", description="Topic (under the namespace) belt_speed_relay takes std_msgs/Float64 speed commands from - set it to something else (e.g. line_speed) when a controller sits between speed requests and the belt"))
    # workcell.launch.py starts every conveyor (and the robot) together, and
    # each instance's own controller spawner used to fire at the same fixed
    # 4.0s delay - meaning all of them called switch_controller on their
    # respective controller_managers at the exact same instant, which is real
    # contention (see controllers_spawner's own comment below). Exposed here
    # so workcell.launch.py can stagger each conveyor instance's spawn time
    # instead of them all colliding.
    args.append(DeclareLaunchArgument("controllers_spawn_delay", default_value="4.0", description="Seconds after this belt's own nodes start before its controller spawner fires - stagger this per-instance from the caller to avoid every belt (and the robot) hitting switch_controller at the same moment"))
    return args


def _num_rollers(length: float, roller_radius: float, roller_gap: float) -> int:
    """Roller count from belt length/roller size/target gap - MUST match
    conveyor_description/urdf/conveyor_macro.xacro's num_rollers property
    formula exactly (same operand order, same float arithmetic), since this
    is what generates the roller_1_joint..roller_N_joint names the
    dynamically-built controller config below has to list. If you change the
    formula, change it in both places."""
    end_margin = roller_radius + 0.02
    roller_pitch = 2 * roller_radius + roller_gap
    available_span = length - 2 * end_margin
    return max(2, int(available_span / roller_pitch) + 1)


def _make_temp_file(contents: str) -> str:
    temp_file = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".yaml")
    temp_file.write(contents)
    temp_file.close()
    return temp_file.name


def _controllers_yaml(namespace: str, joint_names: list[str]) -> str:
    # Roller count (hence this joint list) varies per belt instance - a
    # static config/controllers.yaml file can't list a fixed set of joint
    # names that's simultaneously right for every instance, so this whole
    # file is generated here instead, once per launch, with the joint count
    # this specific instance's width/length/height/roller_radius/roller_gap
    # actually produced. It's used both as `simulation_controllers` (read by
    # gz_ros2_control's plugin block in the URDF) and as the parameter file
    # passed to the real-hardware controller_manager_node/spawner below -
    # both need the SAME generated file, since gz_ros2_control loads
    # controller params straight from a file path, not from any Python
    # object this launch file could hand it directly.
    data = {
        f"/{namespace}": {
            "controller_manager": {
                "ros__parameters": {
                    "update_rate": 100,
                    "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
                    "belt_velocity_controller": {"type": "forward_command_controller/ForwardCommandController"},
                }
            },
            "belt_velocity_controller": {
                "ros__parameters": {
                    "joints": joint_names,
                    "interface_name": "velocity",
                }
            },
        }
    }
    return yaml.safe_dump(data, sort_keys=False)


def launch_setup(context):
    # ── Runtime values ───────────────────────────────────────────────────────
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
    parent_link = LaunchConfiguration("parent_link").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)
    width = LaunchConfiguration("width").perform(context)
    length = LaunchConfiguration("length").perform(context)
    height = LaunchConfiguration("height").perform(context)
    roller_radius = LaunchConfiguration("roller_radius").perform(context)
    roller_gap = LaunchConfiguration("roller_gap").perform(context)
    side_rail_height = LaunchConfiguration("side_rail_height").perform(context)
    belt_speed = LaunchConfiguration("belt_speed").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    controllers_spawn_delay = float(LaunchConfiguration("controllers_spawn_delay").perform(context))

    num_rollers = _num_rollers(float(length), float(roller_radius), float(roller_gap))
    joint_names = [f"roller_{i}_joint" for i in range(1, num_rollers + 1)]
    controllers_file_path = _make_temp_file(_controllers_yaml(namespace, joint_names))

    robot_desc = (
        ParameterBuilder("conveyor_description")
        .xacro_parameter(
            "robot_description",
            "urdf/conveyor.urdf.xacro",
            mappings={
                "parent": parent_link,
                "xyz": xyz,
                "rpy": rpy,
                "width": width,
                "length": length,
                "height": height,
                "roller_radius": roller_radius,
                "roller_gap": roller_gap,
                "side_rail_height": side_rail_height,
                "sim_gazebo": sim_gazebo,
                "use_fake_hardware": use_fake_hardware,
                "simulation_controllers": controllers_file_path,
                "namespace": namespace,
            },
        )
        .to_dict()
    )

    sim_time_param = {"use_sim_time": LaunchConfiguration("sim_gazebo")}

    # tf2_ros hardcodes an absolute "/tf"/"/tf_static" internally, which a
    # Node's own `namespace=` does NOT touch - this explicit remap to the
    # relative "tf"/"tf_static" is what actually makes this belt's transforms
    # land on its own /<namespace>/tf instead of the global /tf. Same
    # strategy as group_a_bringup/launch/bringup.launch.py and
    # agv_bringup/launch/bringup.launch.py (sibling
    # internal_delivery_system_ws workspace).
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

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

    # ── 4. Controller Spawners ───────────────────────────────────────────────
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "belt_velocity_controller",
            "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout",
            "30",
            # See group_a_bringup/launch/bringup.launch.py's matching
            # comment: --controller-manager-timeout only bounds waiting for
            # the service to exist, not the actual activation. On a machine
            # falling back to CPU rendering (no GPU passthrough into the
            # container), 5 controller_managers (this belt + the other 3 +
            # the robot) all racing to activate around the same moment can
            # occasionally blow through ros2_control's 5s default
            # switch_controller wait and kill the spawner outright.
            # --switch-timeout is spawner's own documented knob for exactly
            # this ("switching cannot be performed immediately, e.g. paused
            # simulations at startup").
            "--switch-timeout",
            "20",
        ],
        parameters=[sim_time_param],
    )

    # ── 5. Belt speed relay ──────────────────────────────────────────────────
    # forward_command_controller/ForwardCommandController has no command
    # timeout (unlike diff_drive_controller) - it holds the last velocity it
    # received indefinitely, so publishing once is enough to start the belt
    # spinning and keep it spinning. belt_velocity_controller/commands needs
    # one value per roller (num_rollers, not a fixed count), which nothing
    # outside this launch file knows - belt_speed_relay is a small persistent
    # node (not a one-shot `ros2 topic pub`) that's told num_rollers here and
    # fans a plain std_msgs/Float64 on ~/target_speed out to the full-size
    # array, so anything - a teleop UI (see workcell_teleop), a test script -
    # can drive this belt without knowing its roller count. It also publishes
    # `belt_speed` itself once at startup, replacing what used to be a
    # separate one-shot bootstrap.
    belt_speed_relay = Node(
        package="conveyor_bringup",
        executable="belt_speed_relay.py",
        output="screen",
        namespace=namespace,
        parameters=[
            {"num_rollers": num_rollers, "initial_speed": float(belt_speed)},
            sim_time_param,
        ],
        remappings=[("target_speed", LaunchConfiguration("speed_topic").perform(context))],
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        gazebo_spawn_robot,
        TimerAction(
            period=controllers_spawn_delay,
            actions=[controllers_spawner],
        ),
        # Kept at the same +2.0s offset after the controller spawner this
        # always had (was hardcoded period=6.0 = old fixed 4.0 delay + 2.0) -
        # belt_speed_relay just needs to run after belt_velocity_controller
        # is active, regardless of what controllers_spawn_delay is set to.
        TimerAction(period=controllers_spawn_delay + 2.0, actions=[belt_speed_relay]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )
