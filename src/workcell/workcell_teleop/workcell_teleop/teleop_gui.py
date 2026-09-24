#!/usr/bin/env python3
"""PyQt teleop UI for the workcell.

Layout: a status strip on top (one coloured chip per subsystem: sim clock and
real-time factor, robot joint states, cuMotion planner, navigator server,
gripper, feeding line, box factory, demo), always visible, then one tab per
area - Robot (jog/gripper, demo run/pause/restart), Camera (live gripper-camera colour/depth feed,
<namespace>/camera/color|depth from group_a_bringup's camera_bridge), Line &
Belts (feeding line + conveyor speeds) and Boxes (factory + spawn). Panels in
a tab re-flow into columns with the window width. Controls are sized for
touch.

Robot: hold-to-jog - press and hold a joint's -/+ button to move it at the
selected speed (5-50% of its max velocity), release to stop. The joint
trajectory controller only takes positions, so while held a target just
ahead of the MEASURED position is streamed on
<namespace>/gp70l_joint_trajectory_controller/joint_trajectory (see JOG_*);
the position bars always show the measured joint states, so nothing ever
jumps back to stale slider values. Home moves all joints to 0 at 25% speed.

Also: a gripper ON/OFF toggle per robot (shows what is held), speed
sliders per conveyor (std_msgs/Float64 on <namespace>/target_speed, fanned out
by each belt's belt_speed_relay), and a "Spawn Box" button with adjustable
width/depth/height and X/Y/Z position that drops a box into the scene.

Robot namespaces/joint names and conveyor namespaces are ROS parameters
(set from launch/teleop.launch.py, defaulted here to match
workcell_bringup/launch/workcell.launch.py's current robots_config/
conveyors_config) rather than hardcoded twice - keep those two files' lists
in sync by hand if the workcell layout changes. Joint slider ranges are read
directly from group_a_moveit_config/config/joint_limits.yaml at startup
(also not duplicated here).

Gripper ON/OFF and Spawn Box: this UI is a thin client. All the actual
grasp/scene logic - deciding which objects are in front of the suction plate,
installing the per-object joint in Gazebo, spawning boxes - lives in
group_a_bringup's own gripper_manager.py node, one instance per robot,
launched alongside that robot's own bringup (see
group_a_bringup/launch/bringup.launch.py). This UI just calls its services
(gripper/set_suction, gripper/spawn_box) and shows whatever result comes
back; it knows nothing about objects, joints or Gazebo. Suction is a latching
state rather than a one-shot grab - ON keeps capturing whatever enters the
gripper's footprint, OFF releases everything - and the number of spawnable /
graspable objects is unbounded. See gripper_manager.py's own docstring.

Infeed Line: status and controls of workcell_bringup's infeed_line_controller
(laser beam in front of the robot stops the feeding belt while a box waits to
be picked). Mode, beam and speeds refresh live from its latched line/state;
the buttons call line/set_enabled and line/release (restart ignoring the box).

Box Factory: another thin client. The panel only sets ROS parameters on
box_factory_bringup's factory node (/box_factory/factory) and shows its live
/box_factory/status (enabled, period_sec, time_randomness,
shape_randomness, size_min, size_max - see box_factory_node.py's docstring), which
does all the random sampling and spawning itself, so the stream keeps running
even if this UI is closed. Every change pushes the whole panel state.
"""
import math
import os
import sys
import time
from typing import Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from custom_msgs.msg import BoxFactoryStatus, GripperState, InfeedLineState
from custom_msgs.srv import SpawnBox

MOVE_TIME_SEC = 0.3  # default time_from_start for a single-point joint command

STATUS_COLORS = {
    'ok': '#2e7d32',    # green  - running / healthy
    'busy': '#1565c0',  # blue   - working (moving, holding, released)
    'warn': '#ef6c00',  # orange - waiting / needs attention soon
    'bad': '#c62828',   # red    - stopped / error / missing
    'off': '#616161',   # grey   - switched off / not started
}
SERVICE_CALL_TIMEOUT_SEC = 5.0
SERVICE_WAIT_TIMEOUT_SEC = 2.0
# Default Spawn Box position/size seeds shown in the UI (freely editable per
# click) - on top of conveyor_package_infeed (xyz="-1.00 1.5 0.0", yaw=90deg,
# length=4.0 in workcell_bringup/launch/workcell.launch.py's
# conveyors_config), at the belt's far end - away from the robot/pedestal at
# world origin, not its center. After the belt's 90deg yaw, its local travel
# axis (+X) maps to world_y = spawn_y(1.5) + local_x, so larger local_x (up
# to ~1.93 at the last roller) is larger world_y, farther from the robot at
# y=0. Y=3.2 sits near that far end with a safety margin off the very edge.
# X stays centered on the belt's width (matches the belt's own spawn_x,
# -1.00). Z is a short drop above the belt's z-top=0.4 conveying surface.
# Keep in sync with workcell.launch.py by hand.
DEFAULT_SPAWN_X = -1.00
DEFAULT_SPAWN_Y = 3.2
DEFAULT_SPAWN_Z = 0.55


def _load_joint_limits(package_name: str, joint_names: list[str]) -> dict[str, tuple[float, float, float]]:
    """{joint: (min_position, max_position, max_velocity)} from joint_limits.yaml."""
    path = os.path.join(get_package_share_directory(package_name), 'config', 'joint_limits.yaml')
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    joint_limits = data.get('joint_limits', {})
    limits = {}
    for name in joint_names:
        entry = joint_limits.get(name, {})
        limits[name] = (float(entry.get('min_position', -math.pi)), float(entry.get('max_position', math.pi)),
                        float(entry.get('max_velocity', 1.0)))
    return limits


class TeleopNode(Node):
    def __init__(self):
        super().__init__('workcell_teleop')

        self.declare_parameter('robot_namespaces', ['robot_1'])
        self.declare_parameter(
            'robot_joint_names',
            ['gp_joint_1', 'gp_joint_2', 'gp_joint_3', 'gp_joint_4', 'gp_joint_5', 'gp_joint_6'],
        )
        self.declare_parameter('joint_limits_package', 'group_a_moveit_config')
        self.declare_parameter(
            'conveyor_namespaces',
            ['conveyor_pallet_left', 'conveyor_pallet_right', 'conveyor_pallet_mid', 'conveyor_package_infeed'],
        )
        self.declare_parameter('conveyor_max_speed', 10.0)
        self.declare_parameter('spawn_x', DEFAULT_SPAWN_X)
        self.declare_parameter('spawn_y', DEFAULT_SPAWN_Y)
        self.declare_parameter('spawn_z', DEFAULT_SPAWN_Z)
        # box_factory_bringup instance: its node is <namespace>/factory.
        self.declare_parameter('box_factory_namespace', 'box_factory')
        # Feeding line (workcell_bringup/launch/infeed_line.launch.py) - its
        # controller runs in this belt's namespace.
        self.declare_parameter('infeed_line_namespace', 'conveyor_package_infeed')

        self.robot_namespaces: list[str] = list(self.get_parameter('robot_namespaces').value)
        self.robot_joint_names: list[str] = list(self.get_parameter('robot_joint_names').value)
        joint_limits_package: str = self.get_parameter('joint_limits_package').value
        self.conveyor_namespaces: list[str] = list(self.get_parameter('conveyor_namespaces').value)
        self.conveyor_max_speed: float = float(self.get_parameter('conveyor_max_speed').value)
        self.spawn_x: float = float(self.get_parameter('spawn_x').value)
        self.spawn_y: float = float(self.get_parameter('spawn_y').value)
        self.spawn_z: float = float(self.get_parameter('spawn_z').value)

        factory_ns = self.get_parameter('box_factory_namespace').value
        self._factory_params = AsyncParameterClient(self, f'/{factory_ns}/factory')
        self.factory_status: Optional[BoxFactoryStatus] = None
        self.factory_status_stamp = 0.0
        self.create_subscription(
            BoxFactoryStatus, f'/{factory_ns}/status', self._on_factory_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.joint_limits = _load_joint_limits(joint_limits_package, self.robot_joint_names)

        self._traj_pubs = {
            ns: self.create_publisher(JointTrajectory, f'/{ns}/gp70l_joint_trajectory_controller/joint_trajectory', 10)
            for ns in self.robot_namespaces
        }
        self._speed_pubs = {
            ns: self.create_publisher(Float64, f'/{ns}/target_speed', 10)
            for ns in self.conveyor_namespaces
        }
        # One gripper_manager instance per robot (see that node's own
        # docstring) - these are plain service clients, this UI has no
        # knowledge of slots, TF, or box tracking at all anymore.
        self._suction_clients = {
            ns: self.create_client(SetBool, f'/{ns}/gripper/set_suction')
            for ns in self.robot_namespaces
        }
        self._spawn_box_clients = {
            ns: self.create_client(SpawnBox, f'/{ns}/gripper/spawn_box')
            for ns in self.robot_namespaces
        }
        # Latest JointState per robot, for the panels' Refresh button (depth 1:
        # only the newest message matters, and spin_once drains one callback
        # per Qt tick).
        self._joint_states: dict[str, JointState] = {}
        self.joint_state_stamp: dict[str, float] = {}
        self.gripper_states: dict[str, GripperState] = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for ns in self.robot_namespaces:
            self.create_subscription(JointState, f'/{ns}/joint_states',
                                     lambda msg, ns=ns: self._on_joint_state(ns, msg), 1)
            self.create_subscription(GripperState, f'/{ns}/gripper/state',
                                     lambda msg, ns=ns: self.gripper_states.__setitem__(ns, msg), latched)

        # Gripper camera (group_a_bringup's camera_bridge): only the latest
        # frame is kept; the Camera tab converts it to a QImage on its own
        # tick, and only while visible.
        self.declare_parameter('camera_color_topic', 'camera/color')
        self.declare_parameter('camera_depth_topic', 'camera/depth')
        self.camera_frames: dict[tuple[str, str], Image] = {}
        self.camera_stamps: dict[tuple[str, str], list[float]] = {}
        camera_topics = {'color': self.get_parameter('camera_color_topic').value,
                         'depth': self.get_parameter('camera_depth_topic').value}
        for ns in self.robot_namespaces:
            for kind, topic in camera_topics.items():
                self.create_subscription(Image, f'/{ns}/{topic}',
                                         lambda msg, key=(ns, kind): self._on_camera(key, msg),
                                         qos_profile_sensor_data)

        # Workcell status strip: sim clock (real-time factor), demo state and
        # which servers exist (polled from the ROS graph, 1 Hz).
        self.sim_rtf: Optional[float] = None
        self.clock_stamp = 0.0
        self._clock_window: list[tuple[float, float]] = []
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.demo_state: Optional[str] = None
        self.create_subscription(String, '/workcell_demo/state',
                                 lambda msg: setattr(self, 'demo_state', msg.data), latched)
        self.graph_services: set[str] = set()

        line_ns = self.get_parameter('infeed_line_namespace').value
        self.infeed_line_namespace = line_ns
        self.line_state: Optional[InfeedLineState] = None
        self.line_state_stamp = 0.0  # wall time.monotonic() of the last state
        self.create_subscription(
            InfeedLineState, f'/{line_ns}/line/state', self._on_line_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._line_enable_client = self.create_client(SetBool, f'/{line_ns}/line/set_enabled')
        self._line_release_client = self.create_client(Trigger, f'/{line_ns}/line/release')
        # Pick & place demo (workcell_demo) controls.
        self._demo_enable_client = self.create_client(SetBool, '/workcell_demo/set_enabled')
        self._demo_restart_client = self.create_client(Trigger, '/workcell_demo/restart')

    def _on_joint_state(self, ns: str, msg: JointState) -> None:
        self._joint_states[ns] = msg
        self.joint_state_stamp[ns] = time.monotonic()

    def _on_camera(self, key: tuple[str, str], msg: Image) -> None:
        self.camera_frames[key] = msg
        stamps = self.camera_stamps.setdefault(key, [])
        stamps.append(time.monotonic())
        del stamps[:-20]  # last 20 arrivals, for the rate readout

    def _on_clock(self, msg: Clock) -> None:
        now = time.monotonic()
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_stamp = now
        window = self._clock_window
        window.append((now, sim))
        while len(window) > 2 and now - window[0][0] > 3.0:
            window.pop(0)
        wall_dt = window[-1][0] - window[0][0]
        if wall_dt > 0.5:
            self.sim_rtf = (window[-1][1] - window[0][1]) / wall_dt

    def poll_graph(self) -> None:
        self.graph_services = {name for name, _ in self.get_service_names_and_types()}

    def has_service(self, name: str) -> bool:
        return name in self.graph_services

    def _on_factory_status(self, msg: BoxFactoryStatus) -> None:
        self.factory_status = msg
        self.factory_status_stamp = time.monotonic()

    def _on_line_state(self, msg: InfeedLineState) -> None:
        self.line_state = msg
        self.line_state_stamp = time.monotonic()

    def _call_async(self, client, request, on_done) -> None:
        """Non-blocking service call; on_done(ok, message) runs from the ROS
        spin in the Qt thread."""
        if not client.service_is_ready():
            on_done(False, f'{client.srv_name} unavailable (is its node running?)')
            return

        def _done(fut):
            result = fut.result()
            if result is None:
                on_done(False, f'{client.srv_name} failed: {fut.exception()}')
            else:
                on_done(bool(result.success), str(result.message))

        client.call_async(request).add_done_callback(_done)

    def set_line_enabled(self, enabled: bool, on_done) -> None:
        self._call_async(self._line_enable_client, SetBool.Request(data=bool(enabled)), on_done)

    def release_line(self, on_done) -> None:
        self._call_async(self._line_release_client, Trigger.Request(), on_done)

    def set_demo_enabled(self, enabled: bool, on_done) -> None:
        self._call_async(self._demo_enable_client, SetBool.Request(data=bool(enabled)), on_done)

    def restart_demo(self, on_done) -> None:
        self._call_async(self._demo_restart_client, Trigger.Request(), on_done)

    def current_joint_positions(self, robot_ns: str) -> Optional[list[float]]:
        """The robot's actual joint positions in robot_joint_names order, or
        None if no joint state has arrived yet (or a joint is missing)."""
        msg = self._joint_states.get(robot_ns)
        if msg is None:
            return None
        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index for name in self.robot_joint_names):
            return None
        return [msg.position[index[name]] for name in self.robot_joint_names]

    def send_joint_positions(self, robot_ns: str, positions: list[float],
                             move_time: float = MOVE_TIME_SEC) -> None:
        """Single-point trajectory: reach `positions` in move_time seconds
        (controller time - sim time under Gazebo). A new one replaces the old."""
        pub = self._traj_pubs.get(robot_ns)
        if pub is None:
            return
        msg = JointTrajectory()
        msg.joint_names = list(self.robot_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        sec = int(move_time)
        point.time_from_start = Duration(sec=sec, nanosec=int((move_time - sec) * 1e9))
        msg.points = [point]
        pub.publish(msg)

    def send_belt_speed(self, conveyor_ns: str, speed: float) -> None:
        pub = self._speed_pubs.get(conveyor_ns)
        if pub is None:
            return
        pub.publish(Float64(data=float(speed)))

    def _call_service(self, client, request, service_desc: str) -> tuple[bool, str]:
        """Blocking service call - acceptable for an infrequent button
        click (mirrors this UI's previous subprocess-blocking behavior for
        the same actions, before they moved into gripper_manager)."""
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            return False, f'{service_desc} unavailable (gripper_manager not running yet?)'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_CALL_TIMEOUT_SEC)
        if not future.done():
            return False, f'{service_desc} timed out'
        result = future.result()
        if result is None:
            return False, f'{service_desc} failed: {future.exception()}'
        return bool(result.success), str(result.message)

    def set_gripper(self, robot_ns: str, attach: bool) -> tuple[bool, str]:
        """Suction is a latching state, not a one-shot grab: while it's ON,
        gripper_manager keeps capturing whatever enters the gripper's
        footprint (so several objects can be picked up together), and turning
        it OFF releases everything."""
        client = self._suction_clients.get(robot_ns)
        if client is None:
            return False, f'no gripper service client for {robot_ns}'
        request = SetBool.Request()
        request.data = bool(attach)
        return self._call_service(client, request, 'gripper/set_suction')

    def spawn_box(self, width: float, depth: float, height: float, x: float, y: float, z: float) -> tuple[bool, str]:
        # Spawn Box is a scene-level control (not tied to any one robot in
        # this UI), but gripper_manager's slot pool is inherently per-robot -
        # target the first configured robot. Fine for this single-robot
        # workcell; a multi-robot scale-up would need the UI to let the
        # operator pick which robot's pool to spawn into.
        if not self.robot_namespaces:
            return False, 'no robot_namespaces configured'
        robot_ns = self.robot_namespaces[0]
        client = self._spawn_box_clients.get(robot_ns)
        if client is None:
            return False, f'no spawn_box service client for {robot_ns}'
        request = SpawnBox.Request()
        request.width = float(width)
        request.depth = float(depth)
        request.height = float(height)
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        return self._call_service(client, request, 'gripper/spawn_box')

    def configure_factory(self, config: dict, on_done) -> None:
        """Non-blocking: box_factory applies the parameters on its own timer,
        on_done(ok, message) runs from the ROS spin once it has answered."""
        if not self._factory_params.services_are_ready():
            on_done(False, 'box_factory unavailable (not running yet?)')
            return
        params = [Parameter(name, value=value) for name, value in config.items()]
        future = self._factory_params.set_parameters(params)

        def _done(fut):
            result = fut.result()
            if result is None:
                on_done(False, f'set_parameters failed: {fut.exception()}')
                return
            failed = [r.reason for r in result.results if not r.successful]
            on_done(not failed, '; '.join(failed) if failed else 'factory updated')

        future.add_done_callback(_done)


# Jogging: while a -/+ button is held, a target `JOG_LOOKAHEAD_SEC` ahead of
# the ACTUAL joint position (at the chosen speed) is re-sent every
# JOG_PERIOD_MS - the controller only takes positions, so this is how a
# velocity command is expressed. Built on the measured state, it cannot run
# away or jump, and it moves at the chosen speed in controller (sim) time
# whatever the real-time factor. Releasing the button sends a hold.
JOG_PERIOD_MS = 50
JOG_LOOKAHEAD_SEC = 0.25
JOG_SPEEDS = (('5%', 0.05), ('10%', 0.10), ('25%', 0.25), ('50%', 0.50))
HOME_SPEED_FRACTION = 0.25  # of each joint's max velocity, for the Home move
LIMIT_MARGIN_RAD = 0.01


class JointJogRow(QtWidgets.QWidget):
    """-, live position bar, + for one joint. pressed(direction) / released()."""
    pressed = QtCore.pyqtSignal(int)
    released = QtCore.pyqtSignal()
    _STEPS = 1000

    def __init__(self, joint_name: str, lo: float, hi: float, parent=None):
        super().__init__(parent)
        self.lo, self.hi = lo, (hi if hi > lo else lo + 1.0)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        name = QtWidgets.QLabel(joint_name.replace('gp_joint_', 'J'))
        name.setMinimumWidth(28)
        name.setToolTip(joint_name)
        self.minus = self._jog_button('\u2212', -1)
        self.plus = self._jog_button('+', +1)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(0, self._STEPS)
        self.bar.setTextVisible(False)
        self.bar.setMinimumWidth(40)
        self.bar.setToolTip(f'{math.degrees(self.lo):.0f} .. {math.degrees(self.hi):.0f} deg')
        self.value = QtWidgets.QLabel('-')
        self.value.setMinimumWidth(56)
        self.value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        layout.addWidget(name)
        layout.addWidget(self.minus)
        layout.addWidget(self.bar, 1)
        layout.addWidget(self.plus)
        layout.addWidget(self.value)

    def _jog_button(self, text: str, direction: int) -> QtWidgets.QPushButton:
        btn = QtWidgets.QPushButton(text)
        btn.setObjectName('jog')
        btn.setAutoRepeat(False)
        btn.pressed.connect(lambda: self.pressed.emit(direction))
        btn.released.connect(self.released.emit)
        return btn

    def show_position(self, rad: Optional[float]) -> None:
        if rad is None:
            self.value.setText('-')
            return
        frac = (rad - self.lo) / (self.hi - self.lo)
        self.bar.setValue(int(round(min(1.0, max(0.0, frac)) * self._STEPS)))
        self.value.setText(f'{math.degrees(rad):6.1f}\u00b0')


class RobotPanel(QtWidgets.QGroupBox):
    """Hold-to-jog per joint (see JOG_*), a speed selector, Home and the
    suction gripper. Position bars always show the measured joint states."""

    def __init__(self, node: 'TeleopNode', robot_ns: str, parent=None):
        super().__init__(f'Robot {robot_ns}', parent)
        self.node = node
        self.robot_ns = robot_ns
        self.joint_names = list(node.robot_joint_names)
        self.limits = [node.joint_limits.get(n, (-math.pi, math.pi, 1.0)) for n in self.joint_names]
        self._jog: Optional[tuple[int, int]] = None  # (joint index, direction)

        layout = QtWidgets.QVBoxLayout(self)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel('Jog speed'))
        self.speed_group = QtWidgets.QButtonGroup(self)
        for i, (label, frac) in enumerate(JOG_SPEEDS):
            btn = QtWidgets.QPushButton(label)
            btn.setCheckable(True)
            btn.setChecked(frac == 0.10)
            btn.setToolTip(f'{frac * 100:.0f}% of each joint\'s max velocity')
            self.speed_group.addButton(btn, i)
            speed_row.addWidget(btn)
        layout.addLayout(speed_row)

        self.rows: list[JointJogRow] = []
        for i, name in enumerate(self.joint_names):
            lo, hi, _ = self.limits[i]
            row = JointJogRow(name, lo, hi)
            row.pressed.connect(lambda d, i=i: self._start_jog(i, d))
            row.released.connect(self._stop_jog)
            layout.addWidget(row)
            self.rows.append(row)

        buttons = QtWidgets.QHBoxLayout()
        home_btn = QtWidgets.QPushButton('Home')
        home_btn.setToolTip(f'Joint move to all zeros at {HOME_SPEED_FRACTION * 100:.0f}% speed')
        home_btn.clicked.connect(self._go_home)
        buttons.addWidget(home_btn)
        self.gripper_btn = QtWidgets.QPushButton()
        self.gripper_btn.setCheckable(True)
        self.gripper_btn.clicked.connect(self._on_gripper_clicked)
        buttons.addWidget(self.gripper_btn)
        layout.addLayout(buttons)

        self.status_label = QtWidgets.QLabel('')
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        self._jog_timer = QtCore.QTimer(self)
        self._jog_timer.timeout.connect(self._jog_tick)
        self.refresh()

    # ── Jogging ─────────────────────────────────────────────────────────────

    def _speed_fraction(self) -> float:
        return JOG_SPEEDS[max(0, self.speed_group.checkedId())][1]

    def _start_jog(self, index: int, direction: int) -> None:
        self._jog = (index, direction)
        self._jog_tick()
        self._jog_timer.start(JOG_PERIOD_MS)

    def _stop_jog(self) -> None:
        was = self._jog
        self._jog = None
        self._jog_timer.stop()
        q = self.node.current_joint_positions(self.robot_ns)
        if q is not None:
            self.node.send_joint_positions(self.robot_ns, q, move_time=0.1)  # hold here
            if was is not None:
                self.status_label.setText(f'Stopped - holding {self.joint_names[was[0]]} at '
                                          f'{math.degrees(q[was[0]]):.1f}\u00b0')

    def _jog_tick(self) -> None:
        if self._jog is None:
            return
        q = self.node.current_joint_positions(self.robot_ns)
        if q is None:
            self.status_label.setText(f'No joint states on /{self.robot_ns}/joint_states - cannot jog')
            return
        i, direction = self._jog
        if not (self.rows[i].minus.isDown() or self.rows[i].plus.isDown()):
            self._stop_jog()  # release event lost (focus change, touch) - never keep jogging
            return
        lo, hi, vmax = self.limits[i]
        target = list(q)
        step = direction * self._speed_fraction() * vmax * JOG_LOOKAHEAD_SEC
        target[i] = min(hi - LIMIT_MARGIN_RAD, max(lo + LIMIT_MARGIN_RAD, q[i] + step))
        self.node.send_joint_positions(self.robot_ns, target, move_time=JOG_LOOKAHEAD_SEC)
        at_limit = target[i] in (hi - LIMIT_MARGIN_RAD, lo + LIMIT_MARGIN_RAD)
        self.status_label.setText(
            f'Jogging {self.joint_names[i]} {"+" if direction > 0 else "-"} at '
            f'{self._speed_fraction() * vmax:.2f} rad/s' + (' - at the joint limit' if at_limit else ''))

    def _go_home(self) -> None:
        q = self.node.current_joint_positions(self.robot_ns)
        if q is None:
            self.status_label.setText('No joint states yet')
            return
        # Slowest joint sets the time, so no joint exceeds HOME_SPEED_FRACTION.
        t = max([abs(a) / (HOME_SPEED_FRACTION * lim[2]) for a, lim in zip(q, self.limits)] + [1.0])
        self.node.send_joint_positions(self.robot_ns, [0.0] * len(q), move_time=t)
        self.status_label.setText(f'Moving home in {t:.1f}s (sim time)')

    # ── Gripper ─────────────────────────────────────────────────────────────

    def _on_gripper_clicked(self, checked: bool) -> None:
        ok, message = self.node.set_gripper(self.robot_ns, checked)
        self.status_label.setText(('OK: ' if ok else 'FAILED: ') + message)
        self.refresh()

    # ── Live state ──────────────────────────────────────────────────────────

    def refresh(self) -> None:
        q = self.node.current_joint_positions(self.robot_ns)
        for i, row in enumerate(self.rows):
            row.show_position(None if q is None else q[i])
        g = self.node.gripper_states.get(self.robot_ns)
        on = bool(g and g.suction_on)
        held = list(g.grasped) if g else []
        self.gripper_btn.blockSignals(True)
        self.gripper_btn.setChecked(on)
        self.gripper_btn.blockSignals(False)
        text = ('Suction ON' + (f' ({len(held)} held)' if held else '')) if on else 'Suction OFF'
        color = (STATUS_COLORS['busy'] if held else STATUS_COLORS['ok']) if on else STATUS_COLORS['off']
        self.gripper_btn.setText(text)
        self.gripper_btn.setStyleSheet(f'background-color: {color}; color: white; font-weight: bold;')


class ConveyorPanel(QtWidgets.QGroupBox):
    _STEPS = 1000

    def __init__(self, conveyor_ns: str, max_speed: float, on_change, parent=None):
        super().__init__(conveyor_ns, parent)
        self.conveyor_ns = conveyor_ns
        self.max_speed = max_speed
        self.on_change = on_change

        layout = QtWidgets.QHBoxLayout(self)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(-self._STEPS)
        self.slider.setMaximum(self._STEPS)
        self.slider.setValue(0)
        self.value_label = QtWidgets.QLabel('0.00 rad/s')
        self.value_label.setMinimumWidth(90)
        stop_btn = QtWidgets.QPushButton('Stop')
        stop_btn.clicked.connect(lambda: self.slider.setValue(0))
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.value_label)
        layout.addWidget(stop_btn)

        self.slider.valueChanged.connect(self._on_slider_changed)

    def _on_slider_changed(self, step: int) -> None:
        speed = (step / self._STEPS) * self.max_speed
        self.value_label.setText(f'{speed:6.2f} rad/s')
        self.on_change(self.conveyor_ns, speed)


class SpawnPanel(QtWidgets.QGroupBox):
    """Width/depth/height and X/Y/Z position are all user-set here, not
    randomized - see the module docstring. Position spinboxes are seeded
    from the node's spawn_x/y/z params (default: on top of
    conveyor_package_infeed, at its far end away from the robot - see
    TeleopNode's own declare_parameter comment) but freely editable before
    each click. Each click spawns a NEW box into the next slot of a
    round-robin pool (see TeleopNode's module docstring/spawn_box) - it
    does NOT despawn other existing boxes, only the older box (if any)
    that already occupied the slot being reused."""

    def __init__(self, on_spawn, default_x: float, default_y: float, default_z: float, parent=None):
        super().__init__('Spawn Box', parent)
        self.on_spawn = on_spawn

        form = QtWidgets.QFormLayout()
        self.width_spin = self._make_size_spin(0.15)
        self.depth_spin = self._make_size_spin(0.20)
        self.height_spin = self._make_size_spin(0.15)
        form.addRow('Width (X), m', self.width_spin)
        form.addRow('Depth (Y), m', self.depth_spin)
        form.addRow('Height (Z), m', self.height_spin)

        self.x_spin = self._make_position_spin(default_x)
        self.y_spin = self._make_position_spin(default_y)
        self.z_spin = self._make_position_spin(default_z)
        form.addRow('Position X, m', self.x_spin)
        form.addRow('Position Y, m', self.y_spin)
        form.addRow('Position Z, m', self.z_spin)

        self.spawn_btn = QtWidgets.QPushButton('Spawn Box')
        self.spawn_btn.clicked.connect(self._on_spawn_clicked)

        self.status_label = QtWidgets.QLabel('')
        self.status_label.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.spawn_btn)
        layout.addWidget(self.status_label)
        layout.addStretch(1)

    @staticmethod
    def _make_size_spin(default: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(0.05, 0.40)
        spin.setSingleStep(0.01)
        spin.setDecimals(2)
        spin.setValue(default)
        return spin

    @staticmethod
    def _make_position_spin(default: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(-10.0, 10.0)
        spin.setSingleStep(0.05)
        spin.setDecimals(2)
        spin.setValue(default)
        return spin

    def _on_spawn_clicked(self) -> None:
        self.spawn_btn.setEnabled(False)
        self.status_label.setText('spawning...')
        QtWidgets.QApplication.processEvents()
        ok, message = self.on_spawn(
            self.width_spin.value(), self.depth_spin.value(), self.height_spin.value(),
            self.x_spin.value(), self.y_spin.value(), self.z_spin.value(),
        )
        self.status_label.setText(('OK: ' if ok else 'FAILED: ') + message)
        self.spawn_btn.setEnabled(True)


class FactoryPanel(QtWidgets.QGroupBox):
    """Controls for box_factory (see the module docstring). Defaults mirror
    box_factory_bringup/config/box_factory.yaml - the node itself starts
    disabled, so nothing runs until the checkbox is ticked."""
    _SIZE_LIMITS = (0.05, 0.40)  # same range as the Spawn Box panel

    def __init__(self, on_configure, parent=None):
        super().__init__('Box Factory', parent)
        self.on_configure = on_configure

        self.enabled_check = QtWidgets.QCheckBox('Factory active')
        self.period_spin = QtWidgets.QDoubleSpinBox()
        self.period_spin.setRange(0.2, 60.0)
        self.period_spin.setSingleStep(0.5)
        self.period_spin.setDecimals(1)
        self.period_spin.setValue(3.0)
        self.period_spin.setSuffix(' s')

        self.time_rand = self._make_percent_slider(30)
        self.shape_rand = self._make_percent_slider(50)

        form = QtWidgets.QFormLayout()
        form.addRow(self.enabled_check)
        form.addRow('Spawn every', self.period_spin)
        form.addRow('Time randomization', self.time_rand)
        form.addRow('Size/pose randomization', self.shape_rand)

        # Per-edge min/max, one row per dimension.
        sizes = QtWidgets.QGridLayout()
        sizes.addWidget(QtWidgets.QLabel('min, m'), 0, 1)
        sizes.addWidget(QtWidgets.QLabel('max, m'), 0, 2)
        self.min_spins, self.max_spins = [], []
        for row, (label, lo, hi) in enumerate(
                (('Width (X)', 0.10, 0.30), ('Depth (Y)', 0.10, 0.35), ('Height (Z)', 0.08, 0.25)), start=1):
            min_spin, max_spin = self._make_size_spin(lo), self._make_size_spin(hi)
            min_spin.valueChanged.connect(lambda v, m=max_spin: m.setValue(max(m.value(), v)))
            max_spin.valueChanged.connect(lambda v, m=min_spin: m.setValue(min(m.value(), v)))
            sizes.addWidget(QtWidgets.QLabel(label), row, 0)
            sizes.addWidget(min_spin, row, 1)
            sizes.addWidget(max_spin, row, 2)
            self.min_spins.append(min_spin)
            self.max_spins.append(max_spin)

        self.status_label = QtWidgets.QLabel('')
        self.status_label.setWordWrap(True)
        # Live state from the factory node's latched status topic.
        self.live_label = QtWidgets.QLabel('')
        self.live_label.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.live_label)
        layout.addLayout(form)
        layout.addLayout(sizes)
        layout.addWidget(self.status_label)

        self.enabled_check.toggled.connect(self._push)
        self.period_spin.valueChanged.connect(self._push)
        for slider in (self.time_rand, self.shape_rand):
            slider.findChild(QtWidgets.QSlider).valueChanged.connect(self._push)
        for spin in self.min_spins + self.max_spins:
            spin.valueChanged.connect(self._push)

    @staticmethod
    def _make_percent_slider(default: int) -> QtWidgets.QWidget:
        box = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(box)
        row.setContentsMargins(0, 0, 0, 0)
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, 100)
        label = QtWidgets.QLabel()
        label.setMinimumWidth(40)
        slider.valueChanged.connect(lambda v: label.setText(f'{v}%'))
        slider.setValue(default)
        label.setText(f'{default}%')
        row.addWidget(slider, 1)
        row.addWidget(label)
        return box

    @classmethod
    def _make_size_spin(cls, default: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(*cls._SIZE_LIMITS)
        spin.setSingleStep(0.01)
        spin.setDecimals(2)
        spin.setValue(default)
        return spin

    @staticmethod
    def _percent(slider_box: QtWidgets.QWidget) -> float:
        return slider_box.findChild(QtWidgets.QSlider).value() / 100.0

    def config(self) -> dict:
        return {
            'enabled': self.enabled_check.isChecked(),
            'period_sec': self.period_spin.value(),
            'time_randomness': self._percent(self.time_rand),
            'shape_randomness': self._percent(self.shape_rand),
            'size_min': [s.value() for s in self.min_spins],
            'size_max': [s.value() for s in self.max_spins],
        }

    def _push(self, *_args) -> None:
        self.on_configure(self.config(), self._on_result)

    def refresh(self, status: Optional[BoxFactoryStatus]) -> None:
        if status is None:
            self.live_label.setText('<b>Factory: not running</b> (started by workcell.launch.py)')
            return
        if not status.enabled:
            state, color = 'OFF', '#616161'
        elif status.paused_by_line:
            state, color = 'PAUSED - feeding line stopped', '#c62828'
        elif status.spawn_area_blocked:
            state, color = 'WAITING - spawn area occupied', '#ef6c00'
        else:
            nxt = f', next box in {status.next_spawn_in:.1f}s' if status.next_spawn_in >= 0 else ''
            state, color = f'SPAWNING{nxt}', '#2e7d32'
        extra = '' if status.gazebo_ok else ' | Gazebo create service not answering'
        last = f' (last: {status.last_box})' if status.last_box else ''
        self.live_label.setText(f'<b><span style="color:{color}">{state}</span></b> - '
                                f'{status.boxes_spawned} boxes spawned{last}{extra}')
        # Keep the checkbox in sync if someone else toggled `enabled`.
        if self.enabled_check.isChecked() != status.enabled:
            self.enabled_check.blockSignals(True)
            self.enabled_check.setChecked(status.enabled)
            self.enabled_check.blockSignals(False)

    def _on_result(self, ok: bool, message: str) -> None:
        self.status_label.setText(('OK: ' if ok else 'FAILED: ') + message)
        if not ok and self.enabled_check.isChecked():
            self.enabled_check.blockSignals(True)
            self.enabled_check.setChecked(False)
            self.enabled_check.blockSignals(False)


class InfeedLinePanel(QtWidgets.QGroupBox):
    """Feeding line status and controls (workcell_bringup's
    infeed_line_controller). Refreshed from its latched line/state topic on
    every Qt tick by TeleopMainWindow; the infeed belt's speed slider in the
    Conveyors column sets the speed the line runs at."""
    STALE_SEC = 3.0
    _MODES = {
        InfeedLineState.MODE_DISABLED: ('OFF', '#616161', 'line switched off'),
        InfeedLineState.MODE_RUNNING: ('RUNNING', '#2e7d32', 'feeding boxes'),
        InfeedLineState.MODE_BLOCKED: ('STOPPED - BOX AT BEAM', '#c62828',
                                       'waiting for the robot to pick the box (or Release)'),
        InfeedLineState.MODE_RELEASED: ('RELEASED', '#ef6c00', 'running until the box passes the beam'),
    }

    def __init__(self, node: 'TeleopNode', parent=None):
        super().__init__('Infeed Line', parent)
        self.node = node

        self.mode_label = QtWidgets.QLabel()
        self.mode_label.setAlignment(QtCore.Qt.AlignCenter)
        self.detail_label = QtWidgets.QLabel()
        self.detail_label.setWordWrap(True)
        self.beam_label = QtWidgets.QLabel()
        self.speed_label = QtWidgets.QLabel()

        self.enable_btn = QtWidgets.QPushButton()
        self.enable_btn.setCheckable(True)
        self.enable_btn.clicked.connect(self._on_enable_clicked)
        self.release_btn = QtWidgets.QPushButton('Release box')
        self.release_btn.setToolTip('Ignore the box at the beam: run the belt until it has passed')
        self.release_btn.clicked.connect(self._on_release_clicked)

        self.status_label = QtWidgets.QLabel('')
        self.status_label.setWordWrap(True)

        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.enable_btn)
        buttons.addWidget(self.release_btn)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.mode_label)
        layout.addWidget(self.detail_label)
        layout.addWidget(self.beam_label)
        layout.addWidget(self.speed_label)
        layout.addLayout(buttons)
        layout.addWidget(self.status_label)
        self.refresh()

    def _state(self) -> Optional[InfeedLineState]:
        if self.node.line_state is None or time.monotonic() - self.node.line_state_stamp > self.STALE_SEC:
            return None
        return self.node.line_state

    def refresh(self) -> None:
        st = self._state()
        if st is None:
            self._set_mode('NO CONTROLLER', '#424242')
            self.detail_label.setText('Start it: ros2 launch workcell_bringup infeed_line.launch.py '
                                      '(until then the infeed belt ignores its speed slider and the '
                                      'box factory runs unconditionally)')
            self.beam_label.setText('Beam: -')
            self.speed_label.setText('')
            self.enable_btn.setEnabled(False)
            self.release_btn.setEnabled(False)
            self._set_enable_text(False)
            return
        text, color, detail = self._MODES.get(st.mode, (f'mode {st.mode}', '#424242', ''))
        self._set_mode(text, color)
        self.detail_label.setText(detail + ' - box factory ' +
                                  ('spawning' if st.mode in (InfeedLineState.MODE_RUNNING,
                                                             InfeedLineState.MODE_RELEASED) else 'paused'))
        if not st.beam_ok:
            self.beam_label.setText('Beam: NO SCANS (sensor missing?)')
        else:
            rng = f'{st.beam_range:.2f} m' if math.isfinite(st.beam_range) else 'clear'
            self.beam_label.setText(f"Beam: {'BLOCKED' if st.beam_blocked else 'clear'} ({rng}) | "
                                    f'boxes stopped: {st.boxes_stopped}')
        self.speed_label.setText(f'Speed: request {st.speed_request:+.2f} rad/s, '
                                 f'belt {st.speed_command:+.2f} rad/s')
        self.enable_btn.setEnabled(True)
        self.release_btn.setEnabled(st.mode == InfeedLineState.MODE_BLOCKED)
        self._set_enable_text(st.enabled)

    def _set_mode(self, text: str, color: str) -> None:
        self.mode_label.setText(text)
        self.mode_label.setStyleSheet(
            f'background-color: {color}; color: white; font-weight: bold; padding: 4px;')

    def _set_enable_text(self, enabled: bool) -> None:
        self.enable_btn.blockSignals(True)
        self.enable_btn.setChecked(enabled)
        self.enable_btn.blockSignals(False)
        self.enable_btn.setText('Line ON - stop' if enabled else 'Line OFF - start')

    def _report(self, ok: bool, message: str) -> None:
        self.status_label.setText(('OK: ' if ok else 'FAILED: ') + message)

    def _on_enable_clicked(self, checked: bool) -> None:
        self.node.set_line_enabled(checked, self._report)

    def _on_release_clicked(self) -> None:
        self.node.release_line(self._report)


class DemoPanel(QtWidgets.QGroupBox):
    """Pick & place demo (workcell_demo): its live ~/state, run/pause, and
    Restart - abort the running cycle, suction off, go home, clear any ERROR
    and run again (the demo's ~/restart service)."""
    _LEVELS = {'ERROR': 'bad', 'WAITING': 'ok', 'DISABLED': 'off', 'STARTING': 'warn'}

    def __init__(self, node: 'TeleopNode', parent=None):
        super().__init__('Pick & Place Demo', parent)
        self.node = node
        self.state_label = QtWidgets.QLabel()
        self.state_label.setAlignment(QtCore.Qt.AlignCenter)
        self.detail_label = QtWidgets.QLabel()
        self.detail_label.setWordWrap(True)
        self.run_btn = QtWidgets.QPushButton('Pause')
        self.run_btn.setToolTip('Pause / run the demo (a running cycle finishes first)')
        self.run_btn.clicked.connect(self._on_run_clicked)
        self.restart_btn = QtWidgets.QPushButton('Restart')
        self.restart_btn.setToolTip('Abort the running cycle, suction off, go home, clear ERROR and run again')
        self.restart_btn.clicked.connect(self._on_restart_clicked)
        self.status_label = QtWidgets.QLabel('')
        self.status_label.setWordWrap(True)

        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.run_btn)
        buttons.addWidget(self.restart_btn)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.state_label)
        layout.addWidget(self.detail_label)
        layout.addLayout(buttons)
        layout.addWidget(self.status_label)
        self.refresh()

    def refresh(self) -> None:
        up = self.node.has_service('/workcell_demo/restart')
        text = self.node.demo_state if up else None
        state, _, detail = (text or '').partition('|')
        state = state.strip()
        if not up:
            state, detail = 'NOT RUNNING', 'start it: ros2 launch workcell_demo demo.launch.py'
        level = self._LEVELS.get(state, 'busy') if up else 'off'
        self.state_label.setText(f'<b>{state or "?"}</b>')
        self.state_label.setStyleSheet(f'background-color: {STATUS_COLORS[level]}; color: white; '
                                       f'border-radius: 6px; padding: 6px;')
        self.detail_label.setText(detail.strip())
        self.run_btn.setText('Run' if state == 'DISABLED' else 'Pause')
        for btn in (self.run_btn, self.restart_btn):
            btn.setEnabled(up)

    def _report(self, ok: bool, message: str) -> None:
        self.status_label.setText(message if ok else f'<span style="color:#c62828">{message}</span>')

    def _on_run_clicked(self) -> None:
        self.node.set_demo_enabled(self.run_btn.text() == 'Run', self._report)

    def _on_restart_clicked(self) -> None:
        self.node.restart_demo(self._report)


class FlowLayout(QtWidgets.QLayout):
    """Left-to-right layout that wraps onto new lines (Qt's flow layout example)."""

    def __init__(self, parent=None, spacing: int = 6):
        super().__init__(parent)
        self._items = []
        self._spacing = spacing

    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, index):
        return self._items[index] if 0 <= index < len(self._items) else None

    def takeAt(self, index):
        return self._items.pop(index) if 0 <= index < len(self._items) else None

    def expandingDirections(self):
        return QtCore.Qt.Orientations(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._arrange(QtCore.QRect(0, 0, width, 0), apply=False)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._arrange(rect, apply=True)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QtCore.QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        m = self.contentsMargins()
        return size + QtCore.QSize(m.left() + m.right(), m.top() + m.bottom())

    def _arrange(self, rect, apply: bool) -> int:
        x, y, line_h = rect.x(), rect.y(), 0
        for item in self._items:
            w, h = item.sizeHint().width(), item.sizeHint().height()
            if x + w > rect.right() and line_h > 0:
                x, y, line_h = rect.x(), y + line_h + self._spacing, 0
            if apply:
                item.setGeometry(QtCore.QRect(QtCore.QPoint(x, y), item.sizeHint()))
            x += w + self._spacing
            line_h = max(line_h, h)
        return y + line_h - rect.y()


class StatusChip(QtWidgets.QLabel):
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.set('off', '-')

    def set(self, level: str, text: str, tooltip: str = '') -> None:
        self.setText(f'<b>{self.name}</b> {text}')
        self.setToolTip(tooltip or text)
        self.setStyleSheet(f'background-color: {STATUS_COLORS[level]}; color: white; '
                           f'border-radius: 10px; padding: 4px 10px;')


class StatusBanner(QtWidgets.QWidget):
    """Workcell status at a glance - one coloured chip per subsystem, wrapping
    onto more lines on a narrow window. Refreshed by the main window's tick."""
    STALE_SEC = 3.0

    def __init__(self, node: 'TeleopNode', parent=None):
        super().__init__(parent)
        self.node = node
        self.ns = node.robot_namespaces[0] if node.robot_namespaces else 'robot_1'
        self.chips = {k: StatusChip(k) for k in
                      ('Sim', 'Robot', 'Planner', 'Navigator', 'Gripper', 'Line', 'Factory', 'Demo')}
        flow = FlowLayout(self)
        flow.setContentsMargins(4, 4, 4, 4)
        for chip in self.chips.values():
            flow.addWidget(chip)

    def _fresh(self, stamp: float) -> bool:
        return time.monotonic() - stamp < self.STALE_SEC

    def refresh(self) -> None:
        n, c, ns = self.node, self.chips, self.ns

        if not self._fresh(n.clock_stamp):
            c['Sim'].set('bad', 'no /clock', 'Gazebo not running (workcell.launch.py) or paused')
        else:
            rtf = n.sim_rtf
            level = 'ok' if rtf is None or rtf > 0.5 else 'warn'
            c['Sim'].set(level, f'{rtf:.2f}x' if rtf is not None else 'running',
                         'Gazebo real-time factor (sim seconds per wall second)')

        if self._fresh(n.joint_state_stamp.get(ns, 0.0)):
            c['Robot'].set('ok', 'joint states', f'/{ns}/joint_states arriving')
        else:
            c['Robot'].set('bad', 'no joint states', f'nothing on /{ns}/joint_states')

        up = n.has_service
        c['Planner'].set('ok' if up(f'/{ns}/cumotion/motion_plan/_action/send_goal') else 'off',
                         'up' if up(f'/{ns}/cumotion/motion_plan/_action/send_goal') else 'down',
                         'cuMotion MotionPlan action (cumotion.launch.py)')
        c['Navigator'].set('ok' if up(f'/{ns}/navigator/navigate_to_node/_action/send_goal') else 'off',
                           'up' if up(f'/{ns}/navigator/navigate_to_node/_action/send_goal') else 'down',
                           'navigator_server actions (demo.launch.py / navigator_server.launch.py)')

        g = n.gripper_states.get(ns)
        if g is None:
            c['Gripper'].set('off', '-', 'no gripper/state (gripper_manager not running?)')
        elif g.grasped:
            c['Gripper'].set('busy', f'holding {len(g.grasped)}', ', '.join(g.grasped))
        else:
            c['Gripper'].set('warn' if g.suction_on else 'off', 'suction on' if g.suction_on else 'off')

        st = n.line_state if self._fresh(n.line_state_stamp) else None
        if st is None:
            c['Line'].set('off', 'no controller', 'infeed_line.launch.py not running')
        else:
            text, level = {InfeedLineState.MODE_DISABLED: ('off', 'off'),
                           InfeedLineState.MODE_RUNNING: ('running', 'ok'),
                           InfeedLineState.MODE_BLOCKED: ('box at beam', 'bad'),
                           InfeedLineState.MODE_RELEASED: ('released', 'busy')}.get(st.mode, ('?', 'warn'))
            c['Line'].set(level, text, f'beam {"blocked" if st.beam_blocked else "clear"}, '
                                      f'{st.boxes_stopped} boxes stopped')

        fs = n.factory_status if self._fresh(n.factory_status_stamp) else None
        if fs is None:
            c['Factory'].set('off', 'not running')
        elif not fs.enabled:
            c['Factory'].set('off', f'off ({fs.boxes_spawned})')
        elif fs.paused_by_line:
            c['Factory'].set('warn', f'paused ({fs.boxes_spawned})', 'feeding line stopped')
        else:
            c['Factory'].set('ok', f'spawning ({fs.boxes_spawned})', f'last box {fs.last_box}')

        demo_up = up('/workcell_demo/set_enabled')
        text = n.demo_state if demo_up and n.demo_state else None
        if text is None:
            c['Demo'].set('off', 'not running', 'demo.launch.py')
        else:
            state = text.split('|')[0].strip()
            level = {'ERROR': 'bad', 'WAITING': 'ok', 'DISABLED': 'off', 'STARTING': 'warn'}.get(state, 'busy')
            c['Demo'].set(level, state.lower(), text)


class ResponsiveColumns(QtWidgets.QWidget):
    """Lays the panels out in as many columns as fit (column_width px each,
    up to max_columns), re-flowing on resize - three side by side on a wide
    screen, one below the other on a phone-sized window."""

    def __init__(self, panels: list, column_width: int = 400, max_columns: int = 3, parent=None):
        super().__init__(parent)
        self.panels = panels
        self.column_width = column_width
        self.max_columns = max_columns
        self._columns = 0
        self._row = QtWidgets.QHBoxLayout(self)
        self._row.setContentsMargins(0, 0, 0, 0)
        self._reflow(1)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        n = max(1, min(self.max_columns, event.size().width() // self.column_width))
        if n != self._columns:
            self._reflow(n)

    def _reflow(self, n: int) -> None:
        self._columns = n
        while self._row.count():
            item = self._row.takeAt(0)
            if item.layout():
                while item.layout().count():
                    item.layout().takeAt(0)
        columns = [QtWidgets.QVBoxLayout() for _ in range(n)]
        # Fill column by column in order, balancing by panel height.
        heights = [0] * n
        for panel in self.panels:
            col = heights.index(min(heights)) if n > 1 else 0
            columns[col].addWidget(panel)
            heights[col] += panel.sizeHint().height()
        for col in columns:
            col.addStretch(1)
            self._row.addLayout(col, 1)


class ConveyorsPanel(QtWidgets.QGroupBox):
    def __init__(self, node: 'TeleopNode', on_change, parent=None):
        super().__init__('Conveyors', parent)
        layout = QtWidgets.QVBoxLayout(self)
        for ns in node.conveyor_namespaces:
            row = ConveyorPanel(ns.replace('conveyor_', ''), node.conveyor_max_speed,
                                lambda _name, speed, ns=ns: on_change(ns, speed))
            row.setFlat(True)
            layout.addWidget(row)


def image_to_qimage(msg: Image, depth_range: tuple[float, float]) -> Optional[QtGui.QImage]:
    """sensor_msgs/Image -> QImage (copied, owns its data). Depth (32FC1 metres
    / 16UC1 millimetres) is shown as grey, near = bright, far = dark grey,
    clipped to depth_range; invalid pixels (nan/inf/0) are black."""
    w, h, enc = msg.width, msg.height, msg.encoding
    if w == 0 or h == 0:
        return None
    if enc in ('rgb8', 'bgr8'):
        img = QtGui.QImage(bytes(msg.data), w, h, msg.step, QtGui.QImage.Format_RGB888)
        return img.rgbSwapped() if enc == 'bgr8' else img.copy()
    if enc in ('rgba8', 'bgra8'):
        img = QtGui.QImage(bytes(msg.data), w, h, msg.step, QtGui.QImage.Format_RGBA8888)
        return img.rgbSwapped() if enc == 'bgra8' else img.copy()
    if enc == 'mono8':
        return QtGui.QImage(bytes(msg.data), w, h, msg.step, QtGui.QImage.Format_Grayscale8).copy()
    if enc in ('32FC1', '16UC1'):
        dtype, scale = (np.float32, 1.0) if enc == '32FC1' else (np.uint16, 1e-3)
        depth = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(h, msg.step // np.dtype(dtype).itemsize)[:, :w]
        depth = depth.astype(np.float32) * scale
        near, far = depth_range
        valid = np.isfinite(depth) & (depth > 0.0)
        grey = np.zeros((h, w), dtype=np.uint8)
        # 255 (near) .. 40 (far), so black stays reserved for "no reading".
        t = (np.clip(depth[valid], near, far) - near) / (far - near)
        grey[valid] = (255.0 - 215.0 * t).astype(np.uint8)
        return QtGui.QImage(grey.tobytes(), w, h, w, QtGui.QImage.Format_Grayscale8).copy()
    return None


class CameraPanel(QtWidgets.QWidget):
    """Live gripper-camera feed (colour or depth) of one robot. Converts the
    node's latest frame on its own timer, and only while it is on screen."""
    STALE_SEC = 2.0
    DEPTH_RANGE = (0.10, 3.0)  # Gazebo sensor clip planes (group_a_macro.xacro)

    def __init__(self, node: 'TeleopNode', parent=None):
        super().__init__(parent)
        self.node = node
        self._shown_stamp = None

        self.robot = QtWidgets.QComboBox()
        self.robot.addItems(node.robot_namespaces)
        self.robot.setVisible(len(node.robot_namespaces) > 1)
        self.kind = QtWidgets.QComboBox()
        self.kind.addItems(['color', 'depth'])
        self.info = QtWidgets.QLabel('-')
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(self.robot)
        bar.addWidget(self.kind)
        bar.addWidget(self.info, 1)

        self.view = QtWidgets.QLabel('waiting for camera...')
        self.view.setAlignment(QtCore.Qt.AlignCenter)
        self.view.setMinimumSize(320, 240)
        self.view.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.view.setStyleSheet('background-color: black; color: #bdbdbd;')

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(bar)
        layout.addWidget(self.view, 1)

        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(100)  # the Gazebo camera publishes at 10 Hz

    def refresh(self) -> None:
        if not self.isVisible():
            return
        key = (self.robot.currentText(), self.kind.currentText())
        topic = f'/{key[0]}/camera/{key[1]}'
        stamps = self.node.camera_stamps.get(key, [])
        if not stamps or time.monotonic() - stamps[-1] > self.STALE_SEC:
            self.view.setText(f'no images on {topic}')
            self.info.setText(topic)
            self._shown_stamp = None
            return
        msg = self.node.camera_frames[key]
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self._shown_stamp:
            return
        img = image_to_qimage(msg, self.DEPTH_RANGE)
        if img is None:
            self.view.setText(f'unsupported encoding {msg.encoding!r} on {topic}')
            return
        self._shown_stamp = stamp
        self.view.setPixmap(QtGui.QPixmap.fromImage(img).scaled(
            self.view.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
        self.info.setText(f'{topic}  {msg.width}x{msg.height} {msg.encoding}  {rate:.1f} Hz')


APP_STYLE = """
QWidget { font-size: 11pt; }
QGroupBox { font-weight: bold; border: 1px solid #9e9e9e; border-radius: 6px; margin-top: 10px; padding-top: 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { min-height: 34px; padding: 2px 10px; }
QPushButton#jog { min-width: 44px; min-height: 40px; font-size: 16pt; font-weight: bold; }
QPushButton:checked { background-color: #1565c0; color: white; }
QSlider { min-height: 30px; }
QProgressBar { min-height: 16px; }
QDoubleSpinBox, QComboBox { min-height: 30px; }
"""


class TeleopMainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('Workcell Teleop')

        self._robot_panels = [RobotPanel(node, ns) for ns in node.robot_namespaces]
        self._line_panel = InfeedLinePanel(node)
        self._factory_panel = FactoryPanel(node.configure_factory)
        self._demo_panel = DemoPanel(node)

        # Status strip on top (always visible), one tab per area below it.
        self._banner = StatusBanner(node)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._scrolled(ResponsiveColumns(
            [*self._robot_panels, self._demo_panel], max_columns=2)), 'Robot')
        self.tabs.addTab(CameraPanel(node), 'Camera')
        self.tabs.addTab(self._scrolled(ResponsiveColumns(
            [self._line_panel, ConveyorsPanel(node, self._on_conveyor_changed)], max_columns=2)), 'Line && Belts')
        self.tabs.addTab(self._scrolled(ResponsiveColumns(
            [self._factory_panel, SpawnPanel(self._on_spawn_box, node.spawn_x, node.spawn_y, node.spawn_z)],
            max_columns=2)), 'Boxes')

        central = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.addWidget(self._banner)
        outer.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        # ROS spins in the Qt thread: drain every ready callback each tick
        # (joint states alone arrive faster than one-callback-per-tick).
        self._ros_timer = QtCore.QTimer(self)
        self._ros_timer.timeout.connect(self._on_ros_tick)
        self._ros_timer.start(50)
        self._graph_timer = QtCore.QTimer(self)
        self._graph_timer.timeout.connect(self.node.poll_graph)
        self._graph_timer.start(1000)
        self.node.poll_graph()

    @staticmethod
    def _scrolled(widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(widget)
        return scroll

    def _on_ros_tick(self) -> None:
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0)
        self._banner.refresh()
        self._line_panel.refresh()
        self._demo_panel.refresh()
        fresh = time.monotonic() - self.node.factory_status_stamp < 3.0
        self._factory_panel.refresh(self.node.factory_status if fresh else None)
        for panel in self._robot_panels:
            panel.refresh()

    def _on_conveyor_changed(self, conveyor_ns: str, speed: float) -> None:
        self.node.send_belt_speed(conveyor_ns, speed)

    def _on_spawn_box(self, width: float, depth: float, height: float, x: float, y: float, z: float) -> tuple[bool, str]:
        return self.node.spawn_box(width, depth, height, x, y, z)


def main():
    rclpy.init(args=sys.argv)
    node = TeleopNode()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(APP_STYLE)
    win = TeleopMainWindow(node)
    win.resize(1280, 860)
    win.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
