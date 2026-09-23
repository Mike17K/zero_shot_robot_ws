#!/usr/bin/env python3
"""PyQt teleop UI for the workcell: joint-position sliders per robot (sent as
single-point JointTrajectory messages straight to
<namespace>/gp70l_joint_trajectory_controller/joint_trajectory - the same
topic interface MoveIt's own trajectory execution uses, just fed directly
instead of through planning; "Refresh from robot" pulls the sliders to the
actual joint states without moving anything, and happens once automatically
on the first joint state), a gripper ON/OFF toggle per robot, speed
sliders per conveyor (sent as std_msgs/Float64 on <namespace>/target_speed,
picked up by each belt's own belt_speed_relay - see
conveyor_bringup/scripts/belt_speed_relay.py - which fans it out to that
belt's actual roller count), and a "Spawn Box" button with adjustable
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

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from PyQt5 import QtCore, QtWidgets
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from custom_msgs.msg import BoxFactoryStatus, InfeedLineState
from custom_msgs.srv import SpawnBox

MOVE_TIME_SEC = 0.3  # time_from_start for each jogged trajectory point
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


def _load_joint_limits(package_name: str, joint_names: list[str]) -> dict[str, tuple[float, float]]:
    path = os.path.join(get_package_share_directory(package_name), 'config', 'joint_limits.yaml')
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    joint_limits = data.get('joint_limits', {})
    limits = {}
    for name in joint_names:
        entry = joint_limits.get(name, {})
        limits[name] = (float(entry.get('min_position', -math.pi)), float(entry.get('max_position', math.pi)))
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
        for ns in self.robot_namespaces:
            self.create_subscription(
                JointState, f'/{ns}/joint_states',
                lambda msg, ns=ns: self._joint_states.__setitem__(ns, msg), 1)

        line_ns = self.get_parameter('infeed_line_namespace').value
        self.infeed_line_namespace = line_ns
        self.line_state: Optional[InfeedLineState] = None
        self.line_state_stamp = 0.0  # wall time.monotonic() of the last state
        self.create_subscription(
            InfeedLineState, f'/{line_ns}/line/state', self._on_line_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._line_enable_client = self.create_client(SetBool, f'/{line_ns}/line/set_enabled')
        self._line_release_client = self.create_client(Trigger, f'/{line_ns}/line/release')

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
            on_done(False, f'{client.srv_name} unavailable (infeed_line.launch.py not running?)')
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

    def send_joint_positions(self, robot_ns: str, positions: list[float]) -> None:
        pub = self._traj_pubs.get(robot_ns)
        if pub is None:
            return
        msg = JointTrajectory()
        msg.joint_names = list(self.robot_joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        sec = int(MOVE_TIME_SEC)
        point.time_from_start = Duration(sec=sec, nanosec=int((MOVE_TIME_SEC - sec) * 1e9))
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


class JointSliderRow(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal()
    _STEPS = 1000

    def __init__(self, joint_name: str, lo: float, hi: float, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.lo = lo
        self.hi = hi if hi > lo else lo + 1.0
        self._rad = min(max(0.0, self.lo), self.hi)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        name_label = QtWidgets.QLabel(joint_name)
        name_label.setMinimumWidth(100)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self._STEPS)
        self.value_label = QtWidgets.QLabel()
        self.value_label.setMinimumWidth(80)
        layout.addWidget(name_label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.value_label)

        self.set_rad(self._rad)
        self.slider.valueChanged.connect(self._on_slider_changed)

    def _step_to_rad(self, step: int) -> float:
        return self.lo + (step / self._STEPS) * (self.hi - self.lo)

    def _rad_to_step(self, rad: float) -> int:
        frac = (rad - self.lo) / (self.hi - self.lo)
        return int(round(min(1.0, max(0.0, frac)) * self._STEPS))

    def _on_slider_changed(self, step: int) -> None:
        self._rad = self._step_to_rad(step)
        self.value_label.setText(f'{math.degrees(self._rad):6.1f} deg')
        self.changed.emit()

    def set_rad(self, rad: float, emit: bool = True) -> None:
        """emit=False only moves the slider (no `changed`, so no command)."""
        step = self._rad_to_step(rad)
        self.slider.blockSignals(True)
        self.slider.setValue(step)
        self.slider.blockSignals(False)
        self._rad = self._step_to_rad(step)
        self.value_label.setText(f'{math.degrees(self._rad):6.1f} deg')
        if emit:
            self.changed.emit()

    def get_rad(self) -> float:
        return self._rad


class RobotPanel(QtWidgets.QGroupBox):
    def __init__(self, robot_ns: str, joint_names: list[str], joint_limits: dict, on_change, on_gripper,
                 read_joints, parent=None):
        super().__init__(f'Robot: {robot_ns}', parent)
        self.robot_ns = robot_ns
        self.joint_names = joint_names
        self.on_change = on_change
        self.on_gripper = on_gripper
        self.read_joints = read_joints  # () -> Optional[list[float]], the robot's actual joints
        self.synced_once = False
        self.rows: dict[str, JointSliderRow] = {}

        layout = QtWidgets.QVBoxLayout(self)
        for name in joint_names:
            lo, hi = joint_limits.get(name, (-math.pi, math.pi))
            row = JointSliderRow(name, lo, hi)
            row.changed.connect(self._on_row_changed)
            layout.addWidget(row)
            self.rows[name] = row

        buttons = QtWidgets.QHBoxLayout()
        home_btn = QtWidgets.QPushButton('Home (all 0)')
        home_btn.clicked.connect(self._go_home)
        buttons.addWidget(home_btn)
        # Pull the sliders to where the arm really is - e.g. after
        # navigator_cli or MoveIt moved it - so the next slider touch does
        # not make the arm jump back to stale slider values.
        refresh_btn = QtWidgets.QPushButton('Refresh from robot')
        refresh_btn.setToolTip('Set the sliders to the current joint states (does not move the arm)')
        refresh_btn.clicked.connect(self.refresh_from_robot)
        buttons.addWidget(refresh_btn)
        buttons.addStretch(1)

        self.gripper_btn = QtWidgets.QPushButton()
        self.gripper_btn.setCheckable(True)
        self.gripper_btn.toggled.connect(self._on_gripper_toggled)
        # Starts OFF: suction is now a latching capture state, so leaving it
        # ON at startup would mean the arm grabs anything that drifts into
        # its footprint before the operator asked for it. Nothing is attached
        # to the arm at startup anymore either - no grasping plugin exists in
        # the URDF at all now (see gripper_manager.py).
        self._set_gripper_style(False)
        buttons.addWidget(self.gripper_btn)

        layout.addLayout(buttons)

        self.gripper_status_label = QtWidgets.QLabel('')
        self.gripper_status_label.setWordWrap(True)
        layout.addWidget(self.gripper_status_label)

    def _on_row_changed(self) -> None:
        self.on_change(self.robot_ns, self.current_positions())

    def current_positions(self) -> list[float]:
        return [self.rows[name].get_rad() for name in self.joint_names]

    def _go_home(self) -> None:
        for row in self.rows.values():
            row.set_rad(0.0, emit=False)
        self._on_row_changed()  # one command for all joints

    def refresh_from_robot(self) -> bool:
        positions = self.read_joints()
        if positions is None:
            self.gripper_status_label.setText(f'FAILED: no joint states on /{self.robot_ns}/joint_states yet')
            return False
        for name, rad in zip(self.joint_names, positions):
            self.rows[name].set_rad(rad, emit=False)
        self.synced_once = True
        self.gripper_status_label.setText('OK: sliders set to the current joint states')
        return True

    def _on_gripper_toggled(self, checked: bool) -> None:
        self._set_gripper_style(checked)
        ok, message = self.on_gripper(self.robot_ns, checked)
        self.gripper_status_label.setText(('OK: ' if ok else 'FAILED: ') + message)
        if not ok:
            # Attach was refused (too far/no box) - reflect that the
            # gripper is NOT actually holding anything, rather than leaving
            # the button showing a successful "ON" it didn't achieve.
            self.gripper_btn.blockSignals(True)
            self.gripper_btn.setChecked(False)
            self.gripper_btn.blockSignals(False)
            self._set_gripper_style(False)

    def _set_gripper_style(self, on: bool) -> None:
        self.gripper_btn.setText('Gripper: ON' if on else 'Gripper: OFF')
        color = '#2e7d32' if on else '#616161'
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
        self.release_btn = QtWidgets.QPushButton('Release box (restart line)')
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
        self.enable_btn.setText('Line: ON (click to stop)' if enabled else 'Line: OFF (click to start)')

    def _report(self, ok: bool, message: str) -> None:
        self.status_label.setText(('OK: ' if ok else 'FAILED: ') + message)

    def _on_enable_clicked(self, checked: bool) -> None:
        self.node.set_line_enabled(checked, self._report)

    def _on_release_clicked(self) -> None:
        self.node.release_line(self._report)


class TeleopMainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('Workcell Teleop')

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QHBoxLayout(central)

        robots_col = QtWidgets.QVBoxLayout()
        robots_col.addWidget(QtWidgets.QLabel('<b>Robots</b>'))
        self._robot_panels: list[RobotPanel] = []
        for ns in node.robot_namespaces:
            panel = RobotPanel(ns, node.robot_joint_names, node.joint_limits, self._on_robot_changed,
                               self._on_gripper_toggled, lambda ns=ns: node.current_joint_positions(ns))
            robots_col.addWidget(panel)
            self._robot_panels.append(panel)
        robots_col.addStretch(1)

        conveyors_col = QtWidgets.QVBoxLayout()
        conveyors_col.addWidget(QtWidgets.QLabel('<b>Conveyors</b>'))
        for ns in node.conveyor_namespaces:
            panel = ConveyorPanel(ns, node.conveyor_max_speed, self._on_conveyor_changed)
            conveyors_col.addWidget(panel)
        conveyors_col.addStretch(1)

        scene_col = QtWidgets.QVBoxLayout()
        scene_col.addWidget(QtWidgets.QLabel('<b>Scene</b>'))
        scene_col.addWidget(SpawnPanel(self._on_spawn_box, node.spawn_x, node.spawn_y, node.spawn_z))
        self._line_panel = InfeedLinePanel(self.node)
        scene_col.addWidget(self._line_panel)
        self._factory_panel = FactoryPanel(self.node.configure_factory)
        scene_col.addWidget(self._factory_panel)
        scene_col.addStretch(1)

        outer.addLayout(robots_col, 2)
        outer.addLayout(conveyors_col, 1)
        outer.addLayout(scene_col, 1)

        # Nothing here subscribes to anything (pure command UI), but pumping
        # the executor completes box_factory's async set_parameters replies
        # (FactoryPanel status) and keeps the node responsive to ROS-side
        # events (parameter changes, discovery) without blocking Qt's own
        # event loop - spin_once/spin can't run in the same thread as exec_().
        self._ros_timer = QtCore.QTimer(self)
        self._ros_timer.timeout.connect(self._on_ros_tick)
        self._ros_timer.start(50)

    def _on_ros_tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0)
        self._line_panel.refresh()
        fresh = time.monotonic() - self.node.factory_status_stamp < 3.0
        self._factory_panel.refresh(self.node.factory_status if fresh else None)
        # Sliders start at 0 while the arm is wherever it is - sync them once
        # as soon as each robot's first joint state arrives.
        for panel in self._robot_panels:
            if not panel.synced_once and self.node.current_joint_positions(panel.robot_ns) is not None:
                panel.refresh_from_robot()

    def _on_robot_changed(self, robot_ns: str, positions: list[float]) -> None:
        self.node.send_joint_positions(robot_ns, positions)

    def _on_conveyor_changed(self, conveyor_ns: str, speed: float) -> None:
        self.node.send_belt_speed(conveyor_ns, speed)

    def _on_gripper_toggled(self, robot_ns: str, on: bool) -> tuple[bool, str]:
        return self.node.set_gripper(robot_ns, on)

    def _on_spawn_box(self, width: float, depth: float, height: float, x: float, y: float, z: float) -> tuple[bool, str]:
        return self.node.spawn_box(width, depth, height, x, y, z)


def main():
    rclpy.init(args=sys.argv)
    node = TeleopNode()

    app = QtWidgets.QApplication(sys.argv)
    win = TeleopMainWindow(node)
    win.resize(1150, 850)
    win.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
