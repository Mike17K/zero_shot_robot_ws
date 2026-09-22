#!/usr/bin/env python3
"""PyQt teleop UI for the workcell: joint-position sliders per robot (sent as
single-point JointTrajectory messages straight to
<namespace>/gp70l_joint_trajectory_controller/joint_trajectory - the same
topic interface MoveIt's own trajectory execution uses, just fed directly
instead of through planning), a gripper ON/OFF toggle per robot, speed
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
"""
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from PyQt5 import QtCore, QtWidgets
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

        self.robot_namespaces: list[str] = list(self.get_parameter('robot_namespaces').value)
        self.robot_joint_names: list[str] = list(self.get_parameter('robot_joint_names').value)
        joint_limits_package: str = self.get_parameter('joint_limits_package').value
        self.conveyor_namespaces: list[str] = list(self.get_parameter('conveyor_namespaces').value)
        self.conveyor_max_speed: float = float(self.get_parameter('conveyor_max_speed').value)
        self.spawn_x: float = float(self.get_parameter('spawn_x').value)
        self.spawn_y: float = float(self.get_parameter('spawn_y').value)
        self.spawn_z: float = float(self.get_parameter('spawn_z').value)

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

    def set_rad(self, rad: float) -> None:
        step = self._rad_to_step(rad)
        self.slider.blockSignals(True)
        self.slider.setValue(step)
        self.slider.blockSignals(False)
        self._on_slider_changed(step)

    def get_rad(self) -> float:
        return self._rad


class RobotPanel(QtWidgets.QGroupBox):
    def __init__(self, robot_ns: str, joint_names: list[str], joint_limits: dict, on_change, on_gripper, parent=None):
        super().__init__(f'Robot: {robot_ns}', parent)
        self.robot_ns = robot_ns
        self.joint_names = joint_names
        self.on_change = on_change
        self.on_gripper = on_gripper
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
            row.set_rad(0.0)
        self._on_row_changed()

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
        for ns in node.robot_namespaces:
            panel = RobotPanel(ns, node.robot_joint_names, node.joint_limits, self._on_robot_changed, self._on_gripper_toggled)
            robots_col.addWidget(panel)
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
        scene_col.addStretch(1)

        outer.addLayout(robots_col, 2)
        outer.addLayout(conveyors_col, 1)
        outer.addLayout(scene_col, 1)

        # Nothing here currently subscribes to anything (pure command UI),
        # but pumping the executor keeps the node responsive to ROS-side
        # events (parameter changes, discovery) without blocking Qt's own
        # event loop - spin_once/spin can't run in the same thread as exec_().
        self._ros_timer = QtCore.QTimer(self)
        self._ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self._ros_timer.start(50)

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
    win.resize(1150, 650)
    win.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
