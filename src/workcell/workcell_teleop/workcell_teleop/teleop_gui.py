#!/usr/bin/env python3
"""PyQt teleop UI for the workcell: joint-position sliders per robot (sent as
single-point JointTrajectory messages straight to
<namespace>/gp70l_joint_trajectory_controller/joint_trajectory - the same
topic interface MoveIt's own trajectory execution uses, just fed directly
instead of through planning) and speed sliders per conveyor (sent as
std_msgs/Float64 on <namespace>/target_speed, picked up by each belt's own
belt_speed_relay - see conveyor_bringup/scripts/belt_speed_relay.py - which
fans it out to that belt's actual roller count).

Robot namespaces/joint names and conveyor namespaces are ROS parameters
(set from launch/teleop.launch.py, defaulted here to match
workcell_bringup/launch/workcell.launch.py's current robots_config/
conveyors_config) rather than hardcoded twice - keep those two files' lists
in sync by hand if the workcell layout changes. Joint slider ranges are read
directly from group_a_moveit_config/config/joint_limits.yaml at startup
(also not duplicated here).
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MOVE_TIME_SEC = 0.3  # time_from_start for each jogged trajectory point


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

        self.robot_namespaces: list[str] = list(self.get_parameter('robot_namespaces').value)
        self.robot_joint_names: list[str] = list(self.get_parameter('robot_joint_names').value)
        joint_limits_package: str = self.get_parameter('joint_limits_package').value
        self.conveyor_namespaces: list[str] = list(self.get_parameter('conveyor_namespaces').value)
        self.conveyor_max_speed: float = float(self.get_parameter('conveyor_max_speed').value)

        self.joint_limits = _load_joint_limits(joint_limits_package, self.robot_joint_names)

        self._traj_pubs = {
            ns: self.create_publisher(JointTrajectory, f'/{ns}/gp70l_joint_trajectory_controller/joint_trajectory', 10)
            for ns in self.robot_namespaces
        }
        self._speed_pubs = {
            ns: self.create_publisher(Float64, f'/{ns}/target_speed', 10)
            for ns in self.conveyor_namespaces
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
    def __init__(self, robot_ns: str, joint_names: list[str], joint_limits: dict, on_change, parent=None):
        super().__init__(f'Robot: {robot_ns}', parent)
        self.robot_ns = robot_ns
        self.joint_names = joint_names
        self.on_change = on_change
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
        layout.addLayout(buttons)

    def _on_row_changed(self) -> None:
        self.on_change(self.robot_ns, self.current_positions())

    def current_positions(self) -> list[float]:
        return [self.rows[name].get_rad() for name in self.joint_names]

    def _go_home(self) -> None:
        for row in self.rows.values():
            row.set_rad(0.0)
        self._on_row_changed()


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
            panel = RobotPanel(ns, node.robot_joint_names, node.joint_limits, self._on_robot_changed)
            robots_col.addWidget(panel)
        robots_col.addStretch(1)

        conveyors_col = QtWidgets.QVBoxLayout()
        conveyors_col.addWidget(QtWidgets.QLabel('<b>Conveyors</b>'))
        for ns in node.conveyor_namespaces:
            panel = ConveyorPanel(ns, node.conveyor_max_speed, self._on_conveyor_changed)
            conveyors_col.addWidget(panel)
        conveyors_col.addStretch(1)

        outer.addLayout(robots_col, 2)
        outer.addLayout(conveyors_col, 1)

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


def main():
    rclpy.init(args=sys.argv)
    node = TeleopNode()

    app = QtWidgets.QApplication(sys.argv)
    win = TeleopMainWindow(node)
    win.resize(900, 600)
    win.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
