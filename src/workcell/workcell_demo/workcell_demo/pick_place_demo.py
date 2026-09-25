#!/usr/bin/env python3
"""Pick and place demo - the first behaviour of the workcell orchestrator.

Whenever a box waits at the infeed beam, the robot is idle and holds nothing:

  1. navigate to inspect_node (5, the InspectionPoint = the home pose -
     usually already there)
  2. with use_vision: ask vision's box_pose_estimator for the box top faces
     in the gripper camera, keep the BIGGEST one, and move the tool in a
     straight line from there to grasp_standoff above its centre - suction
     plate parallel to the face (tool +Z into it) and the plate's long side
     along the face's long edge (see grasp_pose_from_face). Without vision
     the approach starts straight down from inspect_node itself.
  3. straight towards the face (cartesian, along the tool +Z; straight down
     without vision) until the gripper's force pad touches it - at most
     approach_max_distance; contact = the pad's force changing by more than
     contact_force N (a guarded cartesian_move), so any box height works
  4. suction on - and wait until gripper_manager reports the grasp
  5. straight up (cartesian) lift_distance with the box
  6. navigate to the next place node (place_nodes, round robin or random)
  7. suction off
  8. navigate back to home_node and wait for the next box

Lifting the box clears the beam, so the feeding line restarts by itself
(workcell_bringup's infeed_line_controller) and the next box arrives while
this box is being placed.

Everything goes over ROS messages: motion and gripper through
shared_utils.navigation.NavigatorClient (navigator_cli's navigator_server +
gripper_manager), "box waiting" from the line's latched line/state.

ROS interface (node /workcell_demo):
  ~/state        std_msgs/String, latched: "<STATE> | <detail>"
  ~/set_enabled  std_srvs/SetBool   run / pause (a running cycle finishes first)
  ~/reset        std_srvs/Trigger   leave ERROR (after fixing the cause)
  ~/restart      std_srvs/Trigger   start over from any state: abort the running
                                    cycle at its next step, suction off (drops
                                    anything held), go home, clear ERROR, enable

A failed step puts the demo into ERROR and it stops commanding the robot -
nothing is retried blindly. If the pad touches nothing or the grasp fails,
suction goes off, the tool backs out the way it came and returns home before
stopping.
"""
import random
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from custom_msgs.msg import InfeedLineState
from shared_utils.geometry import TfHelper, matrix_to_pose, pose_to_matrix
from shared_utils.navigation import NavigatorClient
from shared_utils.perception import BoxDetectionClient

TOOL_FRAME = 'gripper_tcp'


def grasp_pose_from_face(face: np.ndarray, standoff: float, current_rotation: np.ndarray) -> np.ndarray:
    """Tool (gripper_tcp) pose, as a 4x4 matrix in the face's frame, for a
    suction grasp of a box top face.

    face: 4x4 face pose (vision's convention: +X long edge, +Y short edge,
    +Z face normal pointing away from the box). The tool's +Z points out of
    the suction cups, so it must point INTO the face (-normal); the plate's
    long side is its +Y (0.30 x 0.40 m, group_a_macro.xacro), laid along the
    face's long edge. That leaves two orientations 180 deg apart about the
    normal - the one closer to current_rotation is used, so the wrist turns
    as little as possible. The tool sits `standoff` above the face centre,
    along the normal (the guarded approach closes the gap)."""
    x_f, n = face[:3, 0], face[:3, 2]
    z_t = -n
    best = None
    for y_t in (x_f, -x_f):
        R = np.stack([np.cross(y_t, z_t), y_t, z_t], axis=1)
        turn = np.arccos(np.clip((np.trace(current_rotation.T @ R) - 1.0) / 2.0, -1.0, 1.0))
        if best is None or turn < best[0]:
            best = (turn, R)
    T = np.eye(4)
    T[:3, :3] = best[1]
    T[:3, 3] = face[:3, 3] + n * standoff
    return T


class StepFailed(Exception):
    pass


class RestartRequested(Exception):
    """Raised between steps once ~/restart was called."""


class PickPlaceDemo(Node):
    # States shown on ~/state
    STARTING, DISABLED, WAITING, PICKING, PLACING, RETURNING, ERROR = (
        'STARTING', 'DISABLED', 'WAITING', 'PICKING', 'PLACING', 'RETURNING', 'ERROR')

    def __init__(self):
        super().__init__('workcell_demo')
        p = self.declare_parameter
        p('robot_namespace', 'robot_1')
        p('line_namespace', 'conveyor_package_infeed')
        p('place_nodes', ['2', '3', '4'])
        p('place_order', 'round_robin')
        p('home_node', '5')
        p('place_speed', 1.0)           # cuMotion time dilation for the moves to the place nodes (1.0 = full speed)
        p('return_speed', 1.0)          # cuMotion time dilation for place node -> home after placing (1.0 = full speed)
        p('nav_speed', 0.75)            # cuMotion time dilation for every other navigation (inspect, home)
        p('inspect_node', '5')          # every pick starts here (vision looks from here)
        p('use_vision', True)
        p('grasp_standoff', 0.10)
        p('align_speed', 0.075)
        p('detect_timeout', 15.0)
        p('approach_max_distance', 0.30)
        p('approach_speed', 0.015)
        p('contact_force', 5.0)
        p('lift_distance', 0.30)
        p('lift_speed', 0.075)
        p('grasp_wait_sec', 3.0)
        p('start_enabled', True)
        p('go_home_on_start', True)

        self.nav = NavigatorClient(self, namespace=self._p('robot_namespace'))
        self.boxes = BoxDetectionClient(self, namespace=self._p('robot_namespace'))
        self.tf = TfHelper(self, f"/{self._p('robot_namespace')}/tf")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._line: InfeedLineState = None
        self.create_subscription(InfeedLineState, f"/{self._p('line_namespace')}/line/state",
                                 self._on_line_state, latched)
        self._state_pub = self.create_publisher(String, '~/state', latched)
        self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)
        self.create_service(Trigger, '~/reset', self._on_reset)
        self.create_service(Trigger, '~/restart', self._on_restart)
        self._restart = threading.Event()

        self._enabled = bool(self._p('start_enabled'))
        self._error = False
        self._state = None
        self._place_index = 0
        self.cycles = 0
        self._set_state(self.STARTING, 'waiting for the navigator')

    def _p(self, name):
        return self.get_parameter(name).value

    # ── ROS callbacks ───────────────────────────────────────────────────────

    def _on_line_state(self, msg: InfeedLineState) -> None:
        self._line = msg

    def _on_set_enabled(self, request, response):
        self._enabled = bool(request.data)
        response.success = True
        response.message = 'demo enabled' if self._enabled else 'demo paused (a running cycle finishes first)'
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, request, response):
        response.success = self._error
        response.message = 'error cleared' if self._error else 'not in ERROR'
        self._error = False
        return response

    def _on_restart(self, request, response):
        self._restart.set()
        response.success = True
        response.message = ('restart requested - the running step finishes, then suction off, '
                            'home, and the demo runs again')
        self.get_logger().info(response.message)
        return response

    def _check_restart(self) -> None:
        if self._restart.is_set():
            raise RestartRequested()

    def _do_restart(self) -> None:
        """Suction off, home, clear ERROR, enable. A failure here is an ERROR again."""
        self._restart.clear()
        self._error = False
        self._enabled = True
        self._set_state(self.RETURNING, 'restart: suction off')
        self.nav.gripper(False)
        try:
            self._navigate(self._p('home_node'), self.RETURNING, 'restart')
        except (StepFailed, RestartRequested) as exc:
            if isinstance(exc, RestartRequested):
                return self._do_restart()
            self._error = True
            self._set_state(self.ERROR, f'restart: {exc} - fix it, then call ~/restart')
            return
        self._set_state(self.WAITING, 'restarted')

    def _set_state(self, state: str, detail: str = '') -> None:
        text = f'{state} | {detail}' if detail else state
        if text != self._state:
            self._state = text
            self._state_pub.publish(String(data=text))
            self.get_logger().info(text)

    # ── Conditions ──────────────────────────────────────────────────────────

    def box_waiting(self) -> bool:
        return self._line is not None and self._line.mode == InfeedLineState.MODE_BLOCKED

    def next_place_node(self) -> str:
        nodes = list(self._p('place_nodes'))
        if self._p('place_order') == 'random':
            return random.choice(nodes)
        node = nodes[self._place_index % len(nodes)]
        self._place_index += 1
        return node

    # ── Steps ───────────────────────────────────────────────────────────────

    def _navigate(self, node: str, state: str, why: str, speed: float = 0.0) -> None:
        """speed: cuMotion time dilation (0, 1]; 0 = nav_speed."""
        self._check_restart()
        self._set_state(state, f'{why}: navigating to node {node}')
        r = self.nav.navigate(node, speed=speed or float(self._p('nav_speed')))
        if not r.success:
            raise StepFailed(f'navigate to node {node} failed: {r.message}')

    def _align_over_box(self, state: str) -> None:
        """Vision: biggest box top face -> tool pose grasp_standoff above it -> straight line there."""
        self._check_restart()
        self._set_state(state, 'vision: detecting boxes')
        r = self.boxes.detect(timeout_sec=float(self._p('detect_timeout')))
        if not r.success:
            raise StepFailed(f'vision: {r.message}')
        if not r.boxes:
            raise StepFailed(f'vision: no box in view ({r.message})')
        box = max(r.boxes, key=lambda b: b.area)
        frame = box.pose.header.frame_id
        current = self.tf.frame_pose(frame, TOOL_FRAME)
        if current is None:
            raise StepFailed(f'vision: no TF {frame} -> {TOOL_FRAME}')
        T = grasp_pose_from_face(pose_to_matrix(box.pose.pose), float(self._p('grasp_standoff')),
                                 pose_to_matrix(current.pose)[:3, :3])
        target = PoseStamped()
        target.header.frame_id = frame
        target.pose = matrix_to_pose(T)
        c = box.pose.pose.position
        self._set_state(state, f'vision: biggest of {len(r.boxes)}: {box.size[0] * 100:.0f}x'
                               f'{box.size[1] * 100:.0f} cm at ({c.x:+.2f}, {c.y:+.2f}, {c.z:+.2f}) - aligning')
        self._check_restart()
        m = self.nav.move_to_pose(target, speed=float(self._p('align_speed')))
        if not m.success:
            raise StepFailed(f'align over the box: {m.message}')

    def _approach_until_contact(self, state: str, frame: str) -> float:
        """Towards the box until the force pad touches - along the tool +Z
        (frame 'tool') or straight down ('world'); returns the distance travelled."""
        self._check_restart()
        d, force = float(self._p('approach_max_distance')), float(self._p('contact_force'))
        offset = (0.0, 0.0, d) if frame == 'tool' else (0.0, 0.0, -d)
        self._set_state(state, f'approach: until contact ({force:.0f} N, max {d * 100:.0f} cm)')
        r = self.nav.cartesian(offset, frame=frame, speed=self._p('approach_speed'), stop_force=force)
        if not r.contact:
            if r.distance > 0.0:
                self._back_out(r.distance, frame, state, 'no contact - backing out')
            raise StepFailed(f'approach: {r.message}')
        self._set_state(state, f'approach: {r.message}')
        return r.distance

    def _back_out(self, distance: float, frame: str, state: str, why: str) -> None:
        """Undo an approach of `distance` made along the tool +Z / straight down."""
        if frame == 'world':
            self._straight(+distance, state, why)
            return
        self._check_restart()
        self._set_state(state, f'{why}: {distance * 100:.0f} cm back along the tool axis')
        r = self.nav.cartesian((0.0, 0.0, -distance), frame='tool', speed=self._p('approach_speed'))
        if not r.success:
            raise StepFailed(f'backing out failed: {r.message}')

    def _straight(self, dz: float, state: str, why: str, speed: float = None) -> None:
        self._check_restart()
        self._set_state(state, f'{why}: straight {"down" if dz < 0 else "up"} {abs(dz) * 100:.0f} cm')
        r = self.nav.cartesian((0.0, 0.0, dz), frame='world',
                               speed=self._p('approach_speed') if speed is None else speed)
        if not r.success:
            raise StepFailed(f'cartesian move {dz:+.2f} m failed: {r.message}')

    def _gripper(self, on: bool, state: str) -> None:
        self._check_restart()
        self._set_state(state, f'suction {"on" if on else "off"}')
        r = self.nav.gripper(on)
        if not r.success:
            raise StepFailed(f'suction {"on" if on else "off"} failed: {r.message}')

    def _wait_for_grasp(self) -> list:
        deadline = time.monotonic() + self._p('grasp_wait_sec')
        while time.monotonic() < deadline:
            self._check_restart()
            held = self.nav.holding()
            if held:
                return held
            time.sleep(0.1)
        return []

    def run_cycle(self) -> None:
        home, start = self._p('home_node'), self._p('inspect_node')
        self._navigate(start, self.PICKING, 'box at the beam - inspecting')
        frame = 'world'
        if self._p('use_vision'):
            self._align_over_box(self.PICKING)
            frame = 'tool'
        d = self._approach_until_contact(self.PICKING, frame)
        self._gripper(True, self.PICKING)
        held = self._wait_for_grasp()
        if not held:
            # Nothing within suction reach: back out cleanly, then stop.
            self.nav.gripper(False)
            self._back_out(d, frame, self.PICKING, 'no grasp - backing out')
            self._navigate(home, self.RETURNING, 'no grasp')
            raise StepFailed(f'pad touched something {d * 100:.0f} cm into the approach from node {start} '
                             f'but no box was grasped (touched the belt, or the box is off-centre?)')
        self._straight(+float(self._p('lift_distance')), self.PICKING, f'lifting {", ".join(held)}',
                       speed=float(self._p('lift_speed')))

        place = self.next_place_node()
        self._navigate(place, self.PLACING, f'carrying {", ".join(held)}',
                       speed=float(self._p('place_speed')))
        self._gripper(False, self.PLACING)
        self._navigate(home, self.RETURNING, 'placed', speed=float(self._p('return_speed')))
        self.cycles += 1

    # ── Main loop (worker thread; ROS spins on the executor) ───────────────

    def loop(self) -> None:
        if not self.nav.wait_until_ready(timeout_sec=60.0):
            self._error = True
            self._set_state(self.ERROR, 'navigator_server not available (navigator_server.launch.py)')
        elif self._p('go_home_on_start') and not self.nav.holding():
            try:
                self._navigate(self._p('home_node'), self.RETURNING, 'start')
            except StepFailed as exc:
                self._error = True
                self._set_state(self.ERROR, str(exc))

        while rclpy.ok():
            if self._restart.is_set():
                self._do_restart()
                continue
            if self._error:
                time.sleep(0.2)  # keep the ERROR state until ~/reset
                continue
            if not self._enabled:
                self._set_state(self.DISABLED, 'call ~/set_enabled true to run')
            elif self._line is None:
                self._set_state(self.WAITING, 'no line/state yet (infeed_line.launch.py running?)')
            elif self.nav.holding():
                self._set_state(self.WAITING, f'holding {", ".join(self.nav.holding())} - not starting a pick')
            elif not self.box_waiting():
                self._set_state(self.WAITING, f'for a box at the beam ({self.cycles} placed)')
            else:
                try:
                    self.run_cycle()
                    self._set_state(self.WAITING, f'cycle {self.cycles} done')
                except RestartRequested:
                    pass  # handled at the top of the loop
                except StepFailed as exc:
                    self._error = True
                    self._set_state(self.ERROR, f'{exc} - fix it, then call ~/reset')
                continue
            time.sleep(0.2)


def main():
    rclpy.init()
    node = PickPlaceDemo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=node.loop, daemon=True).start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
