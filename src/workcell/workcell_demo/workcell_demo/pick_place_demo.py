#!/usr/bin/env python3
"""Pick and place demo - the first behaviour of the workcell orchestrator.

Whenever a box waits at the infeed beam, the robot is idle and holds nothing:

  1. navigate to pick_node (above the beam)
  2. straight down approach_distance
  3. suction on - and wait until gripper_manager reports the grasp
  4. straight up approach_distance
  5. navigate to the next place node (place_nodes, round robin or random)
  6. suction off
  7. navigate back to home_node and wait for the next box

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

A failed step puts the demo into ERROR and it stops commanding the robot -
nothing is retried blindly. If the grasp fails, suction goes off, the tool
backs out and returns home before stopping.
"""
import random
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from custom_msgs.msg import InfeedLineState
from shared_utils.navigation import NavigatorClient


class StepFailed(Exception):
    pass


class PickPlaceDemo(Node):
    # States shown on ~/state
    STARTING, DISABLED, WAITING, PICKING, PLACING, RETURNING, ERROR = (
        'STARTING', 'DISABLED', 'WAITING', 'PICKING', 'PLACING', 'RETURNING', 'ERROR')

    def __init__(self):
        super().__init__('workcell_demo')
        p = self.declare_parameter
        p('robot_namespace', 'robot_1')
        p('line_namespace', 'conveyor_package_infeed')
        p('pick_node', '6')
        p('place_nodes', ['2', '3', '4'])
        p('place_order', 'round_robin')
        p('home_node', '5')
        p('approach_distance', 0.10)
        p('approach_speed', 0.05)
        p('grasp_wait_sec', 3.0)
        p('start_enabled', True)
        p('go_home_on_start', True)

        self.nav = NavigatorClient(self, namespace=self._p('robot_namespace'))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._line: InfeedLineState = None
        self.create_subscription(InfeedLineState, f"/{self._p('line_namespace')}/line/state",
                                 self._on_line_state, latched)
        self._state_pub = self.create_publisher(String, '~/state', latched)
        self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)
        self.create_service(Trigger, '~/reset', self._on_reset)

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

    def _navigate(self, node: str, state: str, why: str) -> None:
        self._set_state(state, f'{why}: navigating to node {node}')
        r = self.nav.navigate(node)
        if not r.success:
            raise StepFailed(f'navigate to node {node} failed: {r.message}')

    def _straight(self, dz: float, state: str, why: str) -> None:
        self._set_state(state, f'{why}: straight {"down" if dz < 0 else "up"} {abs(dz) * 100:.0f} cm')
        r = self.nav.cartesian((0.0, 0.0, dz), frame='world', speed=self._p('approach_speed'))
        if not r.success:
            raise StepFailed(f'cartesian move {dz:+.2f} m failed: {r.message}')

    def _gripper(self, on: bool, state: str) -> None:
        self._set_state(state, f'suction {"on" if on else "off"}')
        r = self.nav.gripper(on)
        if not r.success:
            raise StepFailed(f'suction {"on" if on else "off"} failed: {r.message}')

    def _wait_for_grasp(self) -> list:
        deadline = time.monotonic() + self._p('grasp_wait_sec')
        while time.monotonic() < deadline:
            held = self.nav.holding()
            if held:
                return held
            time.sleep(0.1)
        return []

    def run_cycle(self) -> None:
        d = float(self._p('approach_distance'))
        pick, home = self._p('pick_node'), self._p('home_node')

        self._navigate(pick, self.PICKING, 'box at the beam')
        self._straight(-d, self.PICKING, 'approach')
        self._gripper(True, self.PICKING)
        held = self._wait_for_grasp()
        if not held:
            # Nothing within suction reach: back out cleanly, then stop.
            self.nav.gripper(False)
            self._straight(+d, self.PICKING, 'no grasp - backing out')
            self._navigate(home, self.RETURNING, 'no grasp')
            raise StepFailed(f'no box grasped {d * 100:.0f} cm below node {pick} '
                             f'(tool too high or too low for this box?)')
        # self._straight(+d, self.PICKING, f'lifting {", ".join(held)}')

        place = self.next_place_node()
        self._navigate(place, self.PLACING, f'carrying {", ".join(held)}')
        self._gripper(False, self.PLACING)
        self._navigate(home, self.RETURNING, 'placed')
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
