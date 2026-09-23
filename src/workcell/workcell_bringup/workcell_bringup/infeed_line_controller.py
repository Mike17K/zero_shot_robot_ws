#!/usr/bin/env python3
"""Feeding-line controller for the infeed belt: a laser beam across the belt
in front of the robot stops the line when a box reaches it, and restarts it
once the box has been picked up.

Runs in the belt's namespace (conveyor_package_infeed) and sits between speed
requests and the belt:

  target_speed  (std_msgs/Float64, in)   requested speed, e.g. the teleop slider
  line_speed    (std_msgs/Float64, out)  what the belt's belt_speed_relay follows
  line/beam     (sensor_msgs/LaserScan)  the beam (gz gpu_lidar via ros_gz_bridge)
  line/state    (custom_msgs/InfeedLineState, latched)  mode, beam, speeds
  line/set_enabled (std_srvs/SetBool)    operator line on/off
  line/release     (std_srvs/Trigger)    restart a blocked line, ignoring the
                                          box at the beam until the beam clears

Modes (InfeedLineState.MODE_*): DISABLED -> belt 0; RUNNING -> belt at the
requested speed; BLOCKED (beam crossed) -> belt 0 until the beam is clear
again, i.e. the robot took the box; RELEASED -> belt runs while the box
passes, back to RUNNING once the beam clears.

conveyor_bringup's box_factory follows line/state: it only spawns while the
line is RUNNING or RELEASED.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

from custom_msgs.msg import InfeedLineState

TICK_SEC = 0.05
STATE_PERIOD_SEC = 0.5     # state is republished at least this often
COMMAND_PERIOD_SEC = 1.0   # the belt command too (a restarted relay picks it up)


class InfeedLineController(Node):
    def __init__(self):
        super().__init__('infeed_line')
        # Speed used until the first target_speed request arrives (rad/s,
        # same sign convention as the belt's own belt_speed launch arg).
        self.declare_parameter('running_speed', 6.0)
        # Beam counts as blocked when it stops closer than this - the opposite
        # rail sits at the belt width, so this is a bit less than that.
        self.declare_parameter('beam_clear_range', 0.44)
        self.declare_parameter('block_samples', 2)   # consecutive blocked scans to stop
        self.declare_parameter('clear_samples', 6)   # consecutive clear scans to restart
        self.declare_parameter('beam_timeout_sec', 2.0)
        self.declare_parameter('start_enabled', True)

        self._speed_request = float(self.get_parameter('running_speed').value)
        self._clear_range = float(self.get_parameter('beam_clear_range').value)
        self._block_n = int(self.get_parameter('block_samples').value)
        self._clear_n = int(self.get_parameter('clear_samples').value)
        self._beam_timeout = float(self.get_parameter('beam_timeout_sec').value)

        self._enabled = bool(self.get_parameter('start_enabled').value)
        self._mode = InfeedLineState.MODE_RUNNING if self._enabled else InfeedLineState.MODE_DISABLED
        self._beam_blocked = False
        self._beam_range = math.nan
        self._last_scan = None  # node-clock time of the last scan
        self._hits = self._misses = 0
        self._boxes_stopped = 0
        self._command = None
        self._last_command_pub = self._last_state_pub = None
        self._last_state = None

        self._speed_pub = self.create_publisher(Float64, 'line_speed', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(InfeedLineState, 'line/state', latched)
        self.create_subscription(Float64, 'target_speed', self._on_request, 10)
        self.create_subscription(LaserScan, 'line/beam', self._on_scan, qos_profile_sensor_data)
        self.create_service(SetBool, 'line/set_enabled', self._on_set_enabled)
        self.create_service(Trigger, 'line/release', self._on_release)
        self.create_timer(TICK_SEC, self._on_tick)
        self.get_logger().info(
            f'infeed line ready: speed={self._speed_request} rad/s, beam blocks below '
            f'{self._clear_range:.2f} m, enabled={self._enabled}')

    # ── Inputs ──────────────────────────────────────────────────────────────

    def _on_request(self, msg: Float64) -> None:
        self._speed_request = float(msg.data)

    def _on_scan(self, msg: LaserScan) -> None:
        finite = [r for r in msg.ranges if math.isfinite(r)]
        self._beam_range = min(finite) if finite else math.inf
        hit = any(msg.range_min <= r < self._clear_range for r in finite)
        self._last_scan = self.get_clock().now()
        # Debounce: a single noisy ray neither stops nor restarts the line.
        if hit:
            self._hits, self._misses = self._hits + 1, 0
            if self._hits >= self._block_n:
                self._beam_blocked = True
        else:
            self._hits, self._misses = 0, self._misses + 1
            if self._misses >= self._clear_n:
                self._beam_blocked = False

    def _beam_ok(self) -> bool:
        if self._last_scan is None:
            return False
        return (self.get_clock().now() - self._last_scan).nanoseconds * 1e-9 < self._beam_timeout

    def _on_set_enabled(self, request, response):
        self._enabled = bool(request.data)
        if not self._enabled:
            self._mode = InfeedLineState.MODE_DISABLED
        elif self._mode == InfeedLineState.MODE_DISABLED:
            self._mode = InfeedLineState.MODE_RUNNING
        self._on_tick()
        response.success = True
        response.message = f"infeed line {'enabled' if self._enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def _on_release(self, request, response):
        if self._mode != InfeedLineState.MODE_BLOCKED:
            response.success = False
            response.message = 'line is not blocked - nothing to release'
            return response
        self._mode = InfeedLineState.MODE_RELEASED
        self._on_tick()
        response.success = True
        response.message = 'line released - running until the box has passed the beam'
        self.get_logger().info(response.message)
        return response

    # ── State machine ───────────────────────────────────────────────────────

    def _on_tick(self) -> None:
        # Without scans the beam cannot see anything: keep the last verdict
        # (a stale "blocked" keeps the line stopped, which is the safe side).
        blocked = self._beam_blocked
        M = InfeedLineState
        if not self._enabled:
            self._mode = M.MODE_DISABLED
        elif self._mode == M.MODE_RELEASED:
            if not blocked:
                self._mode = M.MODE_RUNNING
        elif blocked:
            if self._mode != M.MODE_BLOCKED:
                self._boxes_stopped += 1
                self.get_logger().info(
                    f'beam crossed (range {self._beam_range:.2f} m) - stopping the line')
            self._mode = M.MODE_BLOCKED
        elif self._mode == M.MODE_BLOCKED:
            self.get_logger().info('beam clear - restarting the line')
            self._mode = M.MODE_RUNNING
        else:
            self._mode = M.MODE_RUNNING

        running = self._mode in (M.MODE_RUNNING, M.MODE_RELEASED)
        self._send_command(self._speed_request if running else 0.0)
        self._publish_state()

    def _due(self, last, period: float) -> bool:
        return last is None or (self.get_clock().now() - last).nanoseconds * 1e-9 >= period

    def _send_command(self, speed: float) -> None:
        if speed != self._command or self._due(self._last_command_pub, COMMAND_PERIOD_SEC):
            self._speed_pub.publish(Float64(data=float(speed)))
            self._command = speed
            self._last_command_pub = self.get_clock().now()

    def _publish_state(self) -> None:
        msg = InfeedLineState(
            mode=self._mode, enabled=self._enabled, beam_blocked=self._beam_blocked,
            beam_ok=self._beam_ok(),
            beam_range=float(self._beam_range) if math.isfinite(self._beam_range) else math.inf,
            speed_request=float(self._speed_request), speed_command=float(self._command or 0.0),
            boxes_stopped=self._boxes_stopped)
        key = (msg.mode, msg.enabled, msg.beam_blocked, msg.beam_ok, msg.speed_request,
               msg.speed_command, msg.boxes_stopped)
        if key != self._last_state or self._due(self._last_state_pub, STATE_PERIOD_SEC):
            self._state_pub.publish(msg)
            self._last_state = key
            self._last_state_pub = self.get_clock().now()


def main():
    rclpy.init()
    node = InfeedLineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
