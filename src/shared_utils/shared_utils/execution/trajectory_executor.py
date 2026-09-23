"""Send JointTrajectories straight to a joint_trajectory_controller's
FollowJointTrajectory action (no MoveIt in the loop).

Execution time is judged on the CONTROLLER's clock: the feedback header
stamps, which are sim time under Gazebo. A wall-clock deadline breaks as soon
as the sim runs slower than real time (at a real-time factor of 0.14 a 3 s
trajectory takes ~21 s of wall time) - the goal would be cancelled mid-motion,
leaving the arm between waypoints.
"""
import threading
import time
from typing import Optional

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from ..joint_state import JointStateCache
from ..ros_helpers import wait_for_future
from ..trajectory import drop_duplicate_timestamps, max_start_deviation, trajectory_duration


class TrajectoryExecutor:
    """Executes on e.g. /robot_1/gp70l_joint_trajectory_controller/follow_joint_trajectory.

    If a JointStateCache is given, execute() refuses a trajectory whose first
    point is more than max_start_deviation rad away from the current state on
    any joint - a stale plan would otherwise make the arm jump."""

    def __init__(self, node: Node, action_name: str,
                 joint_states: Optional[JointStateCache] = None,
                 max_start_deviation: float = 0.05, stall_timeout_sec: float = 10.0):
        """stall_timeout_sec: wall seconds without any controller feedback
        before giving up (the controller or the sim stopped)."""
        self._node = node
        self.stall_timeout_sec = stall_timeout_sec
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._joint_states = joint_states
        self.max_start_deviation = max_start_deviation
        self._goal_handle = None

    def is_ready(self, timeout_sec: float = 0.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def execute(self, trajectory: JointTrajectory | RobotTrajectory,
                timeout_margin_sec: float = 5.0) -> bool:
        """Blocking. True once the controller reports SUCCESSFUL. Fails if the
        controller's own elapsed time passes the trajectory duration +
        timeout_margin_sec, or if its feedback stalls."""
        if isinstance(trajectory, RobotTrajectory):
            trajectory = trajectory.joint_trajectory
        if not trajectory.points:
            self._node.get_logger().warn('empty trajectory, nothing to execute')
            return False
        traj = drop_duplicate_timestamps(trajectory)
        traj.header.stamp.sec = traj.header.stamp.nanosec = 0  # start immediately

        if self._joint_states is not None:
            msg = self._joint_states.latest
            current = dict(zip(msg.name, msg.position)) if msg is not None else {}
            deviation = max_start_deviation(traj, current)
            if deviation > self.max_start_deviation:
                self._node.get_logger().error(
                    f'trajectory starts {deviation:.3f} rad from the current state '
                    f'(limit {self.max_start_deviation:.3f}) - not executing')
                return False

        goal = FollowJointTrajectory.Goal(trajectory=traj)
        try:
            return self._run(goal, trajectory_duration(traj) + timeout_margin_sec)
        finally:
            self._goal_handle = None

    def _run(self, goal, allowed_sec: float) -> bool:
        log = self._node.get_logger()
        name = self._client._action_name
        if not self._client.wait_for_server(timeout_sec=2.0):
            log.error(f'action server {name} not available')
            return False

        lock = threading.Lock()
        progress = {'first': None, 'elapsed': 0.0, 'last_wall': time.monotonic()}

        def on_feedback(msg):
            stamp = msg.feedback.header.stamp
            t = stamp.sec + stamp.nanosec * 1e-9
            with lock:
                progress['last_wall'] = time.monotonic()
                if t > 0.0:
                    if progress['first'] is None:
                        progress['first'] = t
                    progress['elapsed'] = t - progress['first']

        handle = wait_for_future(self._client.send_goal_async(goal, feedback_callback=on_feedback), 5.0)
        if handle is None or not handle.accepted:
            log.error(f'goal to {name} was rejected')
            return False
        self._goal_handle = handle
        result_future = handle.get_result_async()

        while not result_future.done():
            time.sleep(0.05)
            with lock:
                elapsed, stalled = progress['elapsed'], time.monotonic() - progress['last_wall']
            if elapsed > allowed_sec:
                log.error(f'trajectory not finished after {elapsed:.1f}s of controller time '
                          f'(allowed {allowed_sec:.1f}s) - cancelling')
                handle.cancel_goal_async()
                return False
            if stalled > self.stall_timeout_sec:
                log.error(f'no feedback from {name} for {stalled:.0f}s - cancelling')
                handle.cancel_goal_async()
                return False

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            log.error(f'trajectory execution failed: error_code={result.error_code} {result.error_string}')
            return False
        return True

    def cancel(self) -> bool:
        """Cancel the goal currently executing (from another thread)."""
        handle = self._goal_handle
        if handle is None:
            return False
        handle.cancel_goal_async()
        return True
