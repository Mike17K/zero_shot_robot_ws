#!/usr/bin/env python3
"""ROS 2 action interface to the navigator: the same pose graph and robot
manager as the interactive CLI, served as actions so the demo (and any other
script, via shared_utils.navigation.NavigatorClient) can drive the robot.

  ros2 run navigator_cli navigator_server --ros-args -p namespace:=robot_1

Actions (under /<namespace>/navigator/):
  navigate_to_node  custom_msgs/action/NavigateToNode  graph path (or direct joint
                                                      move) to a node id / label
  cartesian_move    custom_msgs/action/CartesianMove   straight-line tool offset

One motion at a time: a goal arriving while another runs is rejected.
Cancelling stops the running trajectory. The graph file is reloaded for every
navigate goal, so nodes taught in the CLI are picked up without a restart.
"""
import threading
import traceback

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from custom_msgs.action import CartesianMove, NavigateToNode

from .robots.interface import RobotPose
from .robots.ROS2.setup import setupNode
from .robots.utils import find_and_move_to_node_logic
from .robots.wrapper import RobotInterface
from .shared.constants import AVAILABLE_GROUPS, CANONICAL_EE_GROUP
from .shared.graph import Graph
from .shared.logger import Logger
from .shared.types import TrajectoryMovementOptions

DEFAULT_CARTESIAN_SPEED = 0.05  # m/s


class NavigatorServer:
    def __init__(self, node):
        self._node = node
        node.declare_parameter('cartesian_speed', DEFAULT_CARTESIAN_SPEED)
        self.robot = RobotInterface()
        self.robot.SetMode(node, isReal=True)
        self.graph: Graph[RobotPose] = Graph[RobotPose](node_cls=RobotPose)
        self._busy = threading.Lock()
        prefix = f"/{node.get_parameter('namespace').value.strip('/')}/navigator"
        group = ReentrantCallbackGroup()
        common = dict(goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                      callback_group=group)
        self._nav = ActionServer(node, NavigateToNode, f'{prefix}/navigate_to_node',
                                 execute_callback=self._navigate, **common)
        self._cart = ActionServer(node, CartesianMove, f'{prefix}/cartesian_move',
                                  execute_callback=self._cartesian, **common)
        Logger.INFO(f"navigator_server ready on {prefix}/{{navigate_to_node, cartesian_move}}")

    # ── Goal handling ───────────────────────────────────────────────────────

    def _on_goal(self, goal) -> GoalResponse:
        if self._busy.locked():
            Logger.WARN('navigator busy - rejecting goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        Logger.WARN('cancel requested - stopping the current motion')
        self.robot.manager.cancel_motion()
        return CancelResponse.ACCEPT

    def _resolve(self, target: str):
        try:
            return self.graph._id_to_pose.get(int(target))
        except ValueError:
            matches = [n for n in self.graph._id_to_pose.values()
                       if n.label and n.label.lower() == target.strip().lower()]
            return matches[0] if matches else None

    @staticmethod
    def _options(speed: float):
        return TrajectoryMovementOptions(speed=speed) if speed > 0.0 else None

    def _finish(self, goal_handle, result):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── navigate_to_node ────────────────────────────────────────────────────

    def _navigate(self, goal_handle):
        goal = goal_handle.request
        result = NavigateToNode.Result(success=False, reached_node=-1)
        with self._busy:
            try:
                self.graph.load_from_file()
                node = self._resolve(goal.target)
                if node is None:
                    result.message = (f"no node '{goal.target}' - known: " + ', '.join(
                        f"{i}{' ' + repr(n.label) if n.label else ''}"
                        for i, n in sorted(self.graph._id_to_pose.items())))
                    return self._finish(goal_handle, result)
                node_id = self.graph._get_or_assign_id(node)
                options = self._options(goal.speed)

                def feedback(segment, segments, next_node):
                    goal_handle.publish_feedback(NavigateToNode.Feedback(
                        segment=segment, segments=segments, next_node=next_node,
                        status=f'segment {segment}/{segments} -> node {next_node}'))

                if goal.direct:
                    feedback(1, 1, node_id)
                    ok = self.robot.execute_joint_goal(CANONICAL_EE_GROUP, node.canonical.joints, options)
                else:
                    ok = find_and_move_to_node_logic(
                        self.robot, node_id, self.graph, list(AVAILABLE_GROUPS), interactive=False,
                        on_segment=feedback, should_stop=lambda: goal_handle.is_cancel_requested,
                        options=options)
                result.success = bool(ok) and not goal_handle.is_cancel_requested
                result.reached_node = node_id if result.success else -1
                result.message = (f'reached node {node_id}' if result.success else
                                  'cancelled' if goal_handle.is_cancel_requested else
                                  f'failed on the way to node {node_id} (see navigator_server log)')
            except Exception as exc:  # never leave the goal hanging
                result.message = f'error: {exc}'
                Logger.ERROR(f'navigate_to_node: {exc}\n{traceback.format_exc()}')
        Logger.INFO(f"navigate_to_node '{goal.target}': {result.message}")
        return self._finish(goal_handle, result)

    # ── cartesian_move ──────────────────────────────────────────────────────

    def _cartesian(self, goal_handle):
        goal = goal_handle.request
        result = CartesianMove.Result(success=False, fraction=0.0)
        offset = (goal.offset.x, goal.offset.y, goal.offset.z)
        with self._busy:
            try:
                speed = goal.speed if goal.speed > 0.0 else self._node.get_parameter('cartesian_speed').value
                goal_handle.publish_feedback(CartesianMove.Feedback(
                    status=f'moving {offset} m in {goal.frame or "world"} frame'))
                r = self.robot.manager.execute_cartesian(offset, frame=goal.frame or 'world',
                                                         max_speed=speed)
                result.success, result.message, result.fraction = r.success, r.message, r.fraction
            except Exception as exc:
                result.message = f'error: {exc}'
                Logger.ERROR(f'cartesian_move: {exc}\n{traceback.format_exc()}')
        Logger.INFO(f'cartesian_move {offset}: {result.message}')
        return self._finish(goal_handle, result)


def main(args=None):
    node = setupNode(args, name='navigator_server')
    NavigatorServer(node)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
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
