"""Small rclpy helpers shared by the clients in this package.

Every blocking call in shared_utils waits on a future with wait_for_future(),
which does NOT spin the node itself. The node must already be spun by an
executor on another thread - spin_in_background() is the one-liner for
scripts, a MultiThreadedExecutor does the same inside a larger node. Blocking
from inside one of the node's own callbacks only works if that callback's
group lets the future's callbacks run concurrently (ReentrantCallbackGroup, or
a different group on a MultiThreadedExecutor).
"""
import threading
from typing import Any, Optional

from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future


def spin_in_background(node: Node, num_threads: int = 4) -> MultiThreadedExecutor:
    """Spin node on a daemon thread and return its executor (call
    executor.shutdown() to stop it)."""
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=_spin, daemon=True).start()
    return executor


def wait_for_future(future: Future, timeout_sec: Optional[float] = None) -> Optional[Any]:
    """Block until future is done and return its result, or None on timeout.
    Exceptions raised by the future propagate."""
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout=timeout_sec):
        return None
    return future.result()


def call_service(node: Node, client, request, timeout_sec: float = 5.0,
                 wait_for_service_sec: float = 2.0) -> Optional[Any]:
    """Blocking service call. Returns the response, or None if the service is
    unavailable or the call timed out (logged)."""
    if not client.wait_for_service(timeout_sec=wait_for_service_sec):
        node.get_logger().warn(f'service {client.srv_name} not available')
        return None
    response = wait_for_future(client.call_async(request), timeout_sec)
    if response is None:
        node.get_logger().warn(f'service {client.srv_name} timed out after {timeout_sec:.1f}s')
    return response


def send_action_goal(node: Node, client: ActionClient, goal, timeout_sec: Optional[float] = None,
                     wait_for_server_sec: float = 2.0, feedback_callback=None,
                     on_accepted=None):
    """Blocking action call. Returns (result_response, goal_handle); result_response
    is None if the server is unavailable, the goal was rejected or the timeout
    passed (the goal is cancelled in that case). on_accepted(goal_handle) runs
    as soon as the goal is accepted, so another thread can cancel it."""
    if not client.wait_for_server(timeout_sec=wait_for_server_sec):
        node.get_logger().warn(f'action server {client._action_name} not available')
        return None, None
    goal_handle = wait_for_future(
        client.send_goal_async(goal, feedback_callback=feedback_callback), wait_for_server_sec)
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().warn(f'goal to {client._action_name} was rejected')
        return None, goal_handle
    if on_accepted is not None:
        on_accepted(goal_handle)
    response = wait_for_future(goal_handle.get_result_async(), timeout_sec)
    if response is None:
        node.get_logger().warn(f'goal to {client._action_name} timed out, cancelling')
        goal_handle.cancel_goal_async()
    return response, goal_handle
