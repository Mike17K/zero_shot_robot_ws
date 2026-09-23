"""Plan (and optionally execute) one cuMotion motion from the command line.

  ros2 run shared_utils cumotion_cli joints 0 -0.35 0.35 0 -0.52 0 --execute
  ros2 run shared_utils cumotion_cli pose 1.2 0.0 1.0 3.1416 0 0 --frame world
  ros2 run shared_utils cumotion_cli ik 1.2 0.0 1.0 3.1416 0 0 --frame world

Pose arguments are x y z roll pitch yaw (meters, radians) of the tool frame
(gripper_tcp).
"""
import argparse
import sys

import rclpy
from rclpy.node import Node

from shared_utils.execution import TrajectoryExecutor
from shared_utils.geometry import TfHelper, make_pose_stamped
from shared_utils.joint_state import JointStateCache
from shared_utils.planning import CumotionClient
from shared_utils.ros_helpers import spin_in_background
from shared_utils.trajectory import trajectory_duration


def _parse(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('joints', 'pose', 'ik'))
    parser.add_argument('values', type=float, nargs=6)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--frame', default='gp_base_link', help='frame of a pose goal')
    parser.add_argument('--time-dilation', type=float, default=0.5)
    parser.add_argument('--execute', action='store_true', help='run the planned trajectory')
    parser.add_argument('--controller', default='gp70l_joint_trajectory_controller')
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse(rclpy.utilities.remove_ros_args(sys.argv)[1:] if argv is None else argv)
    rclpy.init()
    node = Node('cumotion_cli')
    ns = args.namespace.strip('/')
    tf = TfHelper(node, f'/{ns}/tf')
    joint_states = JointStateCache(node, f'/{ns}/joint_states')
    planner = CumotionClient(node, namespace=ns, tf=tf, time_dilation_factor=args.time_dilation)
    executor = spin_in_background(node)
    log = node.get_logger()
    code = 1
    try:
        if not planner.is_ready(timeout_sec=10.0):
            log.error(f'/{ns}/cumotion/motion_plan not available - start it with: '
                      f'ros2 launch planning_bringup cumotion.launch.py namespace:={ns} sim_gazebo:=true')
            return code
        if args.mode == 'joints':
            result = planner.plan_to_joints(args.values)
        else:
            x, y, z, roll, pitch, yaw = args.values
            goal = make_pose_stamped(args.frame, (x, y, z), (roll, pitch, yaw))
            if args.mode == 'ik':
                solutions = planner.solve_ik(goal, num_solutions=3)
                for s in solutions:
                    log.info('IK: ' + ', '.join(f'{v:+.3f}' for v in s))
                return 0 if solutions else code
            result = planner.plan_to_pose(goal)

        if not result.success:
            log.error(f'planning failed: {result.error_name} {result.message}')
            return code
        traj = result.trajectory
        log.info(f'planned {len(traj.points)} points, {trajectory_duration(traj):.2f}s '
                 f'in {result.planning_time:.3f}s')
        if args.execute:
            joint_states.wait(timeout_sec=5.0)
            runner = TrajectoryExecutor(
                node, f'/{ns}/{args.controller}/follow_joint_trajectory', joint_states)
            if not runner.execute(traj):
                return code
            log.info('executed')
        code = 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
