# robots/ RULES

- `interface.py` defines the robot client API and the pose data models.
  - `MoveGroupState` is the atomic state of a group: tool `pos` / `rot` in the world frame and `joints`.
  - `RobotPose` is a graph node: `all_ee_poses` maps group name -> `MoveGroupState`; `canonical` returns the `CANONICAL_EE_GROUP` entry. It is a joint goal (`is_joint_goal=True`, the default for taught nodes) or a pose goal.
  - `BaseRobotManager` implementations must provide `get_ee_pose_for_group`, `execute_joint_goal`, `execute_pose_goal`, `get_pose_from_fk` and `get_pose_from_ik`.
- `ROS2/gp70l.py` (`Gp70lRobotManager`) is the implementation for this workspace, all under the robot namespace (default `robot_1`):
  - planning: cuMotion `cumotion/motion_plan` and `cumotion/ik` via `shared_utils.planning.CumotionClient`
  - execution: `gp70l_joint_trajectory_controller/follow_joint_trajectory` via `shared_utils.execution.TrajectoryExecutor`
  - state: `joint_states` + `tf` (tool `gripper_tcp` in `world`), FK: MoveIt `compute_fk`
  - gripper: `gripper/set_suction`
- `wrapper.py` exposes `RobotInterface` with static methods over one manager instance. `RobotInterface.SetMode(node)` creates it from the node's parameters (`namespace`, `world_frame`, `tool_frame`, `controller`).
- `utils.py` combines graph search and execution: `determine_start_node`, `find_and_move_to_node_logic`, `find_and_move_to_target_logic`.
- Move groups must be listed in `shared/constants.py` `AVAILABLE_GROUPS`. For the GP70L that is only `manipulator` (6 joints: `gp_joint_1` ... `gp_joint_6`).
- `TrajectoryMovementOptions.speed` is the cuMotion time dilation factor in (0, 1]; `max_attempts` is the number of planning attempts.
- Keep the wrapper thin (asserts, logging, forwarding); the manager does the actual planner/service/robot calls.
