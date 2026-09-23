# utils/ RULES

- `utils/` contains reusable helpers that combine robot state and graph logic.
- `go_to_node.py`:
  - `go_to_pose(robot, graph, target_joints, direct=False, options=None)` - optionally snap onto the closest graph node first, then move to the joints.
  - `go_to_closest_pose(robot, graph, options=None)` - move to the node a path would start from.
- Keep movement decision logic here and planner details in `robots/`.
- Use `Logger` for status messages; keep ROS node startup out of `utils/`.
