# navigator_cli/ RULES

- The package is an interactive pose-graph navigator for group_a (Yaskawa GP70L + suction gripper).
- `main.py` is the CLI: it builds the robot interface and the graph, registers the commands and dispatches input lines.
- `cli_functions/` holds one class per command.
- `robots/` holds the robot client API (`interface.py`), the thin facade (`wrapper.py`), the path-execution logic (`utils.py`) and the ROS 2 implementation (`ROS2/gp70l.py`).
- `shared/` holds ROS-free building blocks: types, the generic graph, logging and constants.
- `utils/` holds higher-level motion helpers built on `robots/`.
- Planning, execution, TF and joint states come from the `shared_utils` package - do not add new ROS clients here if `shared_utils` already has one.
- Keep package-level code minimal: prefer shared domain types and helper modules over ad-hoc script logic.
