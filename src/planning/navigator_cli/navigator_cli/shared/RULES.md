# shared/ RULES

- `shared/` holds framework-agnostic domain primitives, configuration, logging and graph persistence. No ROS imports.
- `constants.py`: `CANONICAL_EE_GROUP`, `AVAILABLE_GROUPS`, `FLOAT_TOLERANCE`, `CONFIG_FOLDER`.
  - `CONFIG_FOLDER` is `$NAVIGATOR_CLI_CONFIG_DIR`, else `$ISAAC_ROS_WS/src/planning/navigator_cli/config` (taught graphs stay in git), else the installed share copy.
- `types.py`: `Point`, `Quaternion` (tolerance-aware equality and hashing) and `TrajectoryMovementOptions`.
- `logger.py`: `Logger.INFO()`, `Logger.WARN()`, `Logger.ERROR()`.
- `graph.py`: generic `Graph[T]` plus `Edge`.
  - Node types satisfy `NodeProtocol` (`id`, `label`, `model_dump`, `model_validate_json`, `get_similarity`, `copy_with`).
  - `add_node()` stores the node with its assigned id; ids >= 1000 are reserved for temporary nodes.
  - Persistence: `save_to_file()` / `load_from_file()` (JSON: `nodes`, `edges`).
  - `shortest_path()` returns `(pose, move_group, options)` tuples; the first entry is the start node.
  - `find_nearby_nodes()` ranks connected nodes by RMS joint difference of the canonical group.
