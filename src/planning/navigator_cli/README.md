# navigator_cli

Interactive pose-graph navigator for **group_a** (Yaskawa GP70L + suction gripper). You jog the arm to useful poses, store them as graph nodes, connect them with edges, and then send the arm along the shortest path between nodes. Every edge is planned by cuMotion and executed on the arm's `joint_trajectory_controller`.

## Run

Start the workcell and the cuMotion planner first, each in its own terminal inside the container:

```bash
ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false
ros2 launch planning_bringup cumotion.launch.py namespace:=robot_1 sim_gazebo:=true
ros2 launch workcell_teleop teleop.launch.py   # teleop UI: jog the arm to poses you want to teach, gripper, conveyors, box factory
```

Wait for the planner to log `MotionPlan action server initialized.`, then check it with a plan-only request (add `--execute` to move the arm):

```bash
ros2 run shared_utils cumotion_cli joints 0 -0.35 0.35 0 -0.52 0
```

Then start the navigator:

```bash
ros2 run navigator_cli navigator_cli
# another robot / frames:
ros2 run navigator_cli navigator_cli --ros-args -p namespace:=robot_1 -p tool_frame:=gripper_tcp
```

| Parameter    | Default                              | Meaning                              |
| ------------ | ------------------------------------ | ------------------------------------ |
| `namespace`  | `robot_1`                            | Robot namespace (all topics/actions) |
| `world_frame`| `world`                              | Frame node poses are stored in       |
| `tool_frame` | `gripper_tcp`                        | Tool frame (the XRDF tool frame)     |
| `controller` | `gp70l_joint_trajectory_controller`  | FollowJointTrajectory controller     |

It uses, all under `/<namespace>/`:

- `cumotion/motion_plan` and `cumotion/ik` (`planning_bringup/launch/cumotion.launch.py`)
- `<controller>/follow_joint_trajectory`, `joint_states`, `tf`
- `compute_fk` (MoveIt `move_group`)
- `gripper/set_suction` (`group_a_bringup`'s `gripper_manager`)

## Commands

| Command | What it does |
| ------- | ------------ |
| `graph <sub>` | Edit the graph: `add`, `set-parent`, `reload`, `delete-edge`, `delete-node`, `add-edge`, `list`, `save [file]`, `current` |
| `a` `p` `r` `d` `x` `j` `l` `save` | Shortcuts for the `graph` subcommands above |
| `e <id\|label>` | Move along the shortest path to a node, by ID or label (asks y/N first). Invalid input, or plain `e`, prints the usage and every node with its label, position, edge count and where the robot is |
| `e <id\|label> -d` | Move straight to the node's joints, ignoring the graph |
| `e -p` | Show which node the robot is at |
| `e -t` / `t` | Move the tool to a pose you type in (direct plan, else via the closest node) |
| `s <id>` | Show the path to node `<id>` without moving |
| `ta s <key>` / `ta <key>` / `ta i` / `ta l` / `ta clear` | Save, move to, type in, list or clear named joint positions |
| `gripper on\|off`, `g on\|off`, `suction on\|off`, `on`, `off` | Suction gripper |
| `cart <dx> <dy> <dz> [-tool] [-speed=0.05]` | Straight-line tool move (MoveIt `compute_cartesian_path`), world axes or the tool's own with `-tool` (asks y/N first) |
| `home` | Move to the XRDF default joint position |
| `help`, `exit` | |

Typical teaching loop: jog the arm with the teleop UI sliders (`ros2 launch workcell_teleop teleop.launch.py`), `a` to add the pose (it asks for a label, whether paths may start from it, and which node to connect it to), repeat, then `e <id>` to replay.

## Action server

`navigator_server` offers the same navigation over ROS 2, for the demo and any other script:

```bash
ros2 launch navigator_cli navigator_server.launch.py namespace:=robot_1
```

| Interface | Type | What |
| --------- | ---- | ---- |
| `/<ns>/navigator/navigate_to_node` | `custom_msgs/action/NavigateToNode` | graph path (or `direct` joint move) to a node id or label, feedback per segment |
| `/<ns>/navigator/cartesian_move` | `custom_msgs/action/CartesianMove` | straight-line tool offset, `world` or `tool` axes |

One motion at a time (a second goal is rejected); cancel stops the running trajectory; the graph file is reloaded for every goal. From Python, use `shared_utils.navigation.NavigatorClient` (`navigate()`, `cartesian()`, `gripper()`, `holding()`).

## Data

Files live in `config/`: `pose_graph.json` (the graph) and `saved_positions.txt` (`ta` positions). Inside the workspace container (`ISAAC_ROS_WS` set) the CLI reads and writes the **source** `config/` folder, so taught graphs can be committed. Set `NAVIGATOR_CLI_CONFIG_DIR` to use another folder.

`ros2 run navigator_cli visualize_pose_graph [file]` plots a graph in 3D.

## Layout

See the `RULES.md` files in each folder: `cli_functions/` (one class per command), `robots/` (robot API + `ROS2/gp70l.py`), `shared/` (types, graph, logger, constants), `utils/` (motion helpers).
