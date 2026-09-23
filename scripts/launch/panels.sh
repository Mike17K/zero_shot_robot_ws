#!/bin/bash
# Shared panel plan for launch_from_host_ws.sh / launch_from_container_ws.sh.
# Sourced after utils.sh; the caller sets ENTER_CONTAINER=true when every
# pane first has to `bash scripts/shell.sh` into the container.
#
# Commands are pasted but NOT run - press Enter in each pane, top to bottom
# in this order: Workcell, cuMotion, RViz, Teleop, then Navigator once the
# robot is up (it needs /robot_1/joint_states, TF and cumotion/motion_plan).

ROS_DOMAIN_ID=40  # matches the container root .bashrc (setup_workspace.sh)
ROS_DISTRO="jazzy"
CONTAINER_WS="/workspaces/isaac_ros-dev"

# Prepares every pane INSIDE the container. The venv is optional (it does
# not exist in a fresh container) - an unconditional `source` of it used to
# break this && chain, so ROS_DOMAIN_ID/RMW were never exported.
GLOBAL_CMD="cd $CONTAINER_WS && source /opt/ros/$ROS_DISTRO/setup.bash && source $CONTAINER_WS/install/setup.bash \
&& { [ ! -f $CONTAINER_WS/.venv/bin/activate ] || source $CONTAINER_WS/.venv/bin/activate; } \
&& export ISAAC_ROS_WS=$CONTAINER_WS && export PYTHONPATH=\$PYTHONPATH:$CONTAINER_WS/external \
&& export ROS_DOMAIN_ID=$ROS_DOMAIN_ID && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
&& export CYCLONEDDS_URI=file://$CONTAINER_WS/cyclonedds.xml"

# ── Panel layout (scripts/config/terminator_config's GazeboLayout) ─────────
#   left column              | right column
#   terminal0  Workcell      | terminal4  cuMotion planner
#   terminal1  nvblox        | terminal5  RViz
#   terminal2  Navigator CLI | terminal6  Teleop UI
#   terminal3  cuMotion test | terminal7  Scratch
# Terminator's Alt+arrow jumps to the geometrically nearest pane, which is
# ambiguous across the unevenly split columns - so every step first
# saturates to the top-left corner (extra moves past an edge are no-ops),
# then goes right at most once and down n times within one column.
reset_to_top_left() {
    move_up
    move_up
    move_up
    move_up
    move_left
}

goto_pane() {  # goto_pane <column 0|1> <row 0-3>
    reset_to_top_left
    [ "$1" = "1" ] && move_right
    for ((i = 0; i < $2; i++)); do move_down; done
}

launch_panels() {
    reset_to_top_left

    broadcast_on
    if [ "$ENTER_CONTAINER" = "true" ]; then
        paste_cmd "bash scripts/shell.sh" && enter
        sleep 5
    fi
    paste_cmd "$GLOBAL_CMD && clear"
    enter
    broadcast_off

    goto_pane 0 0
    paste_cmd 'ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false'

    goto_pane 0 1
    paste_cmd 'ros2 launch vision nvblox.launch.py'

    # Interactive pose-graph navigator (src/planning/navigator_cli). Graphs
    # are saved to the source config/ folder (ISAAC_ROS_WS is exported above).
    goto_pane 0 2
    paste_cmd 'ros2 run navigator_cli navigator_cli --ros-args -p namespace:=robot_1'

    # One-shot planner check without moving the arm (add --execute to move).
    goto_pane 0 3
    paste_cmd 'ros2 run shared_utils cumotion_cli joints 0 -0.35 0.35 0 -0.52 0'

    # sim_gazebo:=true -> use_sim_time, matching the Gazebo-driven robot.
    goto_pane 1 0
    paste_cmd 'ros2 launch planning_bringup cumotion.launch.py namespace:=robot_1 sim_gazebo:=true'

    goto_pane 1 1
    paste_cmd 'ros2 launch workcell_bringup rviz.launch.py rviz_namespace:=robot_1'

    # Joint sliders, gripper, conveyor speeds, Spawn Box and Box Factory -
    # see src/workcell/workcell_teleop.
    goto_pane 1 2
    paste_cmd 'ros2 launch workcell_teleop teleop.launch.py'
}
