#!/bin/bash

ROS_DOMAIN_ID=40
ROS_DISTRO="jazzy"
# Host-side workspace path (this script itself runs on the host, launching
# terminator - see open_terminator below) vs. the container-side path each
# panel actually lands in after "bash scripts/shell.sh". They are NOT the
# same path: Dockerfile.cumotion_ws sets WORKDIR /workspaces/isaac_ros-dev,
# so that's where the workspace lives *inside* the container regardless of
# what host directory this script was launched from. GLOBAL_CMD is pasted
# into panels that are already inside the container by the time it runs, so
# it must use CONTAINER_WS, not WS - using WS there was the bug (it `cd`ed
# to a host path that doesn't exist in the container, so every subsequent
# `source install/setup.bash` in this script silently ran from the wrong -
# or a nonexistent - directory).
WS=${PWD}
CONTAINER_WS="/workspaces/isaac_ros-dev"
DEFAULT_DELAY=0.05
DEFAULT_LONG_DELAY=0.2
# Η εντολή που προετοιμάζει κάθε νέο terminal panel (runs INSIDE the container)
GLOBAL_CMD="cd $CONTAINER_WS && source /opt/ros/$ROS_DISTRO/setup.bash && source $CONTAINER_WS/install/setup.bash && source $CONTAINER_WS/.venv/bin/activate && export PYTHONPATH=\$PYTHONPATH:$CONTAINER_WS/external && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
LAYOUT_NAME="GazeboLayout"
TERMINATOR_CONFIG="$WS/scripts/config/terminator_config"

source $WS/scripts/utils.sh

open_terminator


# ── Panel layout (scripts/config/terminator_config's GazeboLayout) ─────────
#   terminal0 (top-left)     | terminal2 (top-right)
#   terminal1 (bottom-left)  | terminal3 (middle-right)
#                            | terminal4 (bottom-right) - Teleop UI
# Right column is 3 panes deep, left column only 2 - navigation below always
# resets to the top-left corner first (move_up twice, move_left once) rather
# than chaining relative moves from wherever the previous step left off:
# terminator's Alt+Left/Right/Up/Down jumps to the geometrically nearest
# pane in that direction, which gets ambiguous once the grid is asymmetric
# like this. Repeating a direction past the edge is a no-op, so the extra
# move_up is harmless when it's not needed (e.g. already in the left
# column) - this is the same "saturate to the corner" trick the very first
# move_up/move_left pair already relied on, just extended for the deeper
# right column.
reset_to_top_left() {
    move_up
    move_up
    move_left
}

reset_to_top_left

# paste_cmd "bash scripts/shell.sh" && enter
# sleep 5

broadcast_on
paste_cmd "bash scripts/shell.sh" && enter
sleep 2
paste_cmd "$GLOBAL_CMD && clear"
enter
broadcast_off

# configuration broadcasting
paste_cmd 'source install/setup.bash && ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false'

reset_to_top_left
move_right
paste_cmd 'source install/setup.bash && ros2 launch planning_bringup cumotion.launch.py'

reset_to_top_left
move_right
move_down
paste_cmd 'source install/setup.bash && ros2 launch workcell_bringup rviz.launch.py rviz_namespace:=robot_1'

# Teleop UI: joint-position sliders for the robot(s) and speed sliders for
# the conveyor fleet - see src/workcell/workcell_teleop. Defaults in
# workcell_teleop/launch/teleop.launch.py mirror workcell_bringup/launch/
# workcell.launch.py's own robots_config/conveyors_config namespace lists -
# no extra args needed unless that layout has since changed.
reset_to_top_left
move_right
move_down
move_down
paste_cmd 'source install/setup.bash && ros2 launch workcell_teleop teleop.launch.py'

reset_to_top_left
move_down
paste_cmd 'source install/setup.bash && ros2 launch vision nvblox.launch.py'
