#!/bin/bash
# Opens the Terminator GazeboLayout and pastes the workcell / cuMotion / RViz /
# nvblox / teleop / navigator commands into its panes - run from the
# workspace root INSIDE the container (terminator runs there): panes are already in it.
# Panel plan and pane order: scripts/launch/panels.sh.

WS=${PWD}
DEFAULT_DELAY=0.05
DEFAULT_LONG_DELAY=0.2
LAYOUT_NAME="GazeboLayout"
TERMINATOR_CONFIG="$WS/scripts/config/terminator_config"
ENTER_CONTAINER=false

source $WS/scripts/utils.sh
source $WS/scripts/launch/panels.sh

open_terminator
launch_panels
