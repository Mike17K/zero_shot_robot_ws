# State of the infeed feeding line, published (latched) by workcell_bringup's
# infeed_line_controller on /<infeed namespace>/line/state.

uint8 MODE_DISABLED=0   # operator switched the line off, belt stopped
uint8 MODE_RUNNING=1    # belt runs at speed_request, beam watched
uint8 MODE_BLOCKED=2    # a box crossed the beam: belt stopped until the box is picked up
uint8 MODE_RELEASED=3   # operator released the blocked box: belt runs until the beam clears

uint8 mode
bool enabled            # operator on/off (set via line/set_enabled)
bool beam_blocked       # laser beam currently interrupted
bool beam_ok            # scans are arriving (false: sensor missing / stale)
float32 beam_range      # m, distance the beam travels before hitting something
float64 speed_request   # rad/s the belt runs at when the line is running
float64 speed_command   # rad/s currently sent to the belt
uint32 boxes_stopped    # how many times the beam has stopped the line
