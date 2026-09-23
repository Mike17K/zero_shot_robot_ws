# Status of box_factory_bringup's factory node, published (latched) on
# /<factory namespace>/status.

bool enabled                  # operator switch (parameter `enabled`)
bool paused_by_line           # feeding line stopped / off (follow_line)
bool spawn_area_blocked       # a box is still in the spawn area
bool gazebo_ok                # the world's create service answered last time
uint32 boxes_spawned
string last_box               # name of the last spawned box
float64 next_spawn_in         # seconds until the next box (wall time, <0: waiting)
