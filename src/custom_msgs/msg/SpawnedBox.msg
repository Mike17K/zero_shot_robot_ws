# One box spawned into Gazebo, published by box_factory_bringup's factory node
# on /<factory namespace>/spawned - e.g. so gripper_manager knows its size.

string name                   # Gazebo model name (graspable prefix "box")
geometry_msgs/Vector3 size    # width (x), depth (y), height (z), meters
geometry_msgs/Pose pose       # spawn pose, world frame
float64[] rgb                 # base color (empty: textured)
string albedo_map             # texture file, absolute path (empty: none)
