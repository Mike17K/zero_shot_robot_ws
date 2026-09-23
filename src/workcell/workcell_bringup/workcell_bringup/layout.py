"""Workcell layout shared by the launch files (workcell.launch.py,
infeed_line.launch.py): robots and conveyor belts, all in the world frame.
Values are strings because they are passed straight through as launch
arguments."""
import math

# 7. Ρομπότ στην Κυψέλη Εργασίας
# Single arm (Yaskawa GP70L), mounted on top of the robot_pedestal static
# prop (see workcell_description/worlds/workcell_world.sdf and models/
# robot_pedestal) - z=0.4 matches the pedestal's own height (shorter than
# the old UR10-era pedestal: GP70L's own base+joint_1 already adds 0.54m
# of height before the arm starts moving - see group_a_description/urdf/
# group_a_macro.xacro). This workcell is now a specific zero-shot
# pick/place cell (arm + conveyor fleet below), not a generic multi-arm
# layout - add more entries here (each with its own pedestal + conveyor
# cell) to scale to a real fleet.
ROBOTS = [
    {"name": "robot_1", "xyz": "0.0 0.0 0.4", "rpy": "0.0 0.0 0.0"},
]


# 9. Conveyor fleet around the pedestal (see conveyor_description/
# conveyor_bringup - each is its own namespaced instance, same
# description/bringup-pair pattern as group_a and the sibling
# internal_delivery_system_ws AGV fleet).
#
# Layout (world frame, robot_1 base at the origin on top of the
# pedestal): xyz below is each belt's own footprint CENTER, at floor
# level (see conveyor_description/urdf/conveyor_macro.xacro's origin
# convention - it also only frames the two ends with thin legs, not a
# solid wall the full length, precisely so a crossing belt's own legs
# have a clear gap to land in).
#
# A first pass here had the 3 pallet lanes flush against each other
# (shared y boundaries, zero gap) and the crossing belt's clearance from
# pallet_mid's own leg down to ~0.025m - both individually "correct" on
# paper but together read as one overlapping mess in Gazebo. Real gaps
# everywhere below.
#
# Rescaled by ~1.6x (GP70L's ~2.05m reach vs. the earlier UR10's ~1.3m)
# from the original UR10-era layout when the arm was swapped for a
# Yaskawa GP70L - lane/gap DISTANCES scaled, belt WIDTHS left as-is
# (pallet/package size is independent of which arm is servicing them).
# pallet heights (0.45) also left as-is (floor-clearance choice, not
# reach-related). Still only worked out analytically (see the
# verification script this was computed with), not simulation-verified -
# recheck in Gazebo.
#
# package_infeed is the ONE exception: its xyz/length are pinned by
# direct instruction, NOT part of this rescale - see its own entry below.
#
#   y
#   ^  pallet_left  (long,  1.0 x 5.76, z-top 0.45)   y in [0.8, 1.8]
#   |  pallet_mid   (short, 0.8 x 4.96, z-top 0.45)   y in [-0.4, 0.4],
#   |                 starts at x=0.80 (0.24m clear of the pedestal's
#   |                 x=0.56 edge) - the pedestal blocks the lane near
#   |                 the robot, so this middle lane is shorter and
#   |                 starts further out than the two long side lanes,
#   |                 flush with them at the far end (x=5.76) - "the 2nd
#   |                 is shorter and in the edge of the 2 big ones".
#   |  pallet_right (long,  1.0 x 5.76, z-top 0.45)   y in [-1.8, -0.8]
#   +------------------------------------------------------------> x
#  (pedestal + robot_1 base, centered at the origin)
#
# package_infeed (0.5 x 4.0, z-top 0.4, rotated 90 deg - "length" runs
# along Y) is fixed at xyz=(-0.75, 1.5, 0), NOT auto-rescaled - footprint
# works out to x:[-1.0,-0.5], y:[-0.5,3.5]. It no longer crosses
# pallet_mid at all (pallet_mid's own x-span is [0.8,5.76], entirely
# positive-x; package_infeed sits on the negative-x side of the pedestal
# instead). It DOES have a small bounding-box overlap with the pedestal's
# own corner (pedestal x:[-0.56,0.56] y:[-0.56,0.56]) - roughly a
# 0.06m x 1.06m sliver where package_infeed's near-end leg lands inside
# the pedestal's footprint. Left as-is per instruction; flagging it here
# rather than silently fixing it - check in Gazebo, nudge by hand if it's
# a real collision and not just close.
# side_rail_height: guide-rail height ABOVE the conveying surface (see
# conveyor_description/urdf/conveyor_macro.xacro). Low on the 3 pallet
# belts - a low rail still guides pallets in transit but stays well below
# a pallet's own height, so the robot can still lift one off from above
# without the rail being in the way. Tall on package_infeed - it's
# carrying loose, unpalletized items with nothing else holding them in
# place, so it needs actual containment walls.
# controllers_spawn_delay: staggered 0.5s apart across the 4 belts, and
# offset from robot_1's own fixed 4.0s (see group_a_bringup/launch/
# bringup.launch.py) - all 5 controller_managers used to fire their
# switch_controller call at the exact same instant (every bringup.
# launch.py hardcoded the same 4.0s delay), which was real startup
# contention on a machine without GPU passthrough into the container
# (CPU rendering fallback). See conveyor_bringup/launch/bringup.launch.py
# and group_a_bringup/launch/bringup.launch.py's own comments on this -
# also paired there with a raised --switch-timeout as a second line of
# defense for whatever contention staggering doesn't fully avoid.
CONVEYORS = [
    {"name": "conveyor_pallet_left", "width": "1.1", "length": "5.76", "height": "0.45", "xyz": "2.88 1.3 0.0", "rpy": "0.0 0.0 0.0", "side_rail_height": "0.01", "controllers_spawn_delay": "4.5"},
    {"name": "conveyor_pallet_right", "width": "1.1", "length": "5.76", "height": "0.45", "xyz": "2.88 -1.3 0.0", "rpy": "0.0 0.0 0.0", "side_rail_height": "0.01", "controllers_spawn_delay": "5.0"},
    {"name": "conveyor_pallet_mid", "width": "1.1", "length": "4.96", "height": "0.45", "xyz": "3.28 0.0 0.0", "rpy": "0.0 0.0 0.0", "side_rail_height": "0.01", "controllers_spawn_delay": "5.5"},
    # xyz/length here are intentionally NOT auto-rescaled with the rest of
    # this layout - left exactly as manually placed, per direct
    # instruction. If you reposition pallet_mid/the pedestal later, this
    # one won't automatically stay clear of them - recheck by hand.
    {"name": "conveyor_package_infeed", "width": "0.5", "length": "4.0", "height": "0.4", "xyz": "-1.00 1.5 0.0", "rpy": "0.0 0.0 1.5708", "side_rail_height": "0.12", "controllers_spawn_delay": "6.0"},
]


def conveyor(name: str) -> dict:
    return next(c for c in CONVEYORS if c["name"] == name)


def belt_frame(c: dict):
    """(x, y, z_top, yaw) of a belt: footprint centre, conveying-surface
    height and travel direction (+local X)."""
    x, y, _ = (float(v) for v in c["xyz"].split())
    return x, y, float(c["height"]), float(c["rpy"].split()[2])


def belt_point(c: dict, along: float, across: float = 0.0):
    """World (x, y) of a point given in the belt's own frame: `along` its
    travel axis from the footprint centre, `across` to its left."""
    x, y, _, yaw = belt_frame(c)
    return (x + along * math.cos(yaw) - across * math.sin(yaw),
            y + along * math.sin(yaw) + across * math.cos(yaw))
