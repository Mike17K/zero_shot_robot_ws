import os
import tempfile

os.environ.setdefault('NAVIGATOR_CLI_CONFIG_DIR', tempfile.mkdtemp())

from navigator_cli.robots.interface import MoveGroupState, RobotPose  # noqa: E402
from navigator_cli.shared.constants import CANONICAL_EE_GROUP  # noqa: E402
from navigator_cli.shared.graph import Edge, Graph  # noqa: E402
from navigator_cli.shared.types import Point, Quaternion  # noqa: E402


def _pose(x, joints, **kw):
    state = MoveGroupState(pos=Point(x, 0.0, 1.0), rot=Quaternion(), joints=tuple(joints))
    return RobotPose(all_ee_poses={CANONICAL_EE_GROUP: state}, is_joint_goal=True, **kw)


def _connect(g, a, b):
    g.add_edge(Edge(source_id=a.id, target_id=b.id, move_group_for_transition=CANONICAL_EE_GROUP))
    g.add_edge(Edge(source_id=b.id, target_id=a.id, move_group_for_transition=CANONICAL_EE_GROUP))


def _line_graph():
    g = Graph[RobotPose](node_cls=RobotPose)
    nodes = [g.add_node(_pose(i * 0.5, [i * 0.1] * 6, label=f'n{i}')) for i in range(3)]
    _connect(g, nodes[0], nodes[1])
    _connect(g, nodes[1], nodes[2])
    return g, nodes


def test_add_node_assigns_ids_to_stored_nodes():
    g, nodes = _line_graph()
    assert [n.id for n in nodes] == [0, 1, 2]
    assert all(g._id_to_pose[i].id == i for i in range(3))


def test_shortest_path_and_nearby():
    g, nodes = _line_graph()
    path = g.shortest_path(nodes[0], nodes[2])
    assert [p.id for p, _, _ in path] == [0, 1, 2]
    nearby = g.find_nearby_nodes(nodes[0], current_all_joints=[0.19] * 6)
    assert nearby[0][0].id == 2


def test_save_and_reload_roundtrip(tmp_path):
    g, nodes = _line_graph()
    g.set_file(str(tmp_path / 'graph.json'))
    g.save_to_file()
    h = Graph[RobotPose](node_cls=RobotPose)
    h.set_file(str(tmp_path / 'graph.json'))
    h.load_from_file()
    assert sorted(h._id_to_pose) == [0, 1, 2]
    assert h.get_by_label('n2').canonical.joints == nodes[2].canonical.joints
    assert [p.id for p, _, _ in h.shortest_path(h._id_to_pose[2], h._id_to_pose[0])] == [2, 1, 0]
