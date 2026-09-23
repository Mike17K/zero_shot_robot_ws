#!/usr/bin/env python3
"""Plot a pose graph file in 3D (tool positions of the canonical group).

  ros2 run navigator_cli visualize_pose_graph [path/to/pose_graph.json]
"""
import json
import os
import sys

from .shared.constants import CANONICAL_EE_GROUP, CONFIG_FOLDER


def visualize_graph(filename: str):
    import matplotlib.pyplot as plt

    if not os.path.exists(filename):
        print(f"Graph file not found: {filename}")
        return
    with open(filename) as f:
        data = json.load(f)
    nodes = data.get("nodes", [])
    if not nodes:
        print("No nodes in the graph file.")
        return

    positions, labels = {}, {}
    for node in nodes:
        state = node.get("all_ee_poses", {}).get(CANONICAL_EE_GROUP)
        if state is None:
            continue
        pos = state["pos"]
        positions[node["id"]] = (pos["x"], pos["y"], pos["z"])
        labels[node["id"]] = node.get("label") or ""

    ax = plt.figure(figsize=(10, 8)).add_subplot(111, projection='3d')
    for node_id, (x, y, z) in positions.items():
        ax.text(x, y, z, f"{node_id} {labels[node_id]}".strip(), color='blue', fontsize=8)
    xs, ys, zs = zip(*positions.values())
    ax.scatter(xs, ys, zs, c='red', marker='o', s=50, label='Nodes')

    for edge in data.get("edges", []):
        a, b = positions.get(edge["source_id"]), positions.get(edge["target_id"])
        if a and b:
            ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], 'k-', alpha=0.6)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z (up)')
    ax.set_title('Pose graph')
    ax.legend()
    ax.view_init(elev=20, azim=-60)
    plt.show()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    visualize_graph(args[0] if args else os.path.join(CONFIG_FOLDER, "pose_graph.json"))


if __name__ == "__main__":
    main()
