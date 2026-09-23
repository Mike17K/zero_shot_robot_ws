import math

import numpy as np
import pytest

from shared_utils.geometry import (euler_from_quaternion, make_pose, matrix_to_pose, offset_pose,
                                   pose_distance, pose_to_matrix, quaternion_from_euler)


def test_euler_roundtrip():
    rpy = (0.3, -0.4, 1.2)
    assert euler_from_quaternion(quaternion_from_euler(*rpy)) == pytest.approx(rpy)


def test_yaw_quaternion():
    x, y, z, w = quaternion_from_euler(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert (z, w) == pytest.approx((math.sin(math.pi / 4), math.cos(math.pi / 4)))


def test_matrix_roundtrip():
    p = make_pose((1.0, 2.0, 3.0), (0.1, 0.2, 0.3))
    d_t, d_r = pose_distance(p, matrix_to_pose(pose_to_matrix(p)))
    assert d_t == pytest.approx(0.0, abs=1e-9) and d_r == pytest.approx(0.0, abs=1e-9)


def test_offset_in_pose_frame():
    # Tool pointing down (roll = pi): backing off -0.15 along its own Z moves up in world.
    grasp = make_pose((1.0, 0.0, 0.5), (math.pi, 0.0, 0.0))
    pre = offset_pose(grasp, make_pose((0.0, 0.0, -0.15)))
    assert (pre.position.x, pre.position.y, pre.position.z) == pytest.approx((1.0, 0.0, 0.65))
    assert np.allclose(pose_to_matrix(pre)[:3, :3], pose_to_matrix(grasp)[:3, :3])
