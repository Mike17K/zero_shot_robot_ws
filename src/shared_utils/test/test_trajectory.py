import math

import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from shared_utils.trajectory import (concatenate, drop_duplicate_timestamps, duration_to_sec,
                                     interpolate_cubic, interpolate_linear, interpolate_trapezoidal,
                                     max_start_deviation, resample, sec_to_duration)


def _pt(pos, t, vel=()):
    return JointTrajectoryPoint(positions=list(pos), velocities=list(vel),
                                time_from_start=sec_to_duration(t))


def _traj(points, names=('a', 'b')):
    return JointTrajectory(joint_names=list(names), points=points)


def test_duration_roundtrip():
    for t in (0.0, 0.5, 1.999999999, 12.25):
        assert duration_to_sec(sec_to_duration(t)) == pytest.approx(t, abs=1e-9)


def test_linear_endpoints_and_rate():
    pts = interpolate_linear(_pt([0, 0], 1.0), _pt([1, -2], 2.0), 10.0)
    assert len(pts) == 11
    assert duration_to_sec(pts[0].time_from_start) == pytest.approx(1.0)
    assert duration_to_sec(pts[-1].time_from_start) == pytest.approx(2.0)
    assert list(pts[-1].positions) == pytest.approx([1, -2])
    assert list(pts[5].positions) == pytest.approx([0.5, -1.0])


def test_cubic_matches_boundaries_and_checks_accel():
    pts = interpolate_cubic(_pt([0], 0.0), _pt([1], 2.0), [5.0], [10.0], 50.0)
    assert pts[0].positions[0] == pytest.approx(0.0)
    assert pts[-1].positions[0] == pytest.approx(1.0)
    assert pts[-1].velocities[0] == pytest.approx(0.0, abs=1e-9)
    with pytest.raises(RuntimeError):
        interpolate_cubic(_pt([0], 0.0), _pt([10], 0.5), [100.0], [1.0], 50.0)


@pytest.mark.parametrize('dq', [1.0, -1.0])
def test_trapezoidal_reaches_goal_on_time(dq):
    pts = interpolate_trapezoidal(_pt([0.0], 0.0), _pt([dq], 2.0), [2.0], [2.0], 100.0)
    assert pts[-1].positions[0] == pytest.approx(dq, abs=1e-6)
    assert pts[-1].velocities[0] == pytest.approx(0.0, abs=1e-6)
    assert max(abs(p.velocities[0]) for p in pts) <= 2.0 + 1e-9
    positions = [p.positions[0] for p in pts]
    assert positions == sorted(positions, reverse=dq < 0)  # monotonic


def test_trapezoidal_rejects_impossible():
    with pytest.raises(RuntimeError):
        interpolate_trapezoidal(_pt([0.0], 0.0), _pt([10.0], 1.0), [2.0], [2.0], 100.0)


def test_resample_and_cleanup():
    traj = _traj([_pt([0, 0], 0.0), _pt([1, 1], 1.0), _pt([1, 1], 1.0), _pt([2, 0], 2.0)])
    clean = drop_duplicate_timestamps(traj)
    assert len(clean.points) == 3
    fine = resample(clean, 10.0)
    assert len(fine.points) == 21
    times = [duration_to_sec(p.time_from_start) for p in fine.points]
    assert all(b > a for a, b in zip(times, times[1:]))


def test_concatenate_and_start_deviation():
    a = _traj([_pt([0, 0], 0.0), _pt([1, 1], 1.0)])
    b = _traj([_pt([1, 1], 0.0), _pt([2, 2], 0.5)])
    joined = concatenate([a, b])
    assert len(joined.points) == 3
    assert duration_to_sec(joined.points[-1].time_from_start) == pytest.approx(1.5)
    assert max_start_deviation(a, {'a': 0.1, 'b': -0.2}) == pytest.approx(0.2)
    assert max_start_deviation(a, {'a': 0.0}) == math.inf
