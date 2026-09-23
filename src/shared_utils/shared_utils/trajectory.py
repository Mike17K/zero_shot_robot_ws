"""JointTrajectory helpers: time conversion, cleanup, checks and resampling.

Resampling functions take two JointTrajectoryPoints and return the points
between them (both ends included) at frequency_hz, with time_from_start
continuing from p0's.
"""
import math
from dataclasses import dataclass
from typing import List, Literal, Sequence

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

InterpolationKind = Literal['linear', 'cubic', 'trapezoidal']


# ── Time ──────────────────────────────────────────────────────────────────────

def duration_to_sec(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def sec_to_duration(t: float) -> Duration:
    sec = int(math.floor(t))
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def trajectory_duration(traj: JointTrajectory) -> float:
    return duration_to_sec(traj.points[-1].time_from_start) if traj.points else 0.0


# ── Cleanup / checks ──────────────────────────────────────────────────────────

def drop_duplicate_timestamps(traj: JointTrajectory) -> JointTrajectory:
    """Remove points whose time_from_start equals the previous point's -
    controllers reject non-increasing times."""
    out = JointTrajectory(header=traj.header, joint_names=list(traj.joint_names))
    last = None
    for p in traj.points:
        t = (p.time_from_start.sec, p.time_from_start.nanosec)
        if t != last:
            out.points.append(p)
            last = t
    return out


def max_start_deviation(traj: JointTrajectory, current: dict[str, float]) -> float:
    """Largest |first point - current| over the trajectory's joints (rad), or
    inf if a joint is missing from `current` ({name: position})."""
    if not traj.points:
        return 0.0
    worst = 0.0
    for name, pos in zip(traj.joint_names, traj.points[0].positions):
        if name not in current:
            return math.inf
        worst = max(worst, abs(pos - current[name]))
    return worst


def concatenate(trajectories: Sequence[JointTrajectory]) -> JointTrajectory:
    """Join trajectories back to back (same joints, same order), shifting
    each one's times to start where the previous ended. The first point of
    every later segment is dropped since it repeats the previous last point."""
    if not trajectories:
        return JointTrajectory()
    out = JointTrajectory(header=trajectories[0].header,
                          joint_names=list(trajectories[0].joint_names))
    offset = 0.0
    for i, traj in enumerate(trajectories):
        if list(traj.joint_names) != list(out.joint_names):
            raise ValueError('all trajectories must have the same joint_names order')
        points = traj.points if i == 0 else traj.points[1:]
        for p in points:
            q = JointTrajectoryPoint(positions=list(p.positions), velocities=list(p.velocities),
                                     accelerations=list(p.accelerations), effort=list(p.effort))
            q.time_from_start = sec_to_duration(offset + duration_to_sec(p.time_from_start))
            out.points.append(q)
        offset += trajectory_duration(traj)
    return out


# ── Resampling between two points ─────────────────────────────────────────────

@dataclass
class _Cubic:
    a0: float
    a1: float
    a2: float
    a3: float

    def at(self, t: float) -> tuple[float, float, float]:
        return (self.a0 + self.a1 * t + self.a2 * t * t + self.a3 * t ** 3,
                self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t * t,
                2.0 * self.a2 + 6.0 * self.a3 * t)


def _segment(p0: JointTrajectoryPoint, p1: JointTrajectoryPoint, frequency_hz: float):
    if frequency_hz <= 0.0:
        raise ValueError(f'frequency_hz must be positive, got {frequency_hz}')
    n = len(p0.positions)
    if len(p1.positions) != n:
        raise ValueError('start and end points must have the same joint count')
    t0 = duration_to_sec(p0.time_from_start)
    T = duration_to_sec(p1.time_from_start) - t0
    if T <= 0.0:
        raise ValueError(f'end time must be after start time (got T={T:.4f}s)')
    steps = max(1, int(round(T * frequency_hz)))
    times = [min(s / frequency_hz, T) for s in range(steps)] + [T]
    return n, t0, T, times


def _velocities(p: JointTrajectoryPoint, n: int) -> list[float]:
    return list(p.velocities) if len(p.velocities) == n else [0.0] * n


def _point(t_abs: float, pos, vel, acc) -> JointTrajectoryPoint:
    return JointTrajectoryPoint(positions=list(pos), velocities=list(vel),
                                accelerations=list(acc), time_from_start=sec_to_duration(t_abs))


def interpolate_linear(p0: JointTrajectoryPoint, p1: JointTrajectoryPoint,
                       frequency_hz: float) -> List[JointTrajectoryPoint]:
    n, t0, T, times = _segment(p0, p1, frequency_hz)
    vel = [(b - a) / T for a, b in zip(p0.positions, p1.positions)]
    return [_point(t0 + t, [a + v * t for a, v in zip(p0.positions, vel)], vel, [0.0] * n)
            for t in times]


def interpolate_cubic(p0: JointTrajectoryPoint, p1: JointTrajectoryPoint,
                      max_velocities: Sequence[float], max_accelerations: Sequence[float],
                      frequency_hz: float) -> List[JointTrajectoryPoint]:
    """Cubic per joint matching both ends' positions and velocities (clamped
    to max_velocities). Raises RuntimeError if the peak acceleration of any
    joint exceeds max_accelerations."""
    n, t0, T, times = _segment(p0, p1, frequency_hz)
    if len(max_velocities) != n or len(max_accelerations) != n:
        raise ValueError('limit arrays must match the joint count')
    v0 = [max(-abs(m), min(abs(m), v)) for v, m in zip(_velocities(p0, n), max_velocities)]
    v1 = [max(-abs(m), min(abs(m), v)) for v, m in zip(_velocities(p1, n), max_velocities)]
    coeffs = []
    for j in range(n):
        q0, q1 = p0.positions[j], p1.positions[j]
        c = _Cubic(q0, v0[j],
                   (3.0 * (q1 - q0) - (2.0 * v0[j] + v1[j]) * T) / T ** 2,
                   (2.0 * (q0 - q1) + (v0[j] + v1[j]) * T) / T ** 3)
        peak = max(abs(2.0 * c.a2), abs(2.0 * c.a2 + 6.0 * c.a3 * T))
        if peak > abs(max_accelerations[j]) + 1e-6:
            raise RuntimeError(
                f'joint {j}: peak acceleration {peak:.3f} > limit {max_accelerations[j]:.3f}')
        coeffs.append(c)
    out = []
    for t in times:
        pva = [c.at(t) for c in coeffs]
        out.append(_point(t0 + t, *zip(*pva)))
    return out


def _trapezoid_peak_velocity(dq: float, v0: float, v1: float, a: float, vmax: float, T: float) -> float:
    """Signed cruise velocity of a trapezoidal profile covering dq in exactly
    T with accel a. Solved in closed form from
    dq = vp*T - ((vp - v0)^2 + (vp - v1)^2) / (2a)."""
    sign = 1.0 if dq >= 0.0 else -1.0
    q, v0, v1 = abs(dq), v0 * sign, v1 * sign
    # a*vp^2 terms: vp^2 - vp*(a*T + v0 + v1) + (v0^2 + v1^2)/2 + a*q = 0
    b = a * T + v0 + v1
    disc = b * b - 4.0 * ((v0 * v0 + v1 * v1) / 2.0 + a * q)
    if disc < 0.0:
        raise RuntimeError('acceleration limit too low to cover the motion in the given time')
    vp = (b - math.sqrt(disc)) / 2.0  # smaller root = longest cruise phase
    if vp > abs(vmax) + 1e-9:
        raise RuntimeError(f'required cruise velocity {vp:.3f} exceeds the limit {abs(vmax):.3f}')
    if (vp - v0) / a < -1e-9 or (vp - v1) / a < -1e-9 or T - (2 * vp - v0 - v1) / a < -1e-9:
        raise RuntimeError('no trapezoidal profile fits these boundary velocities')
    return vp * sign


def interpolate_trapezoidal(p0: JointTrajectoryPoint, p1: JointTrajectoryPoint,
                            max_velocities: Sequence[float], max_accelerations: Sequence[float],
                            frequency_hz: float) -> List[JointTrajectoryPoint]:
    """Accelerate / cruise / decelerate per joint, each joint using its own
    max acceleration and finishing exactly at p1's time."""
    n, t0, T, times = _segment(p0, p1, frequency_hz)
    if len(max_velocities) != n or len(max_accelerations) != n:
        raise ValueError('limit arrays must match the joint count')
    v0s, v1s = _velocities(p0, n), _velocities(p1, n)
    profiles = []
    for j in range(n):
        dq = p1.positions[j] - p0.positions[j]
        a = abs(max_accelerations[j])
        if a <= 0.0:
            raise ValueError(f'joint {j}: acceleration limit must be positive')
        vp = _trapezoid_peak_velocity(dq, v0s[j], v1s[j], a, max_velocities[j], T)
        sa = a if vp >= v0s[j] else -a     # sign of the accel phase
        sd = a if vp >= v1s[j] else -a     # sign of the decel phase
        ta, td = abs(vp - v0s[j]) / a, abs(vp - v1s[j]) / a
        profiles.append((p0.positions[j], v0s[j], vp, sa, sd, ta, max(0.0, T - ta - td)))
    out = []
    for t in times:
        pos, vel, acc = [], [], []
        for q0, v0, vp, sa, sd, ta, tc in profiles:
            if t <= ta:
                pos.append(q0 + v0 * t + 0.5 * sa * t * t)
                vel.append(v0 + sa * t)
                acc.append(sa)
            elif t <= ta + tc:
                pos.append(q0 + v0 * ta + 0.5 * sa * ta * ta + vp * (t - ta))
                vel.append(vp)
                acc.append(0.0)
            else:
                tau = t - ta - tc
                pos.append(q0 + v0 * ta + 0.5 * sa * ta * ta + vp * tc
                           + vp * tau - 0.5 * sd * tau * tau)
                vel.append(vp - sd * tau)
                acc.append(-sd)
        out.append(_point(t0 + t, pos, vel, acc))
    return out


def resample(traj: JointTrajectory, frequency_hz: float, kind: InterpolationKind = 'linear',
             max_velocities: Sequence[float] = (), max_accelerations: Sequence[float] = ()) -> JointTrajectory:
    """Resample every segment of traj at frequency_hz."""
    out = JointTrajectory(header=traj.header, joint_names=list(traj.joint_names))
    for i, (p0, p1) in enumerate(zip(traj.points, traj.points[1:])):
        if kind == 'linear':
            seg = interpolate_linear(p0, p1, frequency_hz)
        elif kind == 'cubic':
            seg = interpolate_cubic(p0, p1, max_velocities, max_accelerations, frequency_hz)
        elif kind == 'trapezoidal':
            seg = interpolate_trapezoidal(p0, p1, max_velocities, max_accelerations, frequency_hz)
        else:
            raise ValueError(f'unknown interpolation kind: {kind}')
        out.points.extend(seg if i == 0 else seg[1:])
    return out
