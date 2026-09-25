"""Box top face from a segmentation mask + depth image (numpy / OpenCV only,
no ROS, no torch - unit-testable on synthetic data).

Frames: the camera's optical frame (z forward, x right, y down), metres.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class TopFace:
    center: np.ndarray        # (3,) top-face centre
    R: np.ndarray             # (3, 3) columns: long edge, short edge, normal (towards the camera)
    size: Tuple[float, float]  # (long, short) edge lengths
    inliers: int              # points on the fitted plane
    tilt_deg: float           # angle between the face normal and the camera axis

    def corners(self) -> np.ndarray:
        """(4, 3) face corners, in order around the rectangle."""
        hx, hy = self.size[0] / 2.0, self.size[1] / 2.0
        local = np.array([[hx, hy, 0.0], [-hx, hy, 0.0], [-hx, -hy, 0.0], [hx, -hy, 0.0]])
        return self.center + local @ self.R.T

    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.center
        return T


@dataclass
class FaceParams:
    near: float = 0.10            # m, depth sensor clip planes (group_a_macro.xacro)
    far: float = 3.0
    plane_tolerance: float = 0.005
    ransac_iterations: int = 100
    min_points: int = 50          # valid depth pixels in the mask
    min_inliers: int = 80         # points on the plane
    max_tilt_deg: float = 35.0    # steeper = a side face (or the camera looks too obliquely)
    min_size: float = 0.05        # m, face edges
    max_size: float = 0.50
    max_aspect: float = 4.0       # long / short edge - belt rollers and rails are long thin strips


def backproject(mask: np.ndarray, depth: np.ndarray, K: np.ndarray,
                near: float, far: float) -> np.ndarray:
    """(N, 3) points of the mask's valid depth pixels."""
    ys, xs = np.nonzero(mask)
    zs = depth[ys, xs].astype(np.float64)
    ok = np.isfinite(zs) & (zs > near) & (zs < far)
    xs, ys, zs = xs[ok], ys[ok], zs[ok]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.stack([(xs - cx) * zs / fx, (ys - cy) * zs / fy, zs], axis=1)


def fit_plane(points: np.ndarray, tolerance: float, iterations: int,
              rng: Optional[np.random.Generator] = None) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
    """RANSAC plane n.p + d = 0 with the most inliers, refined by least squares
    on them. Returns (unit normal facing the camera origin, d, inlier mask)."""
    n_pts = len(points)
    if n_pts < 3:
        return None
    rng = rng or np.random.default_rng(0)
    best = None
    for _ in range(iterations):
        a, b, c = points[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        inl = np.abs(points @ n - n @ a) < tolerance
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 3:
        return None
    # Refine: normal = smallest principal direction of the inliers.
    P = points[best]
    centroid = P.mean(axis=0)
    n = np.linalg.svd(P - centroid, full_matrices=False)[2][2]
    if n @ centroid > 0:  # face the camera (origin)
        n = -n
    d = -n @ centroid
    inl = np.abs(points @ n + d) < tolerance
    return n, d, inl


def top_face(points: np.ndarray, params: FaceParams,
             rng: Optional[np.random.Generator] = None) -> Tuple[Optional[TopFace], str]:
    """Largest plane in the points as an oriented rectangle, or (None, reason)."""
    if len(points) < params.min_points:
        return None, f'{len(points)} depth points'
    fit = fit_plane(points, params.plane_tolerance, params.ransac_iterations, rng)
    if fit is None:
        return None, 'no plane'
    n, _, inl = fit
    if inl.sum() < params.min_inliers:
        return None, f'{int(inl.sum())} plane inliers'
    tilt = float(np.degrees(np.arccos(np.clip(-n[2], -1.0, 1.0))))
    if tilt > params.max_tilt_deg:
        return None, f'tilted {tilt:.0f} deg (side face?)'

    # In-plane basis, then the minimum-area rectangle around the inliers.
    P = points[inl]
    origin = P.mean(axis=0)
    u = np.cross(n, [0.0, 1.0, 0.0] if abs(n[1]) < 0.9 else [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    uv = np.stack([(P - origin) @ u, (P - origin) @ v], axis=1).astype(np.float32)
    (cu, cv), _, _ = cv2.minAreaRect(uv)
    box = cv2.boxPoints(cv2.minAreaRect(uv))  # 4 corners, consecutive
    e1, e2 = box[1] - box[0], box[2] - box[1]
    l1, l2 = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
    long_2d, (long_len, short_len) = (e1, (l1, l2)) if l1 >= l2 else (e2, (l2, l1))
    if short_len < params.min_size or long_len > params.max_size:
        return None, f'size {long_len:.2f}x{short_len:.2f} m'
    if long_len > params.max_aspect * short_len:
        return None, f'aspect {long_len / short_len:.1f} (strip, not a box top)'

    x = long_2d[0] * u + long_2d[1] * v
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    center = origin + cu * u + cv * v
    return TopFace(center, np.stack([x, y, n], axis=1), (long_len, short_len), int(inl.sum()), tilt), 'ok'


def drop_fraction(face: TopFace, mask: np.ndarray, depth: np.ndarray, K: np.ndarray,
                  params: FaceParams, min_height: float, ring_px: int = 6) -> float:
    """Share of the pixels in a ring just outside the mask that lie at least
    min_height BELOW the face plane (1.0 when the ring has no valid depth,
    i.e. nothing measurable around it). A box top stands above what surrounds
    it; a flat mark or patch on the belt does not."""
    kernel = np.ones((2 * ring_px + 1, 2 * ring_px + 1), np.uint8)
    ring = cv2.dilate(mask.astype(np.uint8), kernel).astype(bool) & ~mask
    # Pixels with no depth return (beyond the far clip) are certainly lower.
    zs = depth[ring]
    far = int((~np.isfinite(zs) | (zs >= params.far)).sum())
    pts = backproject(ring, depth, K, params.near, params.far)
    if len(pts) + far == 0:
        return 1.0
    n = face.R[:, 2]
    below = (pts - face.center) @ n < -min_height
    return float((int(below.sum()) + far) / (len(pts) + far))


def project(points: np.ndarray, K: np.ndarray) -> np.ndarray:
    """(N, 2) pixel coordinates of camera-frame points."""
    p = points @ K.T
    return p[:, :2] / p[:, 2:3]


def dedupe(faces: list, min_separation: float) -> list:
    """Drop faces whose centre is within min_separation of a face with more
    inliers (SAM proposes nested masks: whole box, top face, parts of it)."""
    kept = []
    for f in sorted(faces, key=lambda f: -f.inliers):
        if all(np.linalg.norm(f.center - k.center) >= min_separation for k in kept):
            kept.append(f)
    return kept
