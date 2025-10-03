"""
shapes.py
Geometric shape definitions and basic collision utilities used by the
volume placement / collision-detection pipeline.

Dependencies:
    numpy
"""

from dataclasses import dataclass, asdict
from typing import Tuple, List
import numpy as np
import math

EPS = 1e-9


# -------------------------
# Rotation helpers
# -------------------------
def euler_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles (roll, pitch, yaw) in radians to a 3x3 rotation matrix.
    Convention: intrinsic rotations applied in order roll (X), pitch (Y), yaw (Z),
    combined matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]])
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


# -------------------------
# Base shape
# -------------------------
@dataclass
class VolumeShape:
    """
    Base class for volume shapes. Subclasses must implement:
      - get_corners() -> np.ndarray shape (8,3) for box-like shapes
      - get_aabb() -> (min:np.ndarray(3,), max:np.ndarray(3,))
      - contains_point(pt) -> bool
    """
    def get_corners(self) -> np.ndarray:
        raise NotImplementedError

    def get_aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        corners = self.get_corners()
        return np.min(corners, axis=0), np.max(corners, axis=0)

    def contains_point(self, pt: Tuple[float, float, float]) -> bool:
        raise NotImplementedError

    def to_dict(self):
        return asdict(self)


# -------------------------
# Ellipsoid
# -------------------------
@dataclass
class Ellipsoid(VolumeShape):
    cx: float
    cy: float
    cz: float
    rx: float  # radius along X
    ry: float  # radius along Y
    rz: float  # radius along Z

    def get_corners(self) -> np.ndarray:
        """Return AABB corners (8 points) for ellipsoid (simple center +/- radii)."""
        c = np.array([self.cx, self.cy, self.cz])
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        radii = np.array([self.rx, self.ry, self.rz])
        return c + signs * radii

    def contains_point(self, pt: Tuple[float, float, float]) -> bool:
        x, y, z = pt
        val = ((x - self.cx) / (self.rx + EPS))**2 + ((y - self.cy) / (self.ry + EPS))**2 + ((z - self.cz) / (self.rz + EPS))**2
        return val <= 1.0

    def bounding_sphere_radius(self) -> float:
        return max(self.rx, self.ry, self.rz)


# -------------------------
# Axis-aligned Box (Box/cube)
# -------------------------
@dataclass
class Box(VolumeShape):
    cx: float
    cy: float
    cz: float
    lx: float  # full length along X
    ly: float
    lz: float

    def half_sizes(self) -> np.ndarray:
        return np.array([self.lx / 2.0, self.ly / 2.0, self.lz / 2.0])

    def get_corners(self) -> np.ndarray:
        """Return 8 corners of axis-aligned box."""
        c = np.array([self.cx, self.cy, self.cz])
        hx, hy, hz = self.half_sizes()
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        offsets = signs * np.array([hx, hy, hz])
        return c + offsets

    def contains_point(self, pt: Tuple[float, float, float]) -> bool:
        px, py, pz = pt
        amin, amax = self.get_aabb()
        return (amin[0] - EPS <= px <= amax[0] + EPS) and (amin[1] - EPS <= py <= amax[1] + EPS) and (amin[2] - EPS <= pz <= amax[2] + EPS)


# -------------------------
# Oriented Box / Parallelepiped (OBB)
# -------------------------
@dataclass
class OrientedBox(VolumeShape):
    cx: float
    cy: float
    cz: float
    lx: float
    ly: float
    lz: float
    # orientation expressed as Euler angles in radians (roll, pitch, yaw)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def half_sizes(self) -> np.ndarray:
        return np.array([self.lx / 2.0, self.ly / 2.0, self.lz / 2.0])

    def rotation_matrix(self) -> np.ndarray:
        return euler_to_rotmat(self.roll, self.pitch, self.yaw)

    def get_local_axes(self) -> np.ndarray:
        """Return 3 orthonormal column vectors (axis_x, axis_y, axis_z) as a 3x3 matrix."""
        R = self.rotation_matrix()
        return R  # columns are axes if we use R @ local_vector

    def get_corners(self) -> np.ndarray:
        """Return the 8 world-space corners of the oriented box."""
        c = np.array([self.cx, self.cy, self.cz])
        hx, hy, hz = self.half_sizes()
        local_corners = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        R = self.rotation_matrix()
        world = (R @ local_corners.T).T + c
        return world

    def get_obb_axes_and_halfsizes(self):
        R = self.rotation_matrix()
        axes = [R[:, 0], R[:, 1], R[:, 2]]  # each a 3-vector
        half = self.half_sizes()
        return axes, half

    def contains_point(self, pt: Tuple[float, float, float]) -> bool:
        # Transform point into local box coordinates: p_local = R^T (p - c)
        p = np.array(pt)
        c = np.array([self.cx, self.cy, self.cz])
        R = self.rotation_matrix()
        p_local = R.T @ (p - c)
        hx, hy, hz = self.half_sizes()
        return (abs(p_local[0]) <= hx + EPS) and (abs(p_local[1]) <= hy + EPS) and (abs(p_local[2]) <= hz + EPS)


# -------------------------
# Utility collision functions
# -------------------------
def aabb_overlap(a_min: np.ndarray, a_max: np.ndarray, b_min: np.ndarray, b_max: np.ndarray) -> bool:
    """Axis-aligned bounding box overlap test (broad-phase)."""
    return not (np.any(a_max < b_min - EPS) or np.any(b_max < a_min - EPS))


def obb_intersect(box1: OrientedBox, box2: OrientedBox) -> bool:
    """
    Oriented box vs oriented box intersection using the Separating Axis Theorem (SAT).
    Implementation follows the standard OBB overlap test (15 separating axes).
    """
    C1 = np.array([box1.cx, box1.cy, box1.cz])
    C2 = np.array([box2.cx, box2.cy, box2.cz])

    # local axes (unit vectors)
    R1 = box1.rotation_matrix()
    R2 = box2.rotation_matrix()

    A = [R1[:, 0], R1[:, 1], R1[:, 2]]
    B = [R2[:, 0], R2[:, 1], R2[:, 2]]

    a = box1.half_sizes()
    b = box2.half_sizes()

    # Rotation matrix expressing B in A's coords: R[i][j] = A_i dot B_j
    R = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            R[i, j] = float(np.dot(A[i], B[j]))

    # Translation vector t in world coords then in A's coordinates
    t_world = C2 - C1
    t = np.array([np.dot(t_world, A[i]) for i in range(3)])

    # Compute absolute values with epsilon to account for near-parallel axes
    absR = np.abs(R) + 1e-12

    # Test axes A0, A1, A2
    for i in range(3):
        ra = a[i]
        rb = b[0] * absR[i, 0] + b[1] * absR[i, 1] + b[2] * absR[i, 2]
        if abs(t[i]) > ra + rb + EPS:
            return False

    # Test axes B0, B1, B2
    for j in range(3):
        ra = a[0] * absR[0, j] + a[1] * absR[1, j] + a[2] * absR[2, j]
        rb = b[j]
        t_j = abs(t[0] * R[0, j] + t[1] * R[1, j] + t[2] * R[2, j])
        if t_j > ra + rb + EPS:
            return False

    # Test cross products A_i x B_j
    # There are 9 tests
    for i in range(3):
        for j in range(3):
            ra = (a[(i + 1) % 3] * absR[(i + 2) % 3, j] +
                  a[(i + 2) % 3] * absR[(i + 1) % 3, j])
            rb = (b[(j + 1) % 3] * absR[i, (j + 2) % 3] +
                  b[(j + 2) % 3] * absR[i, (j + 1) % 3])
            t_ij = abs(t[(i + 2) % 3] * R[(i + 1) % 3, j] - t[(i + 1) % 3] * R[(i + 2) % 3, j])
            if t_ij > ra + rb + EPS:
                return False

    # No separating axis found -> overlap
    return True


def box_box_collision(box1: VolumeShape, box2: VolumeShape) -> bool:
    """
    Generic box-vs-box collision:
      - If both are OrientedBox -> OBB SAT test
      - Otherwise fall back to AABB overlap
    """
    if isinstance(box1, OrientedBox) and isinstance(box2, OrientedBox):
        # Quick broad-phase AABB test first
        a_min, a_max = box1.get_aabb()
        b_min, b_max = box2.get_aabb()
        if not aabb_overlap(a_min, a_max, b_min, b_max):
            return False
        return obb_intersect(box1, box2)
    else:
        a_min, a_max = box1.get_aabb()
        b_min, b_max = box2.get_aabb()
        return aabb_overlap(a_min, a_max, b_min, b_max)


def ellipsoid_box_collision(ell: Ellipsoid, box: VolumeShape) -> bool:
    """
    Conservative collision test between ellipsoid and box:
      - Use bounding sphere for ellipsoid (radius = max(rx,ry,rz)) and AABB for box.
      - This is conservative (may return True even when shapes do not overlap),
        but is safe for rejection (ensures no penetration passes undetected).
    TODO: implement exact ellipsoid-box overlap test later.
    """
    sph_r = ell.bounding_sphere_radius()
    # sphere center
    sc = np.array([ell.cx, ell.cy, ell.cz])
    bmin, bmax = box.get_aabb()

    # compute closest point on AABB to sphere center
    closest = np.maximum(bmin, np.minimum(sc, bmax))
    d2 = np.sum((closest - sc)**2)
    return d2 <= (sph_r + EPS)**2


# -------------------------
# Example convenience factory and JSON helper
# -------------------------
def shape_from_dict(d: dict) -> VolumeShape:
    """
    Small helper to reconstruct shapes from dicts (e.g. JSON).
    supported keys:
      - type: 'ellipsoid' | 'box' | 'oriented_box'
    """
    t = d.get("type", "").lower()
    if t == "ellipsoid":
        return Ellipsoid(d["cx"], d["cy"], d["cz"], d["rx"], d["ry"], d["rz"])
    if t == "box":
        return Box(d["cx"], d["cy"], d["cz"], d["lx"], d["ly"], d["lz"])
    if t == "oriented_box":
        return OrientedBox(d["cx"], d["cy"], d["cz"], d["lx"], d["ly"], d["lz"],
                           d.get("roll", 0.0), d.get("pitch", 0.0), d.get("yaw", 0.0))
    raise ValueError(f"Unknown shape type: {t}")
