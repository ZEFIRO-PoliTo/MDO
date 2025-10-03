# collision.py
"""
High-level collision and constraint checking for placed volumes.
Uses geometric utilities from shapes.py
"""

from typing import List, Tuple
import numpy as np

from shapes import VolumeShape, Ellipsoid, Box, OrientedBox, box_box_collision, ellipsoid_box_collision

# -------------------------
# Fuselage bounds wrapper
# -------------------------
class FuselageBounds:
    """
    Simple wrapper representing fuselage as an axis-aligned box (oriented boxes can be added later).
    For now, bounds are axis-aligned, defined by min/max corners.
    """
    def __init__(self, bbox_min: Tuple[float, float, float], bbox_max: Tuple[float, float, float]):
        self.min = np.array(bbox_min)
        self.max = np.array(bbox_max)

    def contains_volume(self, vol: VolumeShape, safety_margin: float = 0.0) -> bool:
        """
        Check if the volume is fully inside fuselage bounds (AABB)
        """
        vmin, vmax = vol.get_aabb()
        vmin += safety_margin
        vmax -= safety_margin
        return np.all(vmin >= self.min) and np.all(vmax <= self.max)


# -------------------------
# Volume-vs-volume collision
# -------------------------
def check_collision(vol: VolumeShape, others: List[VolumeShape]) -> bool:
    """
    Check if vol collides with any volume in others.
    """
    for other in others:
        if isinstance(vol, Ellipsoid) or isinstance(other, Ellipsoid):
            if ellipsoid_box_collision(vol if isinstance(vol, Ellipsoid) else other,
                                       other if not isinstance(other, Ellipsoid) else vol):
                return True
        else:
            if box_box_collision(vol, other):
                return True
    return False


# -------------------------
# Distance constraint check
# -------------------------
def check_min_distance(vol: VolumeShape, others: List[VolumeShape], min_dist: float) -> bool:
    """
    Ensure vol is at least min_dist away from all others.
    Uses AABB center distance as conservative check.
    """
    v_center = np.mean(vol.get_corners(), axis=0)
    for other in others:
        o_center = np.mean(other.get_corners(), axis=0)
        dist = np.linalg.norm(v_center - o_center)
        if dist < min_dist:
            return False
    return True


# -------------------------
# High-level validator
# -------------------------
def is_valid_placement(vol: VolumeShape,
                       fuselage: FuselageBounds,
                       placed: List[VolumeShape],
                       min_dist_pairs: List[Tuple[VolumeShape, VolumeShape]] = [],
                       safety_margin: float = 0.0) -> bool:
    """
    Check if volume placement is valid:
      1. Inside fuselage
      2. No collisions with existing volumes
      3. Distance constraints respected
    """
    # Inside fuselage
    if not fuselage.contains_volume(vol, safety_margin):
        return False

    # Collisions with existing volumes
    if check_collision(vol, placed):
        return False

    # Distance constraints
    for v1, v2 in min_dist_pairs:
        if vol in (v1, v2):
            other = v2 if vol is v1 else v1
            if not check_min_distance(vol, [other], min_dist=0.0):
                return False

    return True
