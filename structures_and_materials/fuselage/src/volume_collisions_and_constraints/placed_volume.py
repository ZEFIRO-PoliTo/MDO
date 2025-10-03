# placed_volume.py
"""
Volume placement inside fuselage with collision and distance constraints.

Dependencies:
    numpy
    shapes.py
    collision.py
"""

from typing import List, Tuple, Optional
import random
import numpy as np

from shapes import VolumeShape, Box, OrientedBox, Ellipsoid
from collision import FuselageBounds, is_valid_placement

# -------------------------
# Volume parameters
# -------------------------
class VolumeConfig:
    """
    Configuration for random volume generation
    """
    MIN_SIZE = 0.1  # meters
    MAX_SIZE = 0.3  # meters
    MAX_ATTEMPTS = 200  # attempts per volume
    SAFETY_MARGIN = 0.05  # margin from fuselage wall
    DISTANCE_CONSTRAINT = 0.2  # meters for constrained pairs


# -------------------------
# Volume generation helper
# -------------------------
def random_volume(fuselage: FuselageBounds,
                  shape_type: str = "box") -> VolumeShape:
    """
    Generate a random volume inside fuselage bounds (ignores collisions)
    """
    x = random.uniform(fuselage.min[0], fuselage.max[0])
    y = random.uniform(fuselage.min[1], fuselage.max[1])
    z = random.uniform(fuselage.min[2], fuselage.max[2])

    size_x = random.uniform(VolumeConfig.MIN_SIZE, VolumeConfig.MAX_SIZE)
    size_y = random.uniform(VolumeConfig.MIN_SIZE, VolumeConfig.MAX_SIZE)
    size_z = random.uniform(VolumeConfig.MIN_SIZE, VolumeConfig.MAX_SIZE)

    if shape_type == "box":
        return Box(cx=x, cy=y, cz=z, lx=size_x, ly=size_y, lz=size_z)
    elif shape_type == "oriented_box":
        roll = random.uniform(0, np.pi/4)
        pitch = random.uniform(0, np.pi/4)
        yaw = random.uniform(0, np.pi/4)
        return OrientedBox(cx=x, cy=y, cz=z, lx=size_x, ly=size_y, lz=size_z,
                           roll=roll, pitch=pitch, yaw=yaw)
    elif shape_type == "ellipsoid":
        return Ellipsoid(cx=x, cy=y, cz=z, rx=size_x/2, ry=size_y/2, rz=size_z/2)
    else:
        raise ValueError(f"Unknown shape type: {shape_type}")


# -------------------------
# Main placement function
# -------------------------
def place_volumes(fuselage: FuselageBounds,
                  num_volumes: int = 10,
                  shape_types: List[str] = None,
                  constrained_pairs: List[Tuple[int, int]] = None) -> List[VolumeShape]:
    """
    Place volumes inside fuselage:
      - shape_types: list of shape strings per volume (length=num_volumes)
      - constrained_pairs: list of index tuples (i,j) that must maintain min distance
    """
    if shape_types is None:
        shape_types = ["box"] * num_volumes
    if constrained_pairs is None:
        constrained_pairs = []

    volumes: List[VolumeShape] = []

    # Map constrained indices to volume pairs
    min_dist_pairs: List[Tuple[VolumeShape, VolumeShape]] = []

    for i in range(num_volumes):
        shape_type = shape_types[i]

        for attempt in range(VolumeConfig.MAX_ATTEMPTS):
            vol = random_volume(fuselage, shape_type)

            # Validate against fuselage, existing volumes, distance constraints
            if is_valid_placement(vol, fuselage, volumes, min_dist_pairs, VolumeConfig.SAFETY_MARGIN):
                volumes.append(vol)
                # Update min_dist_pairs for future volumes
                for j1, j2 in constrained_pairs:
                    if i == j1 or i == j2:
                        # Only add pairs when both volumes exist
                        if i == j1 and j2 < len(volumes):
                            min_dist_pairs.append((vol, volumes[j2]))
                        elif i == j2 and j1 < len(volumes):
                            min_dist_pairs.append((volumes[j1], vol))
                break
        else:
            print(f"⚠️ Could not place volume {i} after {VolumeConfig.MAX_ATTEMPTS} attempts")

    return volumes
