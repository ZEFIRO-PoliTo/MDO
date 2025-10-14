"""
Fuselage Volume Optimization Script
====================================
This script generates multiple fuselage configurations with slightly perturbed
parameters, places volumes inside each with precise collision detection and 
distance constraints, computes their total volume, and selects the top 20 
configurations with minimum fuselage volume.

Key Features:
- Precise geometric validation using multiple surface points
- Multi-level validation to ensure consistency
- Distance constraints between volumes
- Collision detection with safety margins
- Automated VSP file generation and analysis
"""

import openvsp as vsp
import numpy as np
import os
import json
import csv
import math
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, asdict, field
import matplotlib.pyplot as plt
from datetime import datetime
from tqdm import tqdm


# ====================
# Configuration
# ====================
class OptConfig:
    """Configuration parameters for the optimization process"""
    
    # Base directory setup
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    OUTPUT_BASE = os.path.join(BASE_DIR, "optimization_results")
    TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
    OUTPUT_DIR = os.path.join(OUTPUT_BASE, f"run_{TIMESTAMP}")

    # Base fuselage parameters (nominal design)
    BASE_LENGTH = 20.0  # meters
    BASE_WIDTH = 3.0    # meters  
    BASE_HEIGHT = 3.0   # meters

    # Perturbation parameters (percentage of base values)
    LENGTH_PERTURBATION = 0.1  # ±10% of base length
    WIDTH_PERTURBATION = 0.1   # ±10% of base width
    HEIGHT_PERTURBATION = 0.1  # ±10% of base height

    # Number of configurations to generate
    NUM_CONFIGURATIONS = 30

    # Volume placement parameters
    NUM_VOLUMES = 10
    SAFETY_MARGIN = 0.8  # Safety factor for volume placement inside fuselage
    LONGITUDINAL_MARGIN = 0.03  # Margin at fuselage ends to avoid placement near tips
    COLLISION_MARGIN = 1.1  # Safety margin for collision detection between volumes

    # Surface sampling for volume placement and geometry analysis
    U_SAMPLES = 80  # Number of longitudinal samples along fuselage
    W_SAMPLES = 32  # Number of circumferential samples around fuselage

    # Top N configurations to keep after optimization
    TOP_N = 20

    # Maximum attempts to place each volume before skipping
    MAX_PLACEMENT_ATTEMPTS = 1000


# ====================
# Data Classes
# ====================
@dataclass
class BoxVolume:
    """
    Represents a rectangular volume with position, dimensions, and constraints.
    Used for both volume definition and placement tracking.
    """
    id: int
    length: float
    width: float
    height: float
    distance_constraints: List[Tuple[int, float]] = field(default_factory=list)
    x: float = 0.0  # X coordinate (longitudinal position)
    y: float = 0.0  # Y coordinate (lateral position)  
    z: float = 0.0  # Z coordinate (vertical position)

    half_length: float = 0.0
    half_width: float = 0.0
    half_height: float = 0.0

    def __post_init__(self):
        """Calculate half-dimensions for collision detection efficiency"""
        self.half_length = self.length / 2.0
        self.half_width = self.width / 2.0
        self.half_height = self.height / 2.0

    def to_dict(self):
        """Convert to dictionary for JSON serialization"""
        data = asdict(self)
        data["distance_constraints"] = [
            {"target_volume": t, "min_distance": d} for (t, d) in self.distance_constraints
        ]
        return data


@dataclass
class CrossSection:
    """Represents a cross-sectional slice of the fuselage at a specific U position"""
    u: float  # Parameter along fuselage length (0-1)
    x: float  # Actual X coordinate in meters
    radius_y: float  # Radius in Y direction (half-width)
    radius_z: float  # Radius in Z direction (half-height)
    area: float  # Approximate cross-sectional area
    center_y: float  # Y center of cross-section
    center_z: float  # Z center of cross-section


@dataclass
class FuselageBounds:
    """Complete geometric description of fuselage for volume placement validation"""
    length: float  # Total fuselage length
    x_min: float  # Minimum X coordinate
    x_max: float  # Maximum X coordinate
    cross_sections: List[CrossSection]  # Sampled cross-sections along fuselage
    bbox_min: Tuple[float, float, float]  # Bounding box minimum coordinates
    bbox_max: Tuple[float, float, float]  # Bounding box maximum coordinates
    surf_indx: int  # OpenVSP surface index

    def get_radius_at_u(self, u: float) -> Tuple[float, float, float, float]:
        """
        Get cross-sectional properties at specific U position using interpolation.
        Returns: (radius_y, radius_z, center_y, center_z)
        """
        if not self.cross_sections:
            return 1.0, 1.0, 0.0, 0.0

        u = max(0.0, min(1.0, u))  # Clamp u to valid range

        # Find the cross-section interval containing the requested U
        for i, cs in enumerate(self.cross_sections):
            if cs.u >= u:
                if i == 0:
                    return cs.radius_y, cs.radius_z, cs.center_y, cs.center_z

                # Interpolate between cross-sections
                cs_prev = self.cross_sections[i - 1]
                t = (u - cs_prev.u) / (cs.u - cs_prev.u) if cs.u != cs_prev.u else 0

                radius_y = cs_prev.radius_y + t * (cs.radius_y - cs_prev.radius_y)
                radius_z = cs_prev.radius_z + t * (cs.radius_z - cs_prev.radius_z)
                center_y = cs_prev.center_y + t * (cs.center_y - cs_prev.center_y)
                center_z = cs_prev.center_z + t * (cs.center_z - cs_prev.center_z)

                return radius_y, radius_z, center_y, center_z

        # Fallback to last cross-section if u > 1.0
        last = self.cross_sections[-1]
        return last.radius_y, last.radius_z, last.center_y, last.center_z


@dataclass
class FuselageConfiguration:
    """Complete fuselage configuration with placed volumes and performance metrics"""
    config_id: int  # Unique configuration identifier
    length: float  # Fuselage length
    width: float   # Fuselage width  
    height: float  # Fuselage height
    fuselage_volume: float  # Total fuselage volume from OpenVSP
    volumes_placed: List[BoxVolume]  # Successfully placed volumes
    total_volumes_volume: float  # Sum of all placed volumes
    remaining_volume: float  # Unused volume in fuselage
    num_volumes_skipped: int = 0  # Volumes that couldn't be placed
    vsp_file: str = ""  # Path to saved VSP file
    bounds: Optional[FuselageBounds] = None  # Geometric bounds for validation

    def to_dict(self):
        """Convert configuration to dictionary for JSON serialization"""
        return {
            "config_id": self.config_id,
            "fuselage_params": {
                "length": self.length,
                "width": self.width,
                "height": self.height
            },
            "fuselage_volume": self.fuselage_volume,
            "volumes_placed": [v.to_dict() for v in self.volumes_placed],
            "num_volumes_placed": len(self.volumes_placed),
            "num_volumes_skipped": self.num_volumes_skipped,
            "total_volumes_volume": self.total_volumes_volume,
            "remaining_volume": self.remaining_volume,
            "vsp_file": self.vsp_file
        }


# ====================
# Fuselage Creation Functions
# ====================
def create_perturbed_fuselage(base_length: float, base_width: float, base_height: float,
                              perturbation_factors: Tuple[float, float, float]) -> str:
    """
    Create a fuselage geometry in OpenVSP with perturbed dimensions.
    
    Args:
        base_length: Nominal fuselage length
        base_width: Nominal fuselage width  
        base_height: Nominal fuselage height
        perturbation_factors: Tuple of (length_factor, width_factor, height_factor)
    
    Returns:
        OpenVSP geometry ID of created fuselage
    """
    # Apply perturbations to base dimensions
    length = base_length * perturbation_factors[0]
    width = base_width * perturbation_factors[1]
    height = base_height * perturbation_factors[2]

    # Clear existing model and create new fuselage
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")
    vsp.SetGeomName(fid, "FuselageGeom")

    # Set fuselage length parameter
    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.Update()

    # Configure cross-sections to achieve desired width and height profile
    xsec_surf_id = vsp.GetXSecSurf(fid, 0)
    num_xsecs = vsp.GetNumXSec(xsec_surf_id)

    for i in range(num_xsecs):
        t = i / (num_xsecs - 1) if num_xsecs > 1 else 0.0

        # Set cross-section shape to ellipse
        vsp.ChangeXSecShape(xsec_surf_id, i, vsp.XS_ELLIPSE)
        xsec_id = vsp.GetXSec(xsec_surf_id, i)

        # Apply scaling factors to create tapered fuselage shape
        # Smaller scale at ends, full scale in middle
        if i == 0 or i == num_xsecs - 1:
            # Ends: very small scaling for closure
            scale_factor = 0.05
        elif i == 1 or i == num_xsecs - 2:
            # Transition regions
            scale_factor = 0.3
        elif i == 2 or i == num_xsecs - 3:
            # Near transition regions  
            scale_factor = 0.7
        else:
            # Central region: full scale
            scale_factor = 1.0

        # Set ellipse dimensions with scaling
        vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Width"), width * scale_factor)
        vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Height"), height * scale_factor)

    vsp.Update()
    return fid


def compute_fuselage_volume_compgeom() -> float:
    """
    Compute fuselage volume using OpenVSP's CompGeom analysis.
    
    Returns:
        Fuselage volume in cubic meters, or 0.0 if computation fails
    """
    # Run CompGeom analysis on all geometries
    mesh_id = vsp.ComputeCompGeom(vsp.SET_ALL, False, 0)
    result_id = vsp.FindLatestResultsID("Comp_Geom")

    if not result_id:
        return 0.0

    # Extract volume results from CompGeom analysis
    geom_names = vsp.GetStringResults(result_id, "Comp_Name")
    theo_vols = vsp.GetDoubleResults(result_id, "Theo_Vol")

    # Find and return fuselage volume
    for name, vol in zip(geom_names, theo_vols):
        if "fuselage" in name.lower():
            return vol

    return 0.0


# ====================
# Volume Definition with Distance Constraints
# ====================
def get_manual_volumes() -> List[BoxVolume]:
    """
    Define the 10 volumes with their dimensions and distance constraints.
    
    Returns:
        List of BoxVolume objects with pre-defined constraints between specific volumes
    """
    return [
        BoxVolume(id=1, length=0.40, width=0.40, height=0.40,
                  distance_constraints=[(2, 0.6), (3, 0.8), (4, 0.6)]),
        BoxVolume(id=2, length=0.30, width=0.30, height=0.30,
                  distance_constraints=[(1, 0.6), (3, 0.25), (5, 0.5)]),
        BoxVolume(id=3, length=0.50, width=0.40, height=0.40,
                  distance_constraints=[(1, 0.8), (6, 0.4)]),
        BoxVolume(id=4, length=0.35, width=0.35, height=0.35,
                  distance_constraints=[(1, 0.6), (5, 0.5), (7, 0.35)]),
        BoxVolume(id=5, length=0.45, width=0.40, height=0.30,
                  distance_constraints=[(2, 0.5), (4, 0.5), (8, 0.55)]),
        BoxVolume(id=6, length=0.30, width=0.30, height=0.30,
                  distance_constraints=[(3, 0.4), (7, 0.4), (10, 1.0)]),
        BoxVolume(id=7, length=0.40, width=0.40, height=0.40,
                  distance_constraints=[(4, 0.35), (6, 0.4), (10, 0.38)]),
        BoxVolume(id=8, length=0.30, width=0.30, height=0.30,
                  distance_constraints=[(5, 0.55)]),
        BoxVolume(id=9, length=0.50, width=0.45, height=0.40,
                  distance_constraints=[(3, 1.2)]),
        BoxVolume(id=10, length=0.60, width=0.50, height=0.45,
                  distance_constraints=[(6, 1.0), (7, 0.38)]),
    ]


# ====================
# Geometry Analysis Functions
# ====================
def analyze_fuselage_geometry(fuse_id: str) -> FuselageBounds:
    """
    Analyze fuselage geometry by sampling surface points and extracting cross-sections.
    
    Args:
        fuse_id: OpenVSP geometry ID of the fuselage
        
    Returns:
        FuselageBounds object with complete geometric information
    """
    # Get bounding box of fuselage geometry
    bbox_min_vec = vsp.GetGeomBBoxMin(fuse_id, 0, True)
    bbox_max_vec = vsp.GetGeomBBoxMax(fuse_id, 0, True)

    bbox_min = (bbox_min_vec.x(), bbox_min_vec.y(), bbox_min_vec.z())
    bbox_max = (bbox_max_vec.x(), bbox_max_vec.y(), bbox_max_vec.z())

    length = bbox_max[0] - bbox_min[0]

    # Sample cross-sections along fuselage length
    cross_sections = []
    surf_indx = 0  # Main surface index

    for i in range(OptConfig.U_SAMPLES):
        u = i / (OptConfig.U_SAMPLES - 1) if OptConfig.U_SAMPLES > 1 else 0.5

        points_y = []
        points_z = []
        points_x = []

        # Sample points around circumference at this U position
        for j in range(OptConfig.W_SAMPLES):
            w = j / OptConfig.W_SAMPLES

            try:
                point = vsp.CompPnt01(fuse_id, surf_indx, u, w)
                points_x.append(point.x())
                points_y.append(point.y())
                points_z.append(point.z())
            except:
                continue

        if len(points_y) < 4:
            continue

        # Calculate cross-section properties from sampled points
        center_y = sum(points_y) / len(points_y)
        center_z = sum(points_z) / len(points_z)
        avg_x = sum(points_x) / len(points_x)

        # Calculate radii as maximum distance from center
        radius_y = max([abs(y - center_y) for y in points_y]) if points_y else 0.5
        radius_z = max([abs(z - center_z) for z in points_z]) if points_z else 0.5

        # Approximate cross-sectional area as ellipse
        area = math.pi * radius_y * radius_z

        cross_sections.append(CrossSection(
            u=u, x=avg_x, radius_y=radius_y, radius_z=radius_z,
            area=area, center_y=center_y, center_z=center_z
        ))

    # Calculate actual fuselage length from cross-sections
    x_min_real = min(cs.x for cs in cross_sections) if cross_sections else bbox_min[0]
    x_max_real = max(cs.x for cs in cross_sections) if cross_sections else bbox_max[0]

    return FuselageBounds(
        length=x_max_real - x_min_real,
        x_min=x_min_real,
        x_max=x_max_real,
        cross_sections=cross_sections,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        surf_indx=surf_indx
    )


def x_to_u(x: float, bounds: FuselageBounds) -> float:
    """
    Convert X coordinate to normalized U parameter (0-1) along fuselage length.
    
    Args:
        x: X coordinate in meters
        bounds: Fuselage bounds information
        
    Returns:
        U parameter between 0 and 1
    """
    if bounds.length == 0:
        return 0.5
    return (x - bounds.x_min) / bounds.length


# ====================
# PRECISE Collision Detection and Constraint Validation
# ====================
def generate_box_surface_points(volume: BoxVolume, points_per_edge: int = 3) -> List[Tuple[float, float, float]]:
    """
    Generate multiple test points on the surface of a box volume.
    Includes vertices, edge centers, and face centers for comprehensive coverage.
    
    Args:
        volume: Box volume to generate points for
        points_per_edge: Number of points to generate along each edge
        
    Returns:
        List of (x, y, z) coordinates for test points
    """
    points = []
    
    # Generate coordinates along each axis
    x_coords = np.linspace(-volume.half_length, volume.half_length, points_per_edge)
    y_coords = np.linspace(-volume.half_width, volume.half_width, points_per_edge)
    z_coords = np.linspace(-volume.half_height, volume.half_height, points_per_edge)
    
    # Add all 8 vertices of the box
    for dx in [-volume.half_length, volume.half_length]:
        for dy in [-volume.half_width, volume.half_width]:
            for dz in [-volume.half_height, volume.half_height]:
                points.append((
                    volume.x + dx,
                    volume.y + dy,
                    volume.z + dz
                ))
    
    # Add 12 edge midpoints
    # X-axis edges (constant X, varying Y and Z)
    for dy in [-volume.half_width, volume.half_width]:
        for dz in [-volume.half_height, volume.half_height]:
            points.append((volume.x, volume.y + dy, volume.z + dz))
    
    # Y-axis edges (constant Y, varying X and Z)
    for dx in [-volume.half_length, volume.half_length]:
        for dz in [-volume.half_height, volume.half_height]:
            points.append((volume.x + dx, volume.y, volume.z + dz))
    
    # Z-axis edges (constant Z, varying X and Y)
    for dx in [-volume.half_length, volume.half_length]:
        for dy in [-volume.half_width, volume.half_width]:
            points.append((volume.x + dx, volume.y + dy, volume.z))
    
    # Add additional points along edges for better coverage (excluding endpoints)
    if points_per_edge > 2:
        for axis in ['x', 'y', 'z']:
            if axis == 'x':
                for dy in y_coords[1:-1]:
                    for dz in z_coords[1:-1]:
                        points.append((volume.x, volume.y + dy, volume.z + dz))
            elif axis == 'y':
                for dx in x_coords[1:-1]:
                    for dz in z_coords[1:-1]:
                        points.append((volume.x + dx, volume.y, volume.z + dz))
            else:  # z-axis
                for dx in x_coords[1:-1]:
                    for dy in y_coords[1:-1]:
                        points.append((volume.x + dx, volume.y + dy, volume.z))
    
    return points


def find_surface_distance_at_angle(cross_section: CrossSection, angle: float, surf_indx: int) -> Optional[float]:
    """
    Calculate distance from fuselage center to surface at a specific angle.
    Uses elliptical approximation based on cross-section radii.
    
    Args:
        cross_section: Cross-sectional data
        angle: Angle in radians from positive Y axis
        surf_indx: Surface index (unused but kept for interface consistency)
        
    Returns:
        Distance from center to surface, or None if calculation fails
    """
    # For an ellipse, the distance from center to surface at angle theta is:
    # r = (a * b) / sqrt((b*cos(theta))^2 + (a*sin(theta))^2)
    # where a = radius_y, b = radius_z
    
    a = cross_section.radius_y
    b = cross_section.radius_z
    
    if a <= 0 or b <= 0:
        return None
    
    # Calculate the elliptical radius at this angle
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    
    denominator = math.sqrt((b * cos_theta)**2 + (a * sin_theta)**2)
    if denominator == 0:
        return min(a, b)
    
    surface_dist = (a * b) / denominator
    
    return surface_dist


def is_point_inside_fuselage(x: float, y: float, z: float, 
                            bounds: FuselageBounds, 
                            safety_factor: float = OptConfig.SAFETY_MARGIN,
                            debug: bool = False) -> bool:
    """
    Check if a point is inside the fuselage by comparing to surface distance.
    
    Args:
        x, y, z: Point coordinates to check
        bounds: Fuselage geometry information
        safety_factor: Multiplier for surface distance to provide safety margin
        debug: Enable debug output
        
    Returns:
        True if point is inside fuselage, False otherwise
    """
    # Convert point X to U parameter along fuselage
    u = x_to_u(x, bounds)
    
    # Find the closest cross-section to this U position
    target_cs = None
    min_u_diff = float('inf')
    
    for cs in bounds.cross_sections:
        u_diff = abs(cs.u - u)
        if u_diff < min_u_diff:
            min_u_diff = u_diff
            target_cs = cs
    
    if target_cs is None:
        return False
    
    # Calculate vector from fuselage center to point
    dy = y - target_cs.center_y
    dz = z - target_cs.center_z
    
    # Calculate angle of point relative to fuselage center
    angle = math.atan2(dz, dy) if dy != 0 or dz != 0 else 0
    
    # Find distance to fuselage surface at this angle
    surface_dist = find_surface_distance_at_angle(target_cs, angle, bounds.surf_indx)
    if surface_dist is None:
        return False
    
    # Calculate actual distance from center to point
    point_dist = math.sqrt(dy*dy + dz*dz)
    
    # Apply safety margin to surface distance
    safe_surface_dist = surface_dist * safety_factor
    
    # Point is inside if its distance is less than the safe surface distance
    return point_dist <= safe_surface_dist


def is_box_inside_fuselage(volume: BoxVolume,
                           bounds: FuselageBounds,
                           safety_factor: float = OptConfig.SAFETY_MARGIN,
                           debug: bool = False) -> bool:
    """
    Check if a box volume is fully contained within the fuselage.
    Verifies multiple surface points against actual fuselage geometry.
    
    Args:
        volume: Box volume to check
        bounds: Fuselage geometry information  
        safety_factor: Safety margin for placement
        debug: Enable debug output
        
    Returns:
        True if all box points are inside fuselage, False otherwise
    """
    # Quick rejection: check longitudinal bounds first
    if (volume.x - volume.half_length < bounds.x_min + bounds.length * OptConfig.LONGITUDINAL_MARGIN or
            volume.x + volume.half_length > bounds.x_max - bounds.length * OptConfig.LONGITUDINAL_MARGIN):
        if debug:
            print(f"    DEBUG: Volume {volume.id} failed longitudinal bounds check")
        return False

    # Generate multiple test points on the box surface
    test_points = generate_box_surface_points(volume, points_per_edge=3)
    
    # Check each test point against fuselage surface
    for i, (vx, vy, vz) in enumerate(test_points):
        if not is_point_inside_fuselage(vx, vy, vz, bounds, safety_factor, debug):
            if debug:
                print(f"    DEBUG: Volume {volume.id} test point {i} outside fuselage")
                print(f"           Point ({vx:.3f}, {vy:.3f}, {vz:.3f})")
            return False

    return True


def boxes_collide(v1: BoxVolume, v2: BoxVolume, margin: float = OptConfig.COLLISION_MARGIN) -> bool:
    """
    Check if two box volumes collide using axis-aligned bounding box detection.
    
    Args:
        v1: First box volume
        v2: Second box volume  
        margin: Safety margin multiplier for collision detection
        
    Returns:
        True if boxes collide, False otherwise
    """
    # Check for separation along each axis with safety margin
    x_overlap = (abs(v1.x - v2.x) < (v1.half_length + v2.half_length) * margin)
    y_overlap = (abs(v1.y - v2.y) < (v1.half_width + v2.half_width) * margin)
    z_overlap = (abs(v1.z - v2.z) < (v1.half_height + v2.half_height) * margin)

    # Collision occurs if there's overlap in all three axes
    return x_overlap and y_overlap and z_overlap


def respects_distance_constraints(vol: BoxVolume, placed_volumes: List[BoxVolume], debug: bool = False) -> bool:
    """
    Check if a volume respects all its distance constraints with placed volumes.
    
    Args:
        vol: Volume to check constraints for
        placed_volumes: List of already placed volumes
        debug: Enable debug output
        
    Returns:
        True if all constraints are satisfied, False otherwise
    """
    if not vol.distance_constraints:
        return True

    # Build dictionary for quick volume lookup by ID
    lookup = {v.id: v for v in placed_volumes}
    
    # Check each distance constraint
    for target_id, min_dist in vol.distance_constraints:
        target = lookup.get(target_id)
        if target is None:
            # If target not placed yet, constraint cannot be checked
            continue

        # Skip self-constraint (shouldn't exist but check anyway)
        if target_id == vol.id:
            continue

        # Calculate Euclidean distance between volumes
        dx = vol.x - target.x
        dy = vol.y - target.y
        dz = vol.z - target.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        
        # Check if distance meets minimum requirement
        if dist < min_dist:
            return False

    return True


def is_valid_placement(new_volume: BoxVolume,
                       placed_volumes: List[BoxVolume],
                       bounds: FuselageBounds,
                       debug: bool = False) -> bool:
    """
    Comprehensive validation for volume placement.
    Combines fuselage containment, collision detection, and distance constraints.
    
    Args:
        new_volume: Volume to validate
        placed_volumes: List of already placed volumes
        bounds: Fuselage geometry information
        debug: Enable debug output
        
    Returns:
        True if placement is valid, False otherwise
    """
    # Check if volume is completely inside fuselage
    if not is_box_inside_fuselage(new_volume, bounds, OptConfig.SAFETY_MARGIN, debug):
        return False

    # Check for collisions with already placed volumes
    for pv in placed_volumes:
        if boxes_collide(new_volume, pv):
            return False

    # Check distance constraints with placed volumes
    if not respects_distance_constraints(new_volume, placed_volumes, debug):
        return False

    return True


# ====================
# Volume Placement Functions
# ====================
def place_volumes_in_fuselage(fuse_id: str, bounds: FuselageBounds,
                              base_volumes: List[BoxVolume],
                              debug: bool = False) -> Tuple[List[BoxVolume], int]:
    """
    Attempt to place all volumes in the fuselage with random placement and validation.
    
    Args:
        fuse_id: OpenVSP fuselage geometry ID
        bounds: Fuselage geometry information
        base_volumes: List of volumes to place
        debug: Enable debug output
        
    Returns:
        Tuple of (placed_volumes, num_skipped) where:
        - placed_volumes: List of successfully placed volumes with positions
        - num_skipped: Number of volumes that couldn't be placed
    """
    placed = []
    skipped = 0

    # Sort volumes by size (largest first) for better placement success
    sorted_volumes = sorted(base_volumes,
                           key=lambda v: v.length * v.width * v.height,
                           reverse=True)

    # Attempt to place each volume
    for base_vol in sorted_volumes:
        placed_successfully = False
        attempts = 0

        # Try multiple random positions until successful or max attempts reached
        while attempts < OptConfig.MAX_PLACEMENT_ATTEMPTS and not placed_successfully:
            # Generate random position within fuselage
            u = np.random.random()
            ry, rz, cy, cz = bounds.get_radius_at_u(u)
            x = bounds.x_min + u * bounds.length

            # Calculate valid Y and Z ranges within cross-section with safety margin
            max_y = cy + ry * OptConfig.SAFETY_MARGIN - base_vol.half_width
            min_y = cy - ry * OptConfig.SAFETY_MARGIN + base_vol.half_width
            max_z = cz + rz * OptConfig.SAFETY_MARGIN - base_vol.half_height
            min_z = cz - rz * OptConfig.SAFETY_MARGIN + base_vol.half_height

            # Check if valid position range exists
            if max_y > min_y and max_z > min_z:
                # Generate random position within valid range
                y = np.random.uniform(min_y, max_y)
                z = np.random.uniform(min_z, max_z)

                # Create volume instance with position
                vol = BoxVolume(
                    id=base_vol.id,
                    x=x, y=y, z=z,
                    length=base_vol.length,
                    width=base_vol.width,
                    height=base_vol.height,
                    distance_constraints=base_vol.distance_constraints
                )

                # Validate placement against all constraints
                if is_valid_placement(vol, placed, bounds, debug):
                    placed.append(vol)
                    placed_successfully = True

            attempts += 1

        # Track volumes that couldn't be placed
        if not placed_successfully:
            skipped += 1

    return placed, skipped


def create_volumes_in_vsp(volumes: List[BoxVolume]):
    """
    Create physical box geometries in OpenVSP from placed volume data.
    
    Args:
        volumes: List of placed volumes with positions and dimensions
    """
    for vol in volumes:
        # Create box geometry in OpenVSP
        box_id = vsp.AddGeom("Box", "")
        geom_name = f"Volume_{vol.id}"
        vsp.SetGeomName(box_id, geom_name)

        # Set transformation parameters
        vsp.SetParmValUpdate(box_id, "Abs_Or_Relitive_flag", "XForm", 0)
        vsp.SetParmValUpdate(box_id, "Trans_Attach_Flag", "Attach", 0)
        vsp.SetParmValUpdate(box_id, "X_Location", "XForm", vol.x)
        vsp.SetParmValUpdate(box_id, "Y_Location", "XForm", vol.y)
        vsp.SetParmValUpdate(box_id, "Z_Location", "XForm", vol.z)

        try:
            # Set box dimensions
            vsp.SetParmValUpdate(box_id, "Length", "Design", vol.length)
            vsp.SetParmValUpdate(box_id, "Width", "Design", vol.width)
            vsp.SetParmValUpdate(box_id, "Height", "Design", vol.height)
        except Exception as e:
            print(f"Warning: could not set dimensions for Volume {vol.id}: {e}")

    vsp.Update()


# ====================
# Validation Functions
# ====================
def validate_configuration(volumes: List[BoxVolume], bounds: FuselageBounds) -> Tuple[bool, List[str]]:
    """
    Comprehensive validation of a complete volume configuration.
    
    Args:
        volumes: List of placed volumes to validate
        bounds: Fuselage geometry information
        
    Returns:
        Tuple of (is_valid, error_messages) where:
        - is_valid: True if configuration passes all checks
        - error_messages: List of validation error descriptions
    """
    errors = []
    
    # Validate each volume individually
    for i, vol in enumerate(volumes):
        # Check if volume is inside fuselage
        if not is_box_inside_fuselage(vol, bounds, OptConfig.SAFETY_MARGIN):
            errors.append(f"Volume {vol.id} is outside fuselage bounds")
        
        # Check for collisions with other volumes
        for j, other_vol in enumerate(volumes):
            if i != j and boxes_collide(vol, other_vol):
                errors.append(f"Volume {vol.id} collides with Volume {other_vol.id}")
        
        # Check distance constraints
        if not respects_distance_constraints(vol, volumes):
            errors.append(f"Volume {vol.id} violates distance constraints")
    
    return len(errors) == 0, errors


# ====================
# Main Optimization Functions
# ====================
def generate_perturbation_factors(config: type) -> List[Tuple[float, float, float]]:
    """
    Generate random perturbation factors for fuselage dimensions.
    
    Args:
        config: Configuration class with perturbation parameters
        
    Returns:
        List of (length_factor, width_factor, height_factor) tuples
    """
    # Note: Seed commented out for maximum exploration diversity
    # np.random.seed(42)  # For reproducibility

    factors = []
    for i in range(config.NUM_CONFIGURATIONS):
        # Generate random factors within perturbation range
        length_factor = 1.0 + np.random.uniform(-config.LENGTH_PERTURBATION,
                                                 config.LENGTH_PERTURBATION)
        width_factor = 1.0 + np.random.uniform(-config.WIDTH_PERTURBATION,
                                                config.WIDTH_PERTURBATION)
        height_factor = 1.0 + np.random.uniform(-config.HEIGHT_PERTURBATION,
                                                 config.HEIGHT_PERTURBATION)
        factors.append((length_factor, width_factor, height_factor))

    return factors


def run_optimization():
    """Main optimization loop that generates and evaluates fuselage configurations."""
    print("=" * 70)
    print("FUSELAGE VOLUME OPTIMIZATION - WITH COLLISION & CONSTRAINTS")
    print("=" * 70)

    # Create output directory structure
    os.makedirs(OptConfig.OUTPUT_DIR, exist_ok=True)

    # Generate perturbation factors for fuselage dimensions
    print(f"\n1. Generating {OptConfig.NUM_CONFIGURATIONS} perturbation factors...")
    perturbation_factors = generate_perturbation_factors(OptConfig)

    # Load volume definitions with constraints
    base_volumes = get_manual_volumes()
    print(f"   Base volumes defined: {len(base_volumes)}")
    print(f"   Total distance constraints: {sum(len(v.distance_constraints) for v in base_volumes)}")

    # Store all valid configurations
    all_configs = []

    print(f"\n2. Generating and evaluating configurations...")
    valid_configs = 0

    # Process each configuration with progress bar
    for i, (lf, wf, hf) in enumerate(tqdm(perturbation_factors, 
                                         desc="Configurations", 
                                         unit="config",
                                         ncols=100)):
        debug_mode = False  # Disable detailed debug output for performance

        # Create fuselage with perturbed dimensions
        fuse_id = create_perturbed_fuselage(
            OptConfig.BASE_LENGTH, OptConfig.BASE_WIDTH, OptConfig.BASE_HEIGHT,
            (lf, wf, hf)
        )

        # Analyze fuselage geometry for volume placement
        bounds = analyze_fuselage_geometry(fuse_id)

        # Compute fuselage volume using OpenVSP
        fuse_volume = compute_fuselage_volume_compgeom()

        # Attempt to place all volumes with constraints
        placed_volumes, num_skipped = place_volumes_in_fuselage(
            fuse_id, bounds, base_volumes, debug=debug_mode
        )

        # Only consider configurations where ALL volumes are successfully placed
        if num_skipped == 0:
            # Final validation to ensure configuration integrity
            is_valid, validation_errors = validate_configuration(placed_volumes, bounds)
            
            if is_valid:
                # Calculate volume metrics
                total_boxes_volume = sum([v.length * v.width * v.height for v in placed_volumes])

                # Create configuration object
                config = FuselageConfiguration(
                    config_id=i+1,
                    length=OptConfig.BASE_LENGTH * lf,
                    width=OptConfig.BASE_WIDTH * wf,
                    height=OptConfig.BASE_HEIGHT * hf,
                    fuselage_volume=fuse_volume,
                    volumes_placed=placed_volumes,
                    total_volumes_volume=total_boxes_volume,
                    remaining_volume=fuse_volume - total_boxes_volume,
                    num_volumes_skipped=num_skipped,
                    bounds=bounds  # Store bounds for saving phase
                )

                all_configs.append(config)
                valid_configs += 1
            else:
                # Log validation failures for debugging
                print(f"\n   Config {i+1} failed validation: {validation_errors}")

    print(f"\n3. Valid configurations found: {valid_configs}/{OptConfig.NUM_CONFIGURATIONS}")

    if valid_configs == 0:
        print("   No valid configurations found! Exiting.")
        return

    print(f"   Ranking configurations by fuselage volume...")
    # Sort configurations by fuselage volume (ascending) to find minimum volume designs
    all_configs.sort(key=lambda c: c.fuselage_volume)

    # Select top N configurations
    top_n = min(OptConfig.TOP_N, len(all_configs))
    top_configs = all_configs[:top_n]

    print(f"\n4. Saving top {top_n} configurations...")
    top_dir = os.path.join(OptConfig.OUTPUT_DIR, "top_configurations")
    os.makedirs(top_dir, exist_ok=True)

    # Save each top configuration with progress tracking
    for rank, config in enumerate(tqdm(top_configs, desc="Saving configs", unit="file", ncols=100), 1):
        conf_dir = os.path.join(top_dir, f"rank_{rank:02d}_config_{config.config_id}")
        os.makedirs(conf_dir, exist_ok=True)

        # Recreate fuselage geometry for saving (using validated positions)
        fuse_id = create_perturbed_fuselage(
            OptConfig.BASE_LENGTH, OptConfig.BASE_WIDTH, OptConfig.BASE_HEIGHT,
            (config.length/OptConfig.BASE_LENGTH,
             config.width/OptConfig.BASE_WIDTH,
             config.height/OptConfig.BASE_HEIGHT)
        )

        # Create volumes using previously validated positions
        create_volumes_in_vsp(config.volumes_placed)

        vsp.Update()

        # Final validation before saving to ensure data consistency
        current_bounds = analyze_fuselage_geometry(fuse_id)
        is_valid, validation_errors = validate_configuration(config.volumes_placed, current_bounds)
        
        if not is_valid:
            print(f"\n   WARNING: Configuration {config.config_id} failed final validation!")
            print(f"   Errors: {validation_errors}")
            continue  # Skip saving invalid configurations

        # Save VSP file with complete geometry
        vsp_file = os.path.join(conf_dir, f"fuselage_rank_{rank:02d}.vsp3")
        vsp.WriteVSPFile(vsp_file, vsp.SET_ALL)
        config.vsp_file = vsp_file

        # Save configuration metadata as JSON
        json_file = os.path.join(conf_dir, "config_data.json")
        with open(json_file, "w") as f:
            json.dump(config.to_dict(), f, indent=2)

        # Save volume positions as CSV
        save_volumes_positions_csv(config.volumes_placed, conf_dir)

        # Save constraint verification results as CSV
        save_constraints_csv(config.volumes_placed, base_volumes, conf_dir)

    # Save optimization summary
    print(f"\n5. Saving summary CSV...")
    summary_csv = os.path.join(OptConfig.OUTPUT_DIR, "optimization_summary.csv")
    with open(summary_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Rank", "Config_ID", "Length", "Width", "Height",
                        "Fuselage_Volume", "Boxes_Volume", "Remaining_Volume",
                        "Num_Volumes_Placed", "Num_Volumes_Skipped"])

        for rank, config in enumerate(top_configs, 1):
            writer.writerow([
                rank, config.config_id,
                f"{config.length:.3f}", f"{config.width:.3f}", f"{config.height:.3f}",
                f"{config.fuselage_volume:.3f}", f"{config.total_volumes_volume:.3f}",
                f"{config.remaining_volume:.3f}",
                len(config.volumes_placed), config.num_volumes_skipped
            ])

    # Generate analysis plots
    print(f"\n6. Generating visualizations...")
    generate_plots(all_configs, top_configs)

    # Print completion message
    print("\n" + "=" * 70)
    print("OPTIMIZATION COMPLETE!")
    print(f"Results saved to: {OptConfig.OUTPUT_DIR}")
    print(f"Top {top_n} configurations in: {top_dir}")
    print("=" * 70)

    # Display top 5 configuration summary
    print("\nTop 5 configurations by fuselage volume:")
    print("-" * 70)
    for i, config in enumerate(top_configs[:5], 1):
        print(f"{i}. Config {config.config_id}: Volume = {config.fuselage_volume:.3f} m³")
        print(f"   L={config.length:.2f}, W={config.width:.2f}, H={config.height:.2f}")
        print(f"   Volumes: {len(config.volumes_placed)} placed, {config.num_volumes_skipped} skipped")


def save_volumes_positions_csv(placed_volumes: List[BoxVolume], conf_dir: str):
    """
    Save volume positions and dimensions to CSV file.
    
    Args:
        placed_volumes: List of placed volumes with positions
        conf_dir: Directory to save CSV file in
    """
    csv_file = os.path.join(conf_dir, "volumes_positions.csv")

    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Volume_ID", "X", "Y", "Z", "Length", "Width", "Height"])

        for vol in placed_volumes:
            writer.writerow([
                vol.id,
                f"{vol.x:.4f}",
                f"{vol.y:.4f}",
                f"{vol.z:.4f}",
                f"{vol.length:.4f}",
                f"{vol.width:.4f}",
                f"{vol.height:.4f}"
            ])


def save_constraints_csv(placed_volumes: List[BoxVolume],
                         all_volumes: List[BoxVolume],
                         conf_dir: str):
    """
    Save distance constraint verification results to CSV file.
    
    Args:
        placed_volumes: List of successfully placed volumes
        all_volumes: All volume definitions including constraints
        conf_dir: Directory to save CSV file in
    """
    csv_file = os.path.join(conf_dir, "distance_constraints.csv")

    placed_ids = {vol.id for vol in placed_volumes}
    placed_lookup = {vol.id: vol for vol in placed_volumes}
    all_lookup = {vol.id: vol for vol in all_volumes}

    # Collect all unique constraints (remove duplicates for bidirectional constraints)
    unique_constraints = {}
    for volume in all_volumes:
        for target_id, min_distance in volume.distance_constraints:
            pair_key = tuple(sorted([volume.id, target_id]))
            if pair_key in unique_constraints:
                # Keep the more restrictive (smaller) distance constraint
                existing_min_dist = unique_constraints[pair_key][2]
                unique_constraints[pair_key] = (
                    pair_key[0], pair_key[1], min(existing_min_dist, min_distance)
                )
            else:
                unique_constraints[pair_key] = (pair_key[0], pair_key[1], min_distance)

    constraints_list = list(unique_constraints.values())
    constraints_list.sort(key=lambda x: (x[0], x[1]))

    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Volume_1", "Volume_2",
            "Min_Distance_m", "Actual_Distance_m",
            "Satisfied", "Both_Placed"
        ])

        for vol1_id, vol2_id, min_dist in constraints_list:
            both_placed = "YES" if (vol1_id in placed_ids and vol2_id in placed_ids) else "NO"

            if both_placed == "YES":
                # Calculate actual distance between placed volumes
                vol1 = placed_lookup[vol1_id]
                vol2 = placed_lookup[vol2_id]
                dx = vol1.x - vol2.x
                dy = vol1.y - vol2.y
                dz = vol1.z - vol2.z
                actual_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                satisfied = "YES" if actual_dist >= min_dist else "NO"
            else:
                actual_dist = "N/A"
                satisfied = "N/A"

            writer.writerow([
                vol1_id, vol2_id,
                f"{min_dist:.3f}",
                f"{actual_dist:.3f}" if actual_dist != "N/A" else actual_dist,
                satisfied, both_placed
            ])


def generate_plots(all_configs: List[FuselageConfiguration],
                   top_configs: List[FuselageConfiguration]):
    """
    Generate analysis plots for optimization results.
    
    Args:
        all_configs: All valid configurations found
        top_configs: Top N configurations selected
    """
    # Extract data for plotting
    all_volumes = [c.fuselage_volume for c in all_configs]
    top_volumes = [c.fuselage_volume for c in top_configs]
    all_boxes = [c.total_volumes_volume for c in all_configs]
    top_boxes = [c.total_volumes_volume for c in top_configs]
    all_placed = [len(c.volumes_placed) for c in all_configs]
    top_placed = [len(c.volumes_placed) for c in top_configs]

    # Create figure with 2x2 subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 1. Distribution of all fuselage volumes
    ax1 = axes[0, 0]
    ax1.hist(all_volumes, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
    if top_volumes:
        ax1.axvline(np.mean(top_volumes), color='red', linestyle='--',
                    label=f'Top {len(top_configs)} mean')
    ax1.set_xlabel('Fuselage Volume (m³)')
    ax1.set_ylabel('Count')
    ax1.set_title('Distribution of Fuselage Volumes (Valid Configs Only)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Volumes placed distribution
    ax2 = axes[0, 1]
    ax2.hist(all_placed, bins=range(0, OptConfig.NUM_VOLUMES+2),
             alpha=0.7, color='lightgreen', edgecolor='black')
    if top_placed:
        ax2.axvline(np.mean(top_placed), color='red', linestyle='--',
                    label=f'Top {len(top_configs)} mean')
    ax2.set_xlabel('Number of Volumes Placed')
    ax2.set_ylabel('Count')
    ax2.set_title('Distribution of Successfully Placed Volumes')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Scatter: Fuselage volume vs Boxes volume
    ax3 = axes[1, 0]
    ax3.scatter(all_volumes, all_boxes, alpha=0.5, s=30, label='All valid configs')
    if top_volumes and top_boxes:
        ax3.scatter(top_volumes, top_boxes, color='red', s=50,
                    label=f'Top {len(top_configs)}', edgecolor='black', linewidth=1)
    ax3.set_xlabel('Fuselage Volume (m³)')
    ax3.set_ylabel('Total Boxes Volume (m³)')
    ax3.set_title('Fuselage Volume vs Placed Boxes Volume')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Scatter: Fuselage volume vs Number of volumes placed
    ax4 = axes[1, 1]
    ax4.scatter(all_volumes, all_placed, alpha=0.5, s=30, label='All valid configs')
    if top_volumes and top_placed:
        ax4.scatter(top_volumes, top_placed, color='red', s=50,
                    label=f'Top {len(top_configs)}', edgecolor='black', linewidth=1)
    ax4.set_xlabel('Fuselage Volume (m³)')
    ax4.set_ylabel('Volumes Successfully Placed')
    ax4.set_title('Fuselage Volume vs Placement Success')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save plot to file
    plot_file = os.path.join(OptConfig.OUTPUT_DIR, "optimization_analysis.png")
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"   Saved plot: {plot_file}")

    plt.show()


# ====================
# Entry Point
# ====================
if __name__ == "__main__":
    try:
        run_optimization()
    except Exception as e:
        print(f"\n[ERROR] Optimization failed: {e}")
        import traceback
        traceback.print_exc()
