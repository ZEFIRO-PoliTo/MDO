"""
Fuselage Volume Optimization Script with Aerodynamic Analysis (STEP-BASED APPROACH)
=================================================================================
This script generates multiple fuselage configurations using step-based parameters,
evaluates their aerodynamic performance (Cd) first, then selects
the best performing configurations for volume analysis.

Key Features:
- Step-based geometric variation (replaces perturbation factors)
- Precise geometric validation using multiple surface points
- Multi-level validation to ensure consistency
- Distance constraints between volumes
- Collision detection with safety margins
- Automated VSP file generation and analysis
- Aerodynamic parasite drag analysis
- Combined CD and volume optimization
"""
import openvsp as vsp
import numpy as np
import os
import json
import csv
import math
import sys
import io
from typing import List, Tuple, Optional
from dataclasses import dataclass, asdict, field
import matplotlib.pyplot as plt
from datetime import datetime
from tqdm import tqdm

# Fix encoding for Windows (tqdm character compatibility)
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')


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

    # Length parameters (STEP-BASED)
    LENGTH_MIN = 18.0  # meters
    LENGTH_MAX = 22.0  # meters
    LENGTH_STEP = 1.0  # meters

    # Width parameters (STEP-BASED)
    WIDTH_MIN = 2.7  # meters (minimum radius * 2)
    WIDTH_MAX = 3.3  # meters (maximum radius * 2)
    WIDTH_STEP = 0.2  # meters

    # Height parameters (STEP-BASED)
    HEIGHT_MIN = 2.7  # meters (minimum radius * 2)
    HEIGHT_MAX = 3.3  # meters (maximum radius * 2)
    HEIGHT_STEP = 0.2  # meters

    # Volume placement parameters
    NUM_VOLUMES = 10
    SAFETY_MARGIN = 0.8  # Safety factor for volume placement inside fuselage
    LONGITUDINAL_MARGIN = 0.03  # Margin at fuselage ends to avoid placement near tips
    COLLISION_MARGIN = 1.1  # Safety margin for collision detection between volumes

    # Surface sampling for volume placement and geometry analysis
    U_SAMPLES = 80  # Number of longitudinal samples along fuselage
    W_SAMPLES = 32  # Number of circumferential samples around fuselage

    # Top N configurations to keep after CD optimization
    TOP_N_CD = 10  # Select top by CD for volume analysis
    TOP_N_FINAL = 5  # Final top after combined optimization

    # Maximum attempts to place each volume before skipping
    MAX_PLACEMENT_ATTEMPTS = 1000

    # Aerodynamic analysis parameters
    CD_THRESHOLD = 0.06  # Maximum acceptable drag coefficient
    ANALYSIS_SREF = 100.0  # m²
    ANALYSIS_VINF = 150.0  # m/s
    ANALYSIS_ALTITUDE = 6096.0  # m
    ANALYSIS_DELTA_TEMP = 0.0  # K

    # Combined scoring weights
    CD_WEIGHT = 0.7  # Primary weight for drag coefficient
    VOLUME_WEIGHT = 0.3  # Secondary weight for fuselage volume

    VOLUME_FILE: Optional[str] = None

    # Conversion factor: meters to feet
    METERS_TO_FEET = 3.28084

    @classmethod
    def update_from_gui(cls, config_dict):
        """Update configuration from GUI parameters"""
        cls.LENGTH_MIN = config_dict.get('l_min', cls.LENGTH_MIN)
        cls.LENGTH_MAX = config_dict.get('l_max', cls.LENGTH_MAX)
        cls.LENGTH_STEP = config_dict.get('l_step', cls.LENGTH_STEP)
        cls.TOP_N_CD = int(config_dict.get('top_n_cd', cls.TOP_N_CD))
        cls.TOP_N_FINAL = int(config_dict.get('top_n_final', cls.TOP_N_FINAL))

        # Width and Height are based on radius
        r_min = config_dict.get('r_min', 1.35)
        r_max = config_dict.get('r_max', 1.65)
        r_step = config_dict.get('r_step', 0.15)

        cls.WIDTH_MIN = r_min * 2
        cls.WIDTH_MAX = r_max * 2
        cls.WIDTH_STEP = r_step * 2

        cls.HEIGHT_MIN = r_min * 2
        cls.HEIGHT_MAX = r_max * 2
        cls.HEIGHT_STEP = r_step * 2

        cls.SAFETY_MARGIN = config_dict.get('safety_margin', cls.SAFETY_MARGIN)
        cls.CD_WEIGHT = config_dict.get('cd_weight', cls.CD_WEIGHT)
        cls.VOLUME_WEIGHT = config_dict.get('volume_weight', cls.VOLUME_WEIGHT)
        cls.ANALYSIS_SREF = config_dict.get('sref', cls.ANALYSIS_SREF)
        cls.ANALYSIS_VINF = config_dict.get('vinf', cls.ANALYSIS_VINF)
        cls.ANALYSIS_ALTITUDE = config_dict.get('altitude', cls.ANALYSIS_ALTITUDE)
        cls.ANALYSIS_DELTA_TEMP = config_dict.get('delta_temp', cls.ANALYSIS_DELTA_TEMP)
        cls.VOLUME_FILE = config_dict.get('volume_file', cls.VOLUME_FILE)

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
    config_id: int
    length: float
    width: float
    height: float
    fuselage_volume: float = 0.0
    volumes_placed: List[BoxVolume] = field(default_factory=list)
    total_volumes_volume: float = 0.0
    remaining_volume: float = 0.0
    num_volumes_skipped: int = 0
    vsp_file: str = ""
    vsp_file_empty: str = ""
    bounds: Optional[FuselageBounds] = None
    cd_value: float = 0.0
    combined_score: float = 0.0

    def to_dict(self):
        """Convert configuration to dictionary for JSON serialization"""
        data = {
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
            "cd_value": self.cd_value,
            "combined_score": self.combined_score,
            "vsp_file": self.vsp_file,
            "vsp_file_empty": self.vsp_file_empty
        }
        return data


# ====================
# Fuselage Creation Functions
# ====================
def create_fuselage_from_dimensions(length: float, width: float, height: float) -> str:
    """
    Create a fuselage geometry in OpenVSP with specified dimensions and closed nose/tail.
    Args:
        length: Fuselage length in meters
        width: Fuselage width in meters
        height: Fuselage height in meters
    Returns:
        OpenVSP geometry ID of created fuselage
    """
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
        # CRITICAL: First and last cross-sections MUST be POINT type for proper closure
        if i == 0 or i == num_xsecs - 1:
            # Set to POINT type for nose/tail closure
            vsp.ChangeXSecShape(xsec_surf_id, i, vsp.XS_POINT)
        else:
            # Set intermediate cross-sections to ellipse
            vsp.ChangeXSecShape(xsec_surf_id, i, vsp.XS_ELLIPSE)

            # IMPORTANT: Get XSec ID AFTER changing shape
            xsec_id = vsp.GetXSec(xsec_surf_id, i)
            vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Width"), width)
            vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Height"), height)

    vsp.Update()
    return fid


def create_fuselage_in_feet(length_feet: float, width_feet: float, height_feet: float) -> str:
    """
    Create a fuselage geometry in OpenVSP with dimensions in feet.
    Used for accurate aerodynamic analysis since OpenVSP API works in feet.
    Args:
        length_feet: Fuselage length in feet
        width_feet: Fuselage width in feet
        height_feet: Fuselage height in feet
    Returns:
        OpenVSP geometry ID of created fuselage
    """
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")
    vsp.SetGeomName(fid, "FuselageGeom_Feet")

    # Set fuselage length parameter (already in feet)
    vsp.SetParmValUpdate(fid, "Length", "Design", length_feet)
    vsp.Update()

    # Configure cross-sections
    xsec_surf_id = vsp.GetXSecSurf(fid, 0)
    num_xsecs = vsp.GetNumXSec(xsec_surf_id)

    for i in range(num_xsecs):
        if i == 0 or i == num_xsecs - 1:
            vsp.ChangeXSecShape(xsec_surf_id, i, vsp.XS_POINT)
        else:
            vsp.ChangeXSecShape(xsec_surf_id, i, vsp.XS_ELLIPSE)
            xsec_id = vsp.GetXSec(xsec_surf_id, i)
            vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Width"), width_feet)
            vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Height"), height_feet)

    vsp.Update()
    return fid


def compute_fuselage_volume_compgeom() -> float:
    """
    Compute fuselage volume using OpenVSP's CompGeom analysis.
    Returns:
        Fuselage volume in cubic meters, or 0.0 if computation fails
    """
    # Run CompGeom analysis on all geometries
    vsp.ComputeCompGeom(vsp.SET_ALL, False, 0)
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
# Aerodynamic Analysis Functions
# ====================
def run_parasite_drag_analysis_feet(fuse_id: str) -> float:
    """
    Run parasite drag analysis on empty fuselage with dimensions in feet.
    This ensures accurate CD calculation since OpenVSP API operates in feet.
    Args:
        fuse_id: OpenVSP geometry ID of the fuselage
    Returns:
        CD value (dimensionless, independent of units)
    """
    try:
        analysis_name = "ParasiteDrag"
        vsp.SetAnalysisInputDefaults(analysis_name)

        Sref = round(OptConfig.ANALYSIS_SREF * 10.76391041671, 2)
        Vinf = round(OptConfig.ANALYSIS_VINF * 3.28084, 2)
        Altitude = round(OptConfig.ANALYSIS_ALTITUDE * 3.28084, 2)
        DeltaTemp = round(OptConfig.ANALYSIS_DELTA_TEMP * 9/5, 2)

        vsp.SetDoubleAnalysisInput(analysis_name, "Sref", (Sref,))
        vsp.SetDoubleAnalysisInput(analysis_name, "Vinf", (Vinf,))
        vsp.SetDoubleAnalysisInput(analysis_name, "Altitude", (Altitude,))
        vsp.SetDoubleAnalysisInput(analysis_name, "DeltaTemp", (DeltaTemp,))

        # Use Mach-based calculation
        vsp.SetIntAnalysisInput(analysis_name, "FreestreamPropChoice", (2,))

        # Include ALL geometries
        vsp.SetIntAnalysisInput(analysis_name, "GeomSet", (vsp.SET_ALL,))

        # Update the model to apply changes
        vsp.Update()

        # Execute analysis
        results_id = vsp.ExecAnalysis(analysis_name)

        if not results_id:
            print("Warning: No results from ParasiteDrag analysis")
            return 1.0

        # Get CD value
        cd_value = None
        try:
            total_cd_values = vsp.GetDoubleResults(results_id, "Total_CD_Total")
            if total_cd_values and len(total_cd_values) > 0:
                cd_value = total_cd_values[0]

        except Exception as e:
            print(f"Error getting CD value: {e}")

        return cd_value if (cd_value and cd_value > 0) else 1.0

    except Exception as e:
        print(f"Parasite drag analysis failed: {e}")
        return 1.0


def calculate_combined_score(cd: float, volume: float,
                             length: float, width: float, height: float,
                             cd_weight: float = OptConfig.CD_WEIGHT,
                             volume_weight: float = OptConfig.VOLUME_WEIGHT) -> float:
    """
    Calculate combined score with CD as primary metric (weight 0.7) and volume secondary (weight 0.3).

    Args:
        cd: Drag coefficient (to minimize)
        volume: Fuselage volume in m³ (to minimize)
        length: Actual fuselage length
        width: Actual fuselage width
        height: Actual fuselage height
        cd_weight: Weight for CD (default 0.7)
        volume_weight: Weight for volume (default 0.3)
    Returns:
        Combined score (lower is better)
    """
    # Use actual dimensions to compute theoretical volume
    theoretical_volume = length * width * height
    volume_normalized = volume / theoretical_volume

    # Normalize CD
    cd_normalized = cd / OptConfig.CD_THRESHOLD

    # Combined score prioritizes CD
    combined = (cd_weight * cd_normalized) + (volume_weight * volume_normalized)

    return combined


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
# Collision Detection and Constraint Validation
# ====================
def generate_box_surface_points(volume: BoxVolume, points_per_edge: int = 3) -> List[Tuple[float, float, float]]:
    """
    Generate multiple test points on the surface of a box volume.
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

    # Add 12 edge midpoints and face centers
    for dy in [-volume.half_width, volume.half_width]:
        for dz in [-volume.half_height, volume.half_height]:
            points.append((volume.x, volume.y + dy, volume.z + dz))

    for dx in [-volume.half_length, volume.half_length]:
        for dz in [-volume.half_height, volume.half_height]:
            points.append((volume.x + dx, volume.y, volume.z + dz))

    for dx in [-volume.half_length, volume.half_length]:
        for dy in [-volume.half_width, volume.half_width]:
            points.append((volume.x + dx, volume.y + dy, volume.z))

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


def find_surface_distance_at_angle(cross_section: CrossSection, angle: float) -> Optional[float]:
    """
    Calculate distance from fuselage center to surface at a specific angle.
    """
    a = cross_section.radius_y
    b = cross_section.radius_z

    if a <= 0 or b <= 0:
        return None

    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)

    denominator = math.sqrt((b * cos_theta) ** 2 + (a * sin_theta) ** 2)
    if denominator == 0:
        return min(a, b)

    surface_dist = (a * b) / denominator

    return surface_dist


def is_point_inside_fuselage(x: float, y: float, z: float,
                             bounds: FuselageBounds,
                             safety_factor: float = OptConfig.SAFETY_MARGIN,
                             debug: bool = False) -> bool:
    """Check if a point is inside the fuselage."""
    u = x_to_u(x, bounds)

    target_cs = None
    min_u_diff = float('inf')

    for cs in bounds.cross_sections:
        u_diff = abs(cs.u - u)
        if u_diff < min_u_diff:
            min_u_diff = u_diff
            target_cs = cs

    if target_cs is None:
        return False

    dy = y - target_cs.center_y
    dz = z - target_cs.center_z

    angle = math.atan2(dz, dy) if dy != 0 or dz != 0 else 0

    surface_dist = find_surface_distance_at_angle(target_cs, angle)
    if surface_dist is None:
        return False

    point_dist = math.sqrt(dy * dy + dz * dz)
    safe_surface_dist = surface_dist * safety_factor

    return point_dist <= safe_surface_dist


def is_box_inside_fuselage(volume: BoxVolume,
                           bounds: FuselageBounds,
                           safety_factor: float = OptConfig.SAFETY_MARGIN,
                           debug: bool = False) -> bool:
    """Check if a box volume is fully contained within the fuselage."""
    if (volume.x - volume.half_length < bounds.x_min + bounds.length * OptConfig.LONGITUDINAL_MARGIN or
            volume.x + volume.half_length > bounds.x_max - bounds.length * OptConfig.LONGITUDINAL_MARGIN):
        if debug:
            print(f"    DEBUG: Volume {volume.id} failed longitudinal bounds check")
        return False

    test_points = generate_box_surface_points(volume, points_per_edge=3)

    for i, (vx, vy, vz) in enumerate(test_points):
        if not is_point_inside_fuselage(vx, vy, vz, bounds, safety_factor, debug):
            if debug:
                print(f"    DEBUG: Volume {volume.id} test point {i} outside fuselage")
            return False

    return True


def boxes_collide(v1: BoxVolume, v2: BoxVolume, margin: float = OptConfig.COLLISION_MARGIN) -> bool:
    """Check if two box volumes collide."""
    x_overlap = (abs(v1.x - v2.x) < (v1.half_length + v2.half_length) * margin)
    y_overlap = (abs(v1.y - v2.y) < (v1.half_width + v2.half_width) * margin)
    z_overlap = (abs(v1.z - v2.z) < (v1.half_height + v2.half_height) * margin)

    return x_overlap and y_overlap and z_overlap


def surface_distance_between_boxes(v1: BoxVolume, v2: BoxVolume) -> float:
    """Calculate the minimum surface-to-surface distance between two box volumes."""
    dx = abs(v1.x - v2.x)
    dy = abs(v1.y - v2.y)
    dz = abs(v1.z - v2.z)

    min_touch_distance_x = v1.half_length + v2.half_length
    min_touch_distance_y = v1.half_width + v2.half_width
    min_touch_distance_z = v1.half_height + v2.half_height

    surface_dist_x = max(0.0, dx - min_touch_distance_x)
    surface_dist_y = max(0.0, dy - min_touch_distance_y)
    surface_dist_z = max(0.0, dz - min_touch_distance_z)

    if surface_dist_x == 0 and surface_dist_y == 0 and surface_dist_z == 0:
        return 0.0

    return math.sqrt(surface_dist_x ** 2 + surface_dist_y ** 2 + surface_dist_z ** 2)


def respects_distance_constraints(vol: BoxVolume, placed_volumes: List[BoxVolume], debug: bool = False) -> bool:
    """Check if a volume respects all its distance constraints with placed volumes."""
    if not vol.distance_constraints:
        return True

    lookup = {v.id: v for v in placed_volumes}

    for target_id, min_dist in vol.distance_constraints:
        target = lookup.get(target_id)
        if target is None:
            continue

        if target_id == vol.id:
            continue

        surface_dist = surface_distance_between_boxes(vol, target)

        if surface_dist < min_dist:
            if debug:
                print(f"    DEBUG: Volume {vol.id} violates distance constraint with Volume {target_id}")
                print(f"           Required min distance: {min_dist:.3f}, Actual surface distance: {surface_dist:.3f}")
            return False

    return True


def is_valid_placement(new_volume: BoxVolume,
                       placed_volumes: List[BoxVolume],
                       bounds: FuselageBounds,
                       debug: bool = False) -> bool:
    """Comprehensive validation for volume placement."""
    if not is_box_inside_fuselage(new_volume, bounds, OptConfig.SAFETY_MARGIN, debug):
        return False

    for pv in placed_volumes:
        if boxes_collide(new_volume, pv):
            return False

    if not respects_distance_constraints(new_volume, placed_volumes, debug):
        return False

    return True


# ====================
# Volume Placement Functions
# ====================
def place_volumes_in_fuselage(bounds: FuselageBounds,
                              base_volumes: List[BoxVolume],
                              debug: bool = False) -> Tuple[List[BoxVolume], int]:
    """Attempt to place all volumes in the fuselage with random placement and validation."""
    placed = []
    skipped = 0

    sorted_volumes = sorted(base_volumes,
                            key=lambda v: v.length * v.width * v.height,
                            reverse=True)

    for base_vol in sorted_volumes:
        placed_successfully = False
        attempts = 0

        while attempts < OptConfig.MAX_PLACEMENT_ATTEMPTS and not placed_successfully:
            u = np.random.random()
            ry, rz, cy, cz = bounds.get_radius_at_u(u)
            x = bounds.x_min + u * bounds.length

            max_y = cy + ry * OptConfig.SAFETY_MARGIN - base_vol.half_width
            min_y = cy - ry * OptConfig.SAFETY_MARGIN + base_vol.half_width
            max_z = cz + rz * OptConfig.SAFETY_MARGIN - base_vol.half_height
            min_z = cz - rz * OptConfig.SAFETY_MARGIN + base_vol.half_height

            if max_y > min_y and max_z > min_z:
                y = np.random.uniform(min_y, max_y)
                z = np.random.uniform(min_z, max_z)

                vol = BoxVolume(
                    id=base_vol.id,
                    x=x, y=y, z=z,
                    length=base_vol.length,
                    width=base_vol.width,
                    height=base_vol.height,
                    distance_constraints=base_vol.distance_constraints
                )

                if is_valid_placement(vol, placed, bounds, debug):
                    placed.append(vol)
                    placed_successfully = True

            attempts += 1

        if not placed_successfully:
            skipped += 1

    return placed, skipped


def create_volumes_in_vsp(volumes: List[BoxVolume]):
    """Create physical box geometries in OpenVSP from placed volume data."""
    for vol in volumes:
        box_id = vsp.AddGeom("Box", "")
        geom_name = f"Volume_{vol.id}"
        vsp.SetGeomName(box_id, geom_name)

        vsp.SetParmValUpdate(box_id, "Abs_Or_Relitive_flag", "XForm", 0)
        vsp.SetParmValUpdate(box_id, "Trans_Attach_Flag", "Attach", 0)
        vsp.SetParmValUpdate(box_id, "X_Location", "XForm", vol.x)
        vsp.SetParmValUpdate(box_id, "Y_Location", "XForm", vol.y)
        vsp.SetParmValUpdate(box_id, "Z_Location", "XForm", vol.z)

        try:
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
    """Comprehensive validation of a complete volume configuration."""
    errors = []

    for i, vol in enumerate(volumes):
        if not is_box_inside_fuselage(vol, bounds, OptConfig.SAFETY_MARGIN):
            errors.append(f"Volume {vol.id} is outside fuselage bounds")

        for j, other_vol in enumerate(volumes):
            if i != j and boxes_collide(vol, other_vol):
                errors.append(f"Volume {vol.id} collides with Volume {other_vol.id}")

        if not respects_distance_constraints(vol, volumes):
            errors.append(f"Volume {vol.id} violates distance constraints")

    return len(errors) == 0, errors


# ====================
# Configuration Generation Functions
# ====================
def generate_step_based_configurations() -> List[FuselageConfiguration]:
    """
    Generate all fuselage configurations based on step parameters.
    Returns a list of all combinations within the specified ranges.
    Total configurations = Length_values × Width_values
    """
    configs = []
    config_id = 1

    # Generate all length values
    lengths = []
    current = OptConfig.LENGTH_MIN

    while current <= OptConfig.LENGTH_MAX + 1e-6:
        lengths.append(current)
        current += OptConfig.LENGTH_STEP

    # Generate all width values (WIDTH = HEIGHT always)
    widths = []
    current = OptConfig.WIDTH_MIN
    while current <= OptConfig.WIDTH_MAX + 1e-6:
        widths.append(current)
        current += OptConfig.WIDTH_STEP
    # Create combinations: L × W
    for length in lengths:
        for width in widths:
            height = width  # HEIGHT = WIDTH
            config = FuselageConfiguration(
                config_id=config_id,
                length=round(length, 6),
                width=round(width, 6),
                height=round(height, 6)
            )
            configs.append(config)
            config_id += 1
    return configs


# ====================
# Main Optimization Functions
# ====================
def run_optimization():
    """Main optimization loop - STEP-BASED approach."""
    print("=" * 80)
    print("FUSELAGE VOLUME OPTIMIZATION - STEP-BASED APPROACH")
    print("Sequence: Configuration Generation → CD Evaluation → Volume Analysis → Scoring")
    print(f"Weights: CD={OptConfig.CD_WEIGHT}, Volume={OptConfig.VOLUME_WEIGHT}")
    print("=" * 80)

    # Create output directory structure
    os.makedirs(OptConfig.OUTPUT_DIR, exist_ok=True)

    # Generate all configurations based on step parameters
    print(f"\n1. Generating configurations with step-based parameters...")
    all_configs = generate_step_based_configurations()
    print(f"   Total configurations: {len(all_configs)}")
    print(f"   Length range: {OptConfig.LENGTH_MIN}-{OptConfig.LENGTH_MAX}m (step: {OptConfig.LENGTH_STEP}m)")
    print(f"   Radius range: {OptConfig.WIDTH_MIN/2}-{OptConfig.WIDTH_MAX/2}m (step: {OptConfig.WIDTH_STEP/2}m)")

    # Load volume definitions from file (NO FALLBACK)
    print(f"\n   Loading volume definitions...")
    if not OptConfig.VOLUME_FILE or not os.path.exists(OptConfig.VOLUME_FILE):
        print(f"   FATAL ERROR: Volume file not specified or not found.")
        print(f"   Path: {OptConfig.VOLUME_FILE}")
        print("   Aborting optimization.")
        return  # Exit optimization

    try:
        base_volumes = load_volumes_from_file(OptConfig.VOLUME_FILE)
        if not base_volumes:
            print(f"   FATAL ERROR: Volume file was loaded but contained no volumes.")
            print("   Aborting optimization.")
            return
    except Exception as e:
        print(f"   FATAL ERROR: Failed to load or parse volume file: {e}")
        print("   Aborting optimization.")
        return

    print(f"\n   Base volumes defined: {len(base_volumes)}")
    print(f"   Total distance constraints: {sum(len(v.distance_constraints) for v in base_volumes)}")

    # ========== STEP 1: CD ANALYSIS IN FEET ==========
    print(f"\n2. Performing CD analysis on {len(all_configs)} configurations...")
    print("   This may take several minutes...")
    cd_analyzed_configs = []

    sys.stdout.flush()

    for config in tqdm(all_configs, desc="CD analysis", unit="config", ncols=100, ascii=True):
        try:
            # Convert to feet for CD analysis
            length_feet = config.length * OptConfig.METERS_TO_FEET
            width_feet = config.width * OptConfig.METERS_TO_FEET
            height_feet = config.height * OptConfig.METERS_TO_FEET

            # Create fuselage in feet for CD analysis
            fuse_id = create_fuselage_in_feet(length_feet, width_feet, height_feet)

            # Run parasite drag analysis
            cd_value = run_parasite_drag_analysis_feet(fuse_id)
            config.cd_value = cd_value

            if cd_value > 0:
                cd_analyzed_configs.append(config)

        except Exception as e:
            print(f"Warning: CD analysis failed for config {config.config_id}: {e}")
            continue
    print()
    print(f"   Successful CD analyses: {len(cd_analyzed_configs)}/{len(all_configs)}")

    # ========== STEP 2: SELECT BEST CD CONFIGURATIONS ==========
    cd_analyzed_configs.sort(key=lambda c: c.cd_value)
    top_n = min(OptConfig.TOP_N_CD, len(cd_analyzed_configs))
    top_cd_configs = cd_analyzed_configs[:top_n]

    print(f"\n3. Selected top {top_n} configurations by CD for volume analysis")
    print(f"   CD range: {top_cd_configs[0].cd_value:.5f} to {top_cd_configs[-1].cd_value:.5f}")

    if not top_cd_configs:
        print("No configurations passed CD analysis! Exiting.")
        return

    # ========== STEP 3: VOLUME ANALYSIS ON TOP CD CONFIGS ==========
    print(f"\n4. Performing volume analysis on top {len(top_cd_configs)} CD configurations...")
    print("   Analyzing fuselage geometry and placing volumes...")
    valid_volume_configs = []

    sys.stdout.flush()

    for config in tqdm(top_cd_configs, desc="Volume analysis", unit="config", ncols=100, ascii=True):
        try:
            # Create fuselage in meters for volume analysis
            fuse_id = create_fuselage_from_dimensions(config.length, config.width, config.height)

            # Analyze fuselage geometry for volume placement
            bounds = config.bounds = analyze_fuselage_geometry(fuse_id)

            # Compute fuselage volume
            fuse_volume = compute_fuselage_volume_compgeom()
            config.fuselage_volume = fuse_volume

            # Attempt to place all volumes
            placed_volumes, num_skipped = place_volumes_in_fuselage(bounds, base_volumes, debug=False)

            if num_skipped == 0:
                is_valid, validation_errors = validate_configuration(placed_volumes, bounds)

                if is_valid:
                    total_boxes_volume = sum([v.length * v.width * v.height for v in placed_volumes])

                    config.volumes_placed = placed_volumes
                    config.total_volumes_volume = total_boxes_volume
                    config.remaining_volume = fuse_volume - total_boxes_volume
                    config.num_volumes_skipped = num_skipped

                    valid_volume_configs.append(config)

        except Exception as e:
            print(f"Warning: Volume analysis failed for config {config.config_id}: {e}")
            continue

    if not valid_volume_configs:
        print("   No configurations passed volume placement validation! Exiting.")
        return
    print()
    print(f"   Successfully placed volumes in {len(valid_volume_configs)} configurations")

    # ========== STEP 4: COMBINED SCORING ==========
    print(f"\n5. Computing combined scores...")
    for config in valid_volume_configs:
        config.combined_score = calculate_combined_score(
            config.cd_value,
            config.fuselage_volume,
            config.length,
            config.width,
            config.height,
            OptConfig.CD_WEIGHT,
            OptConfig.VOLUME_WEIGHT
        )

    valid_volume_configs.sort(key=lambda c: c.combined_score)

    top_n_final = min(OptConfig.TOP_N_FINAL, len(valid_volume_configs))
    top_configs = valid_volume_configs[:top_n_final]

    print(f"\n6. Saving top {top_n_final} configurations...")
    top_dir = os.path.join(OptConfig.OUTPUT_DIR, "top_configurations")
    os.makedirs(top_dir, exist_ok=True)
    print("   Creating output directory structure...")
    sys.stdout.flush()

    for rank, config in enumerate(tqdm(top_configs, desc="Saving configs", unit="file", ncols=100, ascii=True), 1):
        conf_dir = os.path.join(top_dir, f"rank_{rank:02d}_config_{config.config_id}")
        os.makedirs(conf_dir, exist_ok=True)

        # Recreate fuselage geometry for saving (in meters)
        fuse_id = create_fuselage_from_dimensions(config.length, config.width, config.height)
        create_volumes_in_vsp(config.volumes_placed)
        vsp.Update()

        current_bounds = analyze_fuselage_geometry(fuse_id)
        is_valid, validation_errors = validate_configuration(config.volumes_placed, current_bounds)

        if not is_valid:
            print(f"\n   WARNING: Configuration {config.config_id} failed final validation!")
            continue

        # Save VSP file with complete geometry (meters)
        vsp_file = os.path.join(conf_dir, f"fuselage_rank_{rank:02d}_with_volumes.vsp3")
        vsp.WriteVSPFile(vsp_file, vsp.SET_ALL)
        config.vsp_file = vsp_file

        # Save empty fuselage (in feet) for reference
        length_feet = config.length * OptConfig.METERS_TO_FEET
        width_feet = config.width * OptConfig.METERS_TO_FEET
        height_feet = config.height * OptConfig.METERS_TO_FEET

        fuse_id_empty = create_fuselage_in_feet(length_feet, width_feet, height_feet)
        vsp.Update()

        vsp_file_empty = os.path.join(conf_dir, f"fuselage_rank_{rank:02d}_empty.vsp3")
        vsp.WriteVSPFile(vsp_file_empty, vsp.SET_ALL)
        config.vsp_file_empty = vsp_file_empty

        # Save configuration metadata as JSON
        json_file = os.path.join(conf_dir, "config_data.json")
        with open(json_file, "w") as f:
            json.dump(config.to_dict(), f, indent=2)

        # Save volume positions as CSV
        save_volumes_positions_csv(config.volumes_placed, conf_dir)

        # Save constraint verification results as CSV
        save_constraints_csv(config.volumes_placed, base_volumes, conf_dir)

    # Save optimization summary
    print(f"\n7. Saving summary CSV files...")
    summary_csv = os.path.join(OptConfig.OUTPUT_DIR, "optimization_summary.csv")
    with open(summary_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Rank", "Config_ID", "Length_m", "Width_m", "Height_m",
                         "Fuselage_Volume_m3", "CD_Value", "Combined_Score"])

        for rank, config in enumerate(top_configs, 1):
            writer.writerow([
                rank, config.config_id,
                f"{config.length:.3f}", f"{config.width:.3f}", f"{config.height:.3f}",
                f"{config.fuselage_volume:.3f}",
                f"{config.cd_value:.5f}", f"{config.combined_score:.5f}"
            ])

    # Save aerodynamic analysis results
    aero_csv = os.path.join(OptConfig.OUTPUT_DIR, "cd_analysis_all_configs.csv")
    with open(aero_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Config_ID", "Length_m", "Width_m", "Height_m", "CD_Value", "Rank_by_CD"])

        for rank, config in enumerate(sorted(cd_analyzed_configs, key=lambda c: c.cd_value), 1):
            writer.writerow([
                config.config_id,
                f"{config.length:.3f}", f"{config.width:.3f}", f"{config.height:.3f}",
                f"{config.cd_value:.5f}", rank
            ])

    # Generate analysis plots
    print(f"\n8. Generating visualizations...")
    generate_plots(cd_analyzed_configs, top_configs, valid_volume_configs)

    # Print completion message
    print("\n" + "=" * 80)
    print("OPTIMIZATION COMPLETE!")
    print(f"Results saved to: {OptConfig.OUTPUT_DIR}")
    print(f"Top {top_n_final} configurations in: {top_dir}")
    print("=" * 80)

    # Display top 5 configuration summary
    print("\nTop configurations by combined score:")
    print("-" * 80)
    for i, config in enumerate(top_configs[:5], 1):
        print(f"{i}. Config {config.config_id}: Combined Score = {config.combined_score:.5f}")
        print(f"   CD = {config.cd_value:.5f}, Volume = {config.fuselage_volume:.3f} m³")
        print(f"   L={config.length:.3f}m, W={config.width:.3f}m, H={config.height:.3f}m")
        print(f"   Volumes: {len(config.volumes_placed)} placed, {config.num_volumes_skipped} skipped")


def save_volumes_positions_csv(placed_volumes: List[BoxVolume], conf_dir: str):
    """Save volume positions and dimensions to CSV file."""
    csv_file = os.path.join(conf_dir, "volumes_positions.csv")

    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Volume_ID", "X", "Y", "Z", "Length", "Width", "Height"])

        for vol in placed_volumes:
            writer.writerow([
                vol.id,
                f"{vol.x:.3f}",
                f"{vol.y:.3f}",
                f"{vol.z:.3f}",
                f"{vol.length:.3f}",
                f"{vol.width:.3f}",
                f"{vol.height:.3f}"
            ])


def save_constraints_csv(placed_volumes: List[BoxVolume],
                         all_volumes: List[BoxVolume],
                         conf_dir: str):
    """Save distance constraint verification results to CSV file."""
    csv_file = os.path.join(conf_dir, "distance_constraints.csv")

    placed_ids = {vol.id for vol in placed_volumes}
    placed_lookup = {vol.id: vol for vol in placed_volumes}

    unique_constraints = {}
    for volume in all_volumes:
        for target_id, min_distance in volume.distance_constraints:
            pair_key = tuple(sorted([volume.id, target_id]))
            if pair_key in unique_constraints:
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
            "Min_Surface_Distance_m", "Actual_Surface_Distance_m",
            "Satisfied", "Both_Placed"
        ])

        for vol1_id, vol2_id, min_dist in constraints_list:
            both_placed = "YES" if (vol1_id in placed_ids and vol2_id in placed_ids) else "NO"

            if both_placed == "YES":
                vol1 = placed_lookup[vol1_id]
                vol2 = placed_lookup[vol2_id]
                actual_dist = surface_distance_between_boxes(vol1, vol2)
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


def load_volumes_from_file(filepath: str) -> List[BoxVolume]:
    """
    Load volume definitions from an external JSON file.
    Args:
        filepath: Path to the JSON file.
    Returns:
        List of BoxVolume objects.
    """
    print(f"   Parsing volume file: {filepath}")
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)

        volumes = []
        for item in data:
            # Convert constraint list [id, dist] to tuple (id, dist)
            constraints = [
                tuple(c) for c in item.get('distance_constraints', [])
            ]

            vol = BoxVolume(
                id=item['id'],
                length=item['length'],
                width=item['width'],
                height=item['height'],
                distance_constraints=constraints
            )
            volumes.append(vol)

        if not volumes:
            print("   WARNING: Volume file was empty or invalid. No volumes loaded.")
        return volumes

    except json.JSONDecodeError as e:
        print(f"   ERROR: Could not parse volume file (invalid JSON): {e}")
        raise ValueError(f"Invalid JSON in {filepath}") from e
    except KeyError as e:
        print(f"   ERROR: Volume file is missing required key: {e}")
        raise ValueError(f"Missing key {e} in {filepath}") from e
    except Exception as e:
        print(f"   ERROR: Failed to load volumes from file: {e}")
        raise

def generate_plots(all_cd_configs: List[FuselageConfiguration],
                   top_configs: List[FuselageConfiguration],
                   volume_configs: List[FuselageConfiguration]):
    """Generate analysis plots for step-based optimization results."""
    all_cds = [c.cd_value for c in all_cd_configs]
    top_cds = [c.cd_value for c in top_configs]

    volume_data_cds = [c.cd_value for c in volume_configs]
    volume_data_volumes = [c.fuselage_volume for c in volume_configs]
    top_volumes = [c.fuselage_volume for c in top_configs]

    top_scores = [c.combined_score for c in top_configs]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # 1. Distribution of CD values
    ax1 = axes[0, 0]
    ax1.hist(all_cds, bins=20, alpha=0.7, color='lightblue', edgecolor='black')
    if top_cds:
        ax1.axvline(np.mean(top_cds), color='red', linestyle='--',
                    label=f'Top {len(top_configs)} mean CD')
    ax1.set_xlabel('Drag Coefficient (CD)')
    ax1.set_ylabel('Count')
    ax1.set_title('CD Distribution - All Configurations')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Top configurations ranked by combined score
    ax2 = axes[0, 1]
    config_ids = [f"Config {c.config_id}" for c in top_configs]
    bars = ax2.barh(config_ids, top_scores, color='steelblue', edgecolor='black')
    ax2.set_xlabel('Combined Score')
    ax2.set_title(f'Top {len(top_configs)} Configurations by Combined Score')
    ax2.invert_yaxis()
    for i, (bar, config) in enumerate(zip(bars, top_configs)):
        ax2.text(bar.get_width(), bar.get_y() + bar.get_height()/2,
                f' CD:{config.cd_value:.4f}', va='center', fontsize=9)
    ax2.grid(True, alpha=0.3, axis='x')

    # 3. CD vs Volume scatter
    ax3 = axes[0, 2]
    ax3.scatter(top_cds, top_volumes, color='red', s=100,
        label=f'Top {len(top_configs)}', edgecolor='black', linewidth=2, marker='*')
    ax3.set_xlabel('Drag Coefficient (CD) - PRIMARY METRIC')
    ax3.set_ylabel('Fuselage Volume (m³)')
    ax3.set_title('CD vs Volume (CD dominates scoring)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. CD ranking - showing selection process
    ax4 = axes[1, 0]
    sorted_cd_configs = sorted(all_cd_configs, key=lambda c: c.cd_value)
    ranks = range(1, len(sorted_cd_configs) + 1)
    cds = [c.cd_value for c in sorted_cd_configs]
    ax4.plot(ranks, cds, marker='o', linestyle='-', alpha=0.7, label='All configs')
    top_n = min(OptConfig.TOP_N_CD, len(sorted_cd_configs))
    ax4.axvline(x=top_n, color='red', linestyle='--', label=f'Top {top_n} selected for volume analysis')
    ax4.fill_between(range(1, top_n + 1), 0, max(cds), alpha=0.2, color='green')
    ax4.set_xlabel('Configuration Rank (by CD)')
    ax4.set_ylabel('CD Value')
    ax4.set_title('CD-based Selection Process')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 5. Score components breakdown
    ax5 = axes[1, 1]
    if top_configs:
        config_labels = [f"C{c.config_id}" for c in top_configs]
        cd_contributions = [c.cd_value / OptConfig.CD_THRESHOLD * 0.7 for c in top_configs]
        volume_contributions = []

        for c in top_configs:
            theoretical_vol = c.length * c.width * c.height
            vol_norm = c.fuselage_volume / theoretical_vol
            volume_contributions.append(vol_norm * 0.3)

        x_pos = np.arange(len(config_labels))
        width = 0.35

        bars1 = ax5.bar(x_pos - width/2, cd_contributions, width, label='CD component',
                       color='#FF6B6B', alpha=0.8, edgecolor='black')
        bars2 = ax5.bar(x_pos + width/2, volume_contributions, width, label='Volume component',
                       color='#4ECDC4', alpha=0.8, edgecolor='black')

        ax5.set_ylabel('Score Component Value')
        ax5.set_title('Score Breakdown')
        ax5.set_xticks(x_pos)
        ax5.set_xticklabels(config_labels)
        ax5.legend()
        ax5.grid(True, alpha=0.3, axis='y')

    # 6. Summary metrics
    ax6 = axes[1, 2]
    ax6.axis('off')

    summary_text = f"""
OPTIMIZATION SUMMARY (STEP-BASED APPROACH)

Initial Configurations: {len(all_cd_configs)}
After Volume Placement: {len(volume_configs)}
Final Top-N: {len(top_configs)}

CD Analysis:
  • Best CD: {min(all_cds):.5f}
  • Worst CD: {max(all_cds):.5f}
  • Threshold: {OptConfig.CD_THRESHOLD:.5f}

Top Configuration:
  • Config ID: {top_configs[0].config_id if top_configs else 'N/A'}
  • CD: {top_configs[0].cd_value:.5f}
  • Volume: {top_configs[0].fuselage_volume:.3f} m³
  • Score: {top_configs[0].combined_score:.5f}

Weights Applied:
  • CD Weight: {OptConfig.CD_WEIGHT}
  • Volume Weight: {OptConfig.VOLUME_WEIGHT}

Step Parameters:
  • Length: {OptConfig.LENGTH_MIN}-{OptConfig.LENGTH_MAX}m (step {OptConfig.LENGTH_STEP}m)
  • Radius: {OptConfig.WIDTH_MIN/2}-{OptConfig.WIDTH_MAX/2}m (step {OptConfig.WIDTH_STEP/2}m)
    """

    ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes,
            fontsize=9, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()

    # Save plot to file
    plot_file = os.path.join(OptConfig.OUTPUT_DIR, "step_based_optimization_analysis.png")
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"   Saved plot: {plot_file}")

    plt.close()


# ====================
# Entry Point
# ====================
if __name__ == "__main__":
    run_optimization()