import openvsp as vsp
import csv
import json
import os
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass, asdict, field


# ====================
# Configuration
# ====================
class Config:
    """Configuration parameters for volume placement"""

    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    INPUT_VSP = os.path.join(BASE_DIR, "input_fuse.vsp3")
    OUTPUT_DIR = os.path.join(BASE_DIR, "configs")
    NUM_VOLUMES = 10

    # Placement parameters
    SAFETY_MARGIN = 0.8  # Keep volumes within 80% of local radius
    LONGITUDINAL_MARGIN = 0.05  # Margin from fuselage ends (5% of length)

    # Surface sampling parameters
    U_SAMPLES = 100  # Longitudinal samples (along fuselage length)
    W_SAMPLES = 36  # Circumferential samples (around fuselage)


# ====================
# Data Classes
# ====================
@dataclass
class EllipsoidVolume:
    """
    Represents a single ellipsoid volume with its properties.
    Now supports defining dimensions (length,width,height) and auto-calculates radii.
    distance_constraints: List of tuples (target_volume_id, min_distance_meters)
    """
    id: int
    x: float
    y: float
    z: float
    length: float  # full length (X axis) in meters
    width: float  # full width (Y axis) in meters
    height: float  # full height (Z axis) in meters
    distance_constraints: List[Tuple[int, float]] = field(default_factory=list)

    # radii will be derived from dimensions
    radius_x: float = 0.0
    radius_y: float = 0.0
    radius_z: float = 0.0

    def __post_init__(self):
        # Derive radii from dimensions if not provided
        self.radius_x = self.length / 2.0
        self.radius_y = self.width / 2.0
        self.radius_z = self.height / 2.0

    def to_list(self):
        """Convert to list for CSV export (dimensions + center position)"""
        return [self.id, self.x, self.y, self.z,
                self.length, self.width, self.height]

    def to_dict(self):
        """Convert to dictionary for JSON export"""
        data = asdict(self)
        # Format distance constraints as objects
        data["distance_constraints"] = [
            {"target_volume": t, "min_distance": d} for (t, d) in self.distance_constraints
        ]
        return data


@dataclass
class CrossSection:
    """Represents a cross-section of the fuselage at a specific u-location"""
    u: float  # Parametric coordinate along fuselage (0-1)
    x: float  # Physical X coordinate
    radius_y: float  # Half-width in Y direction
    radius_z: float  # Half-height in Z direction
    area: float
    center_y: float  # Y coordinate of center
    center_z: float  # Z coordinate of center


@dataclass
class FuselageBounds:
    """Enhanced fuselage bounds with actual surface-sampled data"""
    length: float
    x_min: float
    x_max: float
    cross_sections: List[CrossSection]
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]
    surf_indx: int  # Main surface index

    def get_radius_at_u(self, u: float) -> Tuple[float, float, float, float]:
        """
        Get effective radius and center at specific u position using real sampled data
        Uses linear interpolation between known cross-sections

        Returns: (radius_y, radius_z, center_y, center_z)
        """
        if not self.cross_sections:
            return 1.0, 1.0, 0.0, 0.0

        # Clamp u to [0, 1]
        u = max(0.0, min(1.0, u))

        # Find surrounding cross-sections
        for i, cs in enumerate(self.cross_sections):
            if cs.u >= u:
                if i == 0:
                    return cs.radius_y, cs.radius_z, cs.center_y, cs.center_z

                # Interpolate between cs[i-1] and cs[i]
                cs_prev = self.cross_sections[i - 1]
                t = (u - cs_prev.u) / (cs.u - cs_prev.u) if cs.u != cs_prev.u else 0

                radius_y = cs_prev.radius_y + t * (cs.radius_y - cs_prev.radius_y)
                radius_z = cs_prev.radius_z + t * (cs.radius_z - cs_prev.radius_z)
                center_y = cs_prev.center_y + t * (cs.center_y - cs_prev.center_y)
                center_z = cs_prev.center_z + t * (cs.center_z - cs_prev.center_z)

                return radius_y, radius_z, center_y, center_z

        # If u is beyond last cross-section
        last = self.cross_sections[-1]
        return last.radius_y, last.radius_z, last.center_y, last.center_z


# ====================
# Geometry analysis (unchanged)
# ====================
def analyze_fuselage_geometry_real(fuse_id: str) -> FuselageBounds:
    """
    Analyze fuselage geometry using OpenVSP surface query functions
    to get REAL geometry data from the actual surface
    """
    print("  Analyzing fuselage geometry using REAL surface data...")

    # Get bounding box for reference
    bbox_min_vec = vsp.GetGeomBBoxMin(fuse_id, 0, True)
    bbox_max_vec = vsp.GetGeomBBoxMax(fuse_id, 0, True)

    bbox_min = (bbox_min_vec.x(), bbox_min_vec.y(), bbox_min_vec.z())
    bbox_max = (bbox_max_vec.x(), bbox_max_vec.y(), bbox_max_vec.z())

    length = bbox_max[0] - bbox_min[0]

    print(f"    Bounding box: min{bbox_min}, max{bbox_max}")
    print(f"    Length: {length:.2f}")

    # Get main surface index (usually 0 for fuselage)
    surf_indx = 0

    cross_sections = []

    print(f"    Sampling {Config.U_SAMPLES} cross-sections with {Config.W_SAMPLES} points each...")

    # Sample cross-sections along the fuselage using actual surface points
    for i in range(Config.U_SAMPLES):
        u = i / (Config.U_SAMPLES - 1) if Config.U_SAMPLES > 1 else 0.5

        # Sample points around the circumference at this u location
        points_y = []
        points_z = []
        points_x = []

        for j in range(Config.W_SAMPLES):
            w = j / Config.W_SAMPLES  # Full circle: 0 to 1

            # Get actual 3D point on surface using OpenVSP
            try:
                point = vsp.CompPnt01(fuse_id, surf_indx, u, w)
                points_x.append(point.x())
                points_y.append(point.y())
                points_z.append(point.z())
            except Exception as e:
                # Fallback if CompPnt01 fails
                print(f"      Warning: CompPnt01 failed at u={u:.3f}, w={w:.3f} -> {e}")
                continue

        if len(points_y) < 4:
            print(f"      Warning: Insufficient points at u={u:.3f}, skipping")
            continue

        # Calculate center of this cross-section
        center_y = sum(points_y) / len(points_y)
        center_z = sum(points_z) / len(points_z)
        avg_x = sum(points_x) / len(points_x)

        # Calculate radii (distance from center to surface)
        radii_y = [abs(y - center_y) for y in points_y]
        radii_z = [abs(z - center_z) for z in points_z]

        # Use maximum extents for radius
        radius_y = max(radii_y) if radii_y else 0.5
        radius_z = max(radii_z) if radii_z else 0.5

        # Calculate approximate area
        area = math.pi * radius_y * radius_z

        cs = CrossSection(
            u=u,
            x=avg_x,
            radius_y=radius_y,
            radius_z=radius_z,
            area=area,
            center_y=center_y,
            center_z=center_z
        )
        cross_sections.append(cs)

        if i % 20 == 0:  # Progress indicator
            print(f"      u={u:.3f}: X={avg_x:.2f}, center=({center_y:.3f}, {center_z:.3f}), "
                  f"radii=({radius_y:.3f}, {radius_z:.3f})")

    if not cross_sections:
        raise RuntimeError("Failed to sample any valid cross-sections from fuselage surface!")

    print(f"    ✓ Successfully sampled {len(cross_sections)} real cross-sections from surface")

    # Get actual X range from sampled data
    x_min_real = min(cs.x for cs in cross_sections)
    x_max_real = max(cs.x for cs in cross_sections)

    return FuselageBounds(
        length=x_max_real - x_min_real,
        x_min=x_min_real,
        x_max=x_max_real,
        cross_sections=cross_sections,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        surf_indx=surf_indx
    )


# ====================
# Helper coordinate conversions
# ====================
def x_to_u(x: float, bounds: FuselageBounds) -> float:
    """Convert X coordinate to u parameter (0-1)"""
    if bounds.length == 0:
        return 0.5
    return (x - bounds.x_min) / bounds.length


def u_to_x(u: float, bounds: FuselageBounds) -> float:
    """Convert u parameter (0-1) to X coordinate"""
    return bounds.x_min + u * bounds.length


# ====================
# Controlli e collisioni
# ====================
def is_ellipsoid_inside_fuselage(volume: EllipsoidVolume,
                                 bounds: FuselageBounds,
                                 safety_factor: float = Config.SAFETY_MARGIN,
                                 debug: bool = False) -> bool:
    """
    Check if an ellipsoid is fully contained within the fuselage
    using REAL cross-sectional data from surface sampling
    """
    # Check longitudinal bounds (use envelope along X with half-length radius_x)
    if (volume.x - volume.radius_x < bounds.x_min or
            volume.x + volume.radius_x > bounds.x_max):
        if debug:
            print(f"    DEBUG: Volume {volume.id} LONGITUDINAL OVERFLOW!")
            print(f"           Volume X range: [{volume.x - volume.radius_x:.2f}, {volume.x + volume.radius_x:.2f}]")
            print(f"           Fuselage X range: [{bounds.x_min:.2f}, {bounds.x_max:.2f}]")
        return False

    # Convert X position to u parameter
    u = x_to_u(volume.x, bounds)

    # Get REAL effective radius at this u position from actual surface data
    eff_radius_y, eff_radius_z, center_y, center_z = bounds.get_radius_at_u(u)

    # Apply safety factor
    safe_radius_y = eff_radius_y * safety_factor
    safe_radius_z = eff_radius_z * safety_factor

    # Check if ellipsoid fits within cross-section (relative to actual center)
    y_extent = abs(volume.y - center_y) + volume.radius_y
    z_extent = abs(volume.z - center_z) + volume.radius_z

    y_overflow = y_extent > safe_radius_y
    z_overflow = z_extent > safe_radius_z

    if debug and (y_overflow or z_overflow):
        print(f"    DEBUG: Volume {volume.id} CROSS-SECTION OVERFLOW at X={volume.x:.2f} (u={u:.3f})!")
        print(f"           REAL surface radii: Y={eff_radius_y:.2f}, Z={eff_radius_z:.2f}")
        print(f"           REAL center: Y={center_y:.3f}, Z={center_z:.3f}")
        print(f"           Safe radii (with {safety_factor:.1f} factor): Y={safe_radius_y:.2f}, Z={safe_radius_z:.2f}")
        print(f"           Volume position: Y={volume.y:.2f}, Z={volume.z:.2f}")
        print(f"           Volume radii: Y={volume.radius_y:.2f}, Z={volume.radius_z:.2f}")
        print(f"           Y extent from center: {y_extent:.2f} vs safe={safe_radius_y:.2f}")
        print(f"           Z extent from center: {z_extent:.2f} vs safe={safe_radius_z:.2f}")
        if y_overflow:
            print(f"           Y OVERFLOW: {y_extent:.2f} > {safe_radius_y:.2f}")
        if z_overflow:
            print(f"           Z OVERFLOW: {z_extent:.2f} > {safe_radius_z:.2f}")

    return not (y_overflow or z_overflow)


def volumes_collide(v1: EllipsoidVolume, v2: EllipsoidVolume, margin: float = 1.05) -> bool:
    """
    Check if two ellipsoid volumes collide with safety margin.
    Uses center-to-center distance vs averaged radii sum.
    margin slightly >1 to ensure a small clearance.
    """
    dx = v1.x - v2.x
    dy = v1.y - v2.y
    dz = v1.z - v2.z
    dist_sq = dx * dx + dy * dy + dz * dz

    # Use average radius for collision detection
    r1 = (v1.radius_x + v1.radius_y + v1.radius_z) / 3
    r2 = (v2.radius_x + v2.radius_y + v2.radius_z) / 3
    min_dist = (r1 + r2) * margin

    return dist_sq < (min_dist * min_dist)


def respects_distance_constraints_against_manual(vol: EllipsoidVolume,
                                                 manual_volumes: List[EllipsoidVolume],
                                                 debug: bool = False) -> bool:
    """
    Check if 'vol' respects all its distance constraints relative to
    the manual_volumes list (even if the target hasn't been placed yet).
    """
    if not vol.distance_constraints:
        return True

    # Build lookup by id for manual volumes
    lookup = {v.id: v for v in manual_volumes}
    for target_id, min_dist in vol.distance_constraints:
        target = lookup.get(target_id)
        if target is None:
            # If target not in manual list, ignore (user likely referenced non-existent id)
            if debug:
                print(f"    DEBUG: Volume {vol.id} has constraint to missing Volume {target_id}, ignoring.")
            continue

        dx = vol.x - target.x
        dy = vol.y - target.y
        dz = vol.z - target.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < min_dist:
            if debug:
                print(f"    DEBUG: Volume {vol.id} violates distance constraint with Volume {target_id}: "
                      f"dist={dist:.3f} < min={min_dist:.3f}")
            return False

    return True


def is_valid_against_placed_and_constraints(new_volume: EllipsoidVolume,
                                            placed_volumes: List[EllipsoidVolume],
                                            manual_volumes: List[EllipsoidVolume],
                                            bounds: FuselageBounds,
                                            debug: bool = False) -> bool:
    """
    Combined validator:
    - inside fuselage
    - no collision with already placed volumes
    - respects distance constraints against manual list (absolute)
    """
    # Inside fuselage
    if not is_ellipsoid_inside_fuselage(new_volume, bounds, Config.SAFETY_MARGIN, debug):
        if debug:
            print(f"    -> Volume {new_volume.id} OUTSIDE fuselage (skipped)")
        return False

    # Collision with placed volumes
    for pv in placed_volumes:
        if volumes_collide(new_volume, pv):
            if debug:
                dx = new_volume.x - pv.x
                dy = new_volume.y - pv.y
                dz = new_volume.z - pv.z
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                rsum = ((new_volume.radius_x + pv.radius_x) / 2.0 + (new_volume.radius_y + pv.radius_y) / 2.0 + (
                        new_volume.radius_z + pv.radius_z) / 2.0) / 3.0
                print(f"    DEBUG: Volume {new_volume.id} COLLIDES with placed Volume {pv.id}")
                print(f"           Distance={distance:.3f}, Approx.Required>={(rsum * 1.05):.3f}")
            return False

    # Distance constraints vs manual list (absolute)
    if not respects_distance_constraints_against_manual(new_volume, manual_volumes, debug):
        if debug:
            print(f"    -> Volume {new_volume.id} violates distance constraints (skipped)")
        return False

    return True


# ====================
# Manual placement function
# ====================
def place_manual_volumes(fuse_id: str,
                         bounds: FuselageBounds,
                         manual_volumes: List[EllipsoidVolume]) -> List[EllipsoidVolume]:
    """
    Place manually-defined volumes, checking real geometry, collisions,
    and custom distance constraints. Skip invalid ones.
    """
    placed = []

    print(f"  Placing {len(manual_volumes)} manually defined volumes...")

    for vol in manual_volumes:
        print(f"\n  → Checking Volume {vol.id} (X={vol.x:.2f}, Y={vol.y:.2f}, Z={vol.z:.2f}, "
              f"LxWxH=({vol.length:.2f},{vol.width:.2f},{vol.height:.2f}))...")

        # Validate position against placed volumes and absolute constraints
        if not is_valid_against_placed_and_constraints(vol, placed, manual_volumes, bounds, debug=True):
            print(f"    ✗ Volume {vol.id} skipped (invalid / constraint / collision)")
            continue

        # Create the ellipsoid in OpenVSP
        ellipsoid_id = vsp.AddGeom("ELLIPSOID", "")

        #Giving a name for each volume
        geom_name = f"Volume_{vol.id}"
        vsp.SetGeomName(ellipsoid_id, geom_name)

        #Define the parameters
        vsp.SetParmValUpdate(ellipsoid_id, "Abs_Or_Relitive_flag", "XForm", 0)
        vsp.SetParmValUpdate(ellipsoid_id, "Trans_Attach_Flag", "Attach", 0)
        vsp.SetParmValUpdate(ellipsoid_id, "X_Location", "XForm", vol.x)
        vsp.SetParmValUpdate(ellipsoid_id, "Y_Location", "XForm", vol.y)
        vsp.SetParmValUpdate(ellipsoid_id, "Z_Location", "XForm", vol.z)

        try:
            # OpenVSP ellipsoid parameters are A_Radius (X), B_Radius (Y), C_Radius (Z)
            vsp.SetParmValUpdate(ellipsoid_id, "A_Radius", "Design", vol.radius_x)
            vsp.SetParmValUpdate(ellipsoid_id, "B_Radius", "Design", vol.radius_y)
            vsp.SetParmValUpdate(ellipsoid_id, "C_Radius", "Design", vol.radius_z)
        except Exception as e:
            print(f"    Warning: could not set radii for Volume {vol.id}: {e}")

        vsp.Update()
        placed.append(vol)
        print(f"    ✓ Volume {vol.id} placed successfully and verified as '{geom_name}'")

    print(f"\n  ✓ Total valid volumes placed: {len(placed)}/{len(manual_volumes)}")

    return placed

# ====================
# Save configuration (modified for single config with auto-numbering)
# ====================
def get_next_config_number(output_dir: str) -> int:
    """
    Find the next available configuration number by scanning existing directories
    """
    if not os.path.exists(output_dir):
        return 1

    existing_configs = []
    for item in os.listdir(output_dir):
        if os.path.isdir(os.path.join(output_dir, item)) and item.startswith("conf"):
            try:
                # Extract number from "confX" directory name
                num = int(item[4:])
                existing_configs.append(num)
            except ValueError:
                continue

    if not existing_configs:
        return 1

    return max(existing_configs) + 1


def save_constraints_csv(manual_volumes: List[EllipsoidVolume],
                         placed_volumes: List[EllipsoidVolume],
                         conf_dir: str):
    """
    Save a CSV file with all distance constraints and their verification status
    Optimized to avoid duplicate pairs (only stores unique volume pairs)
    """
    csv_file = os.path.join(conf_dir, "distance_constraints.csv")

    placed_ids = {vol.id for vol in placed_volumes}
    placed_lookup = {vol.id: vol for vol in placed_volumes}

    # Collect all constraints from all volumes using unique pairs
    unique_constraints = {}
    for volume in manual_volumes:
        for target_id, min_distance in volume.distance_constraints:
            # Create a unique key for the pair (always store with smaller ID first)
            pair_key = tuple(sorted([volume.id, target_id]))

            # If we already have this pair, keep the minimum distance requirement
            if pair_key in unique_constraints:
                existing_min_dist = unique_constraints[pair_key][2]
                unique_constraints[pair_key] = (
                    pair_key[0], pair_key[1], min(existing_min_dist, min_distance)
                )
            else:
                unique_constraints[pair_key] = (pair_key[0], pair_key[1], min_distance)

    # Convert to list and sort
    constraints_list = list(unique_constraints.values())
    constraints_list.sort(key=lambda x: (x[0], x[1]))

    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        # Write header
        writer.writerow([
            "Volume_1", "Volume_2",
            "Min_Distance_m", "Actual_Distance_m",
            "Satisfied", "Both_Placed"
        ])

        # Write each constraint
        for vol1_id, vol2_id, min_dist in constraints_list:
            both_placed = "YES" if (vol1_id in placed_ids and vol2_id in placed_ids) else "NO"

            if both_placed == "YES":
                # Calculate actual distance
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

    print(f"  Saved constraints CSV: {csv_file}")
    return len(constraints_list)


def save_configuration(volumes: List[EllipsoidVolume],
                       bounds: FuselageBounds,
                       output_dir: str):
    """
    Save configuration files with enhanced metadata and auto-numbering
    """
    # Create main output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Get next configuration number
    conf_num = get_next_config_number(output_dir)
    conf_dir = os.path.join(output_dir, f"conf{conf_num}")
    os.makedirs(conf_dir, exist_ok=True)

    print(f"  Creating configuration folder: conf{conf_num}")

    # Save VSP3 file
    vsp_file = os.path.join(conf_dir, f"fuse_conf{conf_num}.vsp3")
    vsp.WriteVSPFile(vsp_file, vsp.SET_ALL)
    print(f"  Saved VSP3: {vsp_file}")

    # Save CSV file (dimensions + position)
    csv_file = os.path.join(conf_dir, "volumes_positions.csv")
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Volume_ID", "X", "Y", "Z", "Length", "Width", "Height"])
        for vol in volumes:
            writer.writerow(vol.to_list())
    print(f"  Saved CSV: {csv_file}")

    # Save constraints CSV
    manual_volumes = get_manual_volumes_example()  # Get the original manual volumes for constraints
    num_constraints = save_constraints_csv(manual_volumes, volumes, conf_dir)

    # Save enhanced JSON file with REAL surface data
    json_file = os.path.join(conf_dir, "volumes_data.json")
    config_data = {
        "configuration": conf_num,
        "num_volumes": len(volumes),
        "num_constraints": num_constraints,
        "sampling_method": "REAL_SURFACE_GEOMETRY",
        "sampling_params": {
            "u_samples": Config.U_SAMPLES,
            "w_samples": Config.W_SAMPLES
        },
        "fuselage_bounds": {
            "length": bounds.length,
            "x_min": bounds.x_min,
            "x_max": bounds.x_max,
            "bbox_min": bounds.bbox_min,
            "bbox_max": bounds.bbox_max,
            "num_cross_sections": len(bounds.cross_sections),
            "surface_index": bounds.surf_indx
        },
        "volumes": [vol.to_dict() for vol in volumes],
        "cross_sections_real": [
            {
                "u": cs.u,
                "x": cs.x,
                "center_y": cs.center_y,
                "center_z": cs.center_z,
                "radius_y": cs.radius_y,
                "radius_z": cs.radius_z,
                "area": cs.area
            } for cs in bounds.cross_sections
        ]
    }
    with open(json_file, "w") as f:
        json.dump(config_data, f, indent=2)
    print(f"  Saved JSON with REAL surface data: {json_file}")

    return conf_num


# ====================
# Manual volumes definition (10 volumes)
# Modify positions/dimensions/constraints
# ====================
def get_manual_volumes_example() -> List[EllipsoidVolume]:
    """
    Return a list of 10 manually defined EllipsoidVolume objects.
    Units: meters. Distance constraints in meters.
    Edit this function to set positions/dimensions.
    """
    mv = [
        EllipsoidVolume(id=1, x=0.50, y=0.00, z=0.00, length=0.40, width=0.40, height=0.40,
                        distance_constraints=[(2, 0.6), (3, 0.8), (4, 0.6)]),
        EllipsoidVolume(id=2, x=1.10, y=0.10, z=0.00, length=0.30, width=0.30, height=0.30,
                        distance_constraints=[(1, 0.6), (3, 0.25), (5, 0.5)]),
        EllipsoidVolume(id=3, x=1.80, y=-0.10, z=0.00, length=0.50, width=0.40, height=0.40,
                        distance_constraints=[(1, 0.8), (6, 0.4)]),
        EllipsoidVolume(id=4, x=2.50, y=0.00, z=0.10, length=0.35, width=0.35, height=0.35,
                        distance_constraints=[(1, 0.6), (5, 0.5), (7, 0.35)]),
        EllipsoidVolume(id=5, x=3.05, y=0.12, z=-0.05, length=0.45, width=0.40, height=0.30,
                        distance_constraints=[(2, 0.5), (4, 0.5), (8, 0.55)]),
        EllipsoidVolume(id=6, x=3.60, y=-0.20, z=0.05, length=0.30, width=0.30, height=0.30,
                        distance_constraints=[(3, 0.4), (7, 0.4), (10, 1.0)]),
        EllipsoidVolume(id=7, x=4.20, y=0.00, z=0.00, length=0.40, width=0.40, height=0.40,
                        distance_constraints=[(4, 0.35), (6, 0.4), (10, 0.38)]),
        EllipsoidVolume(id=8, x=4.80, y=0.15, z=-0.10, length=0.30, width=0.30, height=0.30,
                        distance_constraints=[(5, 0.55)]),
        EllipsoidVolume(id=9, x=19.00, y=-0.12, z=0.08, length=0.50, width=0.45, height=0.40,
                        distance_constraints=[(3, 1.2)]),
        EllipsoidVolume(id=10, x=5.95, y=0.00, z=0.00, length=0.60, width=0.50, height=0.45,
                        distance_constraints=[(6, 1.0), (7, 0.38)]),
    ]
    return mv


# ====================
# Main Execution
# ====================
def main():
    """Main execution function - generates ONE configuration with auto-numbering"""

    print("=" * 70)
    print("OpenVSP Manual Volume Placement - REAL SURFACE GEOMETRY + Distance Constraints")
    print("Single Configuration with Auto-numbering")
    print("=" * 70)
    print("Using vsp.CompPnt01() to sample actual surface points")
    print("=" * 70)

    # Get manual volumes
    manual_volumes = get_manual_volumes_example()
    if len(manual_volumes) != Config.NUM_VOLUMES:
        print(f"  Note: Config.NUM_VOLUMES={Config.NUM_VOLUMES} but manual list has {len(manual_volumes)} volumes.")

    # Reset and load model
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(Config.INPUT_VSP)
    vsp.Update()

    # Find fuselage
    geom_ids = vsp.FindGeoms()
    fuse_id = None
    for gid in geom_ids:
        name = vsp.GetGeomName(gid)
        if "fuselage" in name.lower() or "fuse" in name.lower():
            fuse_id = gid
            break

    if fuse_id is None:
        raise RuntimeError("X No fuselage found in model!")

    print(f"  Found fuselage: {vsp.GetGeomName(fuse_id)}")

    # Analyze fuselage geometry using REAL surface sampling
    bounds = analyze_fuselage_geometry_real(fuse_id)

    print(f"  ✓ Analyzed fuselage: L={bounds.length:.1f} (X range {bounds.x_min:.2f}..{bounds.x_max:.2f})")
    print(f"  ✓ Real cross-sections sampled: {len(bounds.cross_sections)}")

    # Place manual volumes (validate & skip invalid)
    volumes = place_manual_volumes(fuse_id, bounds, manual_volumes)

    # Save configuration (only placed volumes) with auto-numbering
    conf_num = save_configuration(volumes, bounds, Config.OUTPUT_DIR)

    print("\n" + "=" * 70)
    print(" ✓ TASK COMPLETED - SINGLE CONFIGURATION GENERATED")
    print(f"   → Configuration: conf{conf_num}")
    print(f"   → Output directory: {Config.OUTPUT_DIR}/conf{conf_num}/")
    print(f"   → Valid volumes placed: {len(volumes)}/{len(manual_volumes)}")
    print(f"   → Contains: VSP3 + CSV + JSON + Constraints CSV with REAL surface data")
    print("=" * 70)


if __name__ == "__main__":
    main()