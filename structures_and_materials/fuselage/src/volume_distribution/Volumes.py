import openvsp as vsp
import random
import csv
import json
import os
import math
from typing import List, Tuple, Optional
from dataclasses import dataclass, asdict


# ====================
# Configuration
# ====================
class Config:
    """Configuration parameters for volume placement"""
    
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    INPUT_VSP = os.path.join(BASE_DIR, "input_fuse.vsp3")
    OUTPUT_DIR = os.path.join(BASE_DIR, "configs")
    NUM_CONFIGS = 10
    NUM_VOLUMES = 10

    # Ellipsoid size parameters (relative to local fuselage radius)
    ELLIPSOID_SIZE_MIN = 0.15  # Minimum size factor
    ELLIPSOID_SIZE_MAX = 0.35  # Maximum size factor

    # Placement parameters
    SAFETY_MARGIN = 0.8  # Keep volumes within 80% of local radius
    LONGITUDINAL_MARGIN = 0.05  # Margin from fuselage ends (5% of length)
    
    # Surface sampling parameters
    U_SAMPLES = 100  # Longitudinal samples (along fuselage length)
    W_SAMPLES = 36   # Circumferential samples (around fuselage)


# ====================
# Data Classes
# ====================
@dataclass
class EllipsoidVolume:
    """Represents a single ellipsoid volume with its properties"""
    id: int
    x: float
    y: float
    z: float
    radius_x: float
    radius_y: float
    radius_z: float

    def to_list(self):
        """Convert to list for CSV export"""
        return [self.id, self.x, self.y, self.z,
                self.radius_x, self.radius_y, self.radius_z]

    def to_dict(self):
        """Convert to dictionary for JSON export"""
        return asdict(self)


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
                cs_prev = self.cross_sections[i-1]
                t = (u - cs_prev.u) / (cs.u - cs_prev.u) if cs.u != cs_prev.u else 0
                
                radius_y = cs_prev.radius_y + t * (cs.radius_y - cs_prev.radius_y)
                radius_z = cs_prev.radius_z + t * (cs.radius_z - cs_prev.radius_z)
                center_y = cs_prev.center_y + t * (cs.center_y - cs_prev.center_y)
                center_z = cs_prev.center_z + t * (cs.center_z - cs_prev.center_z)
                
                return radius_y, radius_z, center_y, center_z
        
        # If u is beyond last cross-section
        last = self.cross_sections[-1]
        return last.radius_y, last.radius_z, last.center_y, last.center_z


def analyze_fuselage_geometry_real(fuse_id: str) -> FuselageBounds:
    """
    Analyze fuselage geometry using OpenVSP surface query functions
    to get REAL geometry data from the actual surface
    
    Args:
        fuse_id: OpenVSP geometry ID of fuselage
    
    Returns:
        FuselageBounds object with actual surface-sampled cross-sectional data
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
            except:
                # Fallback if CompPnt01 fails
                print(f"      Warning: CompPnt01 failed at u={u:.3f}, w={w:.3f}")
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
        
        # Use average of maximum extents for radius
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


def x_to_u(x: float, bounds: FuselageBounds) -> float:
    """Convert X coordinate to u parameter (0-1)"""
    if bounds.length == 0:
        return 0.5
    return (x - bounds.x_min) / bounds.length


def u_to_x(u: float, bounds: FuselageBounds) -> float:
    """Convert u parameter (0-1) to X coordinate"""
    return bounds.x_min + u * bounds.length


def is_ellipsoid_inside_fuselage(volume: EllipsoidVolume,
                                 bounds: FuselageBounds,
                                 safety_factor: float = 0.8,
                                 debug: bool = False) -> bool:
    """
    Check if an ellipsoid is fully contained within the fuselage
    using REAL cross-sectional data from surface sampling
    """
    # Check longitudinal bounds
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


def generate_ellipsoid_position(bounds: FuselageBounds,
                                volume_size: float,
                                safety_margin: float = 0.8,
                                max_attempts: int = 100) -> Optional[Tuple[float, float, float]]:
    """
    Generate a valid position for an ellipsoid inside the fuselage
    using REAL cross-sectional data from surface sampling
    """
    for _ in range(max_attempts):
        # Random u position with margin
        margin_u = Config.LONGITUDINAL_MARGIN
        u = random.uniform(margin_u, 1.0 - margin_u)
        
        # Convert to X coordinate
        x = u_to_x(u, bounds)
        
        # Get REAL effective radius at this u from actual surface data
        eff_radius_y, eff_radius_z, center_y, center_z = bounds.get_radius_at_u(u)
        
        # Apply safety margin and account for volume size
        safe_radius_y = eff_radius_y * safety_margin - volume_size
        safe_radius_z = eff_radius_z * safety_margin - volume_size
        
        if safe_radius_y > 0 and safe_radius_z > 0:
            # Generate random position within elliptical cross-section
            # relative to the REAL center of this cross-section
            angle = random.uniform(0, 2 * math.pi)
            r = math.sqrt(random.uniform(0, 1))  # Uniform distribution in ellipse
            
            y = center_y + r * safe_radius_y * math.cos(angle)
            z = center_z + r * safe_radius_z * math.sin(angle)
            
            return x, y, z
    
    return None


def volumes_collide(v1: EllipsoidVolume, v2: EllipsoidVolume, margin: float = 1.1) -> bool:
    """
    Check if two ellipsoid volumes collide with safety margin
    """
    dx = v1.x - v2.x
    dy = v1.y - v2.y
    dz = v1.z - v2.z
    dist_sq = dx*dx + dy*dy + dz*dz
    
    # Use average radius for collision detection
    r1 = (v1.radius_x + v1.radius_y + v1.radius_z) / 3
    r2 = (v2.radius_x + v2.radius_y + v2.radius_z) / 3
    min_dist = (r1 + r2) * margin
    
    return dist_sq < (min_dist * min_dist)


def is_valid_position(new_volume: EllipsoidVolume, 
                      placed_volumes: List[EllipsoidVolume],
                      bounds: FuselageBounds,
                      debug: bool = False) -> bool:
    """Check if new volume is valid (inside fuselage and no collisions)"""
    # Check if inside fuselage using REAL surface data
    if not is_ellipsoid_inside_fuselage(new_volume, bounds, Config.SAFETY_MARGIN, debug):
        return False
    
    # Check collisions with existing volumes
    for vol in placed_volumes:
        if volumes_collide(new_volume, vol):
            if debug:
                print(f"    DEBUG: Volume {new_volume.id} COLLISION with Volume {vol.id}!")
                dx = new_volume.x - vol.x
                dy = new_volume.y - vol.y
                dz = new_volume.z - vol.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                min_required = ((new_volume.radius_x + vol.radius_x) * 1.1)
                print(f"           Distance: {distance:.2f}, Required: {min_required:.2f}")
            return False
    
    return True


def place_ellipsoid_volumes(fuse_id: str,
                            bounds: FuselageBounds,
                            num_volumes: int = 10,
                            max_attempts: int = 500) -> List[EllipsoidVolume]:
    """
    Place multiple ellipsoid volumes inside the fuselage using REAL geometry
    """
    volumes = []
    
    print(f"  Placing {num_volumes} volumes using REAL surface geometry...")
    
    for i in range(num_volumes):
        placed = False
        
        for attempt in range(max_attempts):
            # Random u position to determine local size constraints
            u_test = random.uniform(0, 1)
            local_radius_y, local_radius_z, _, _ = bounds.get_radius_at_u(u_test)
            max_local_radius = min(local_radius_y, local_radius_z)
            
            # Size based on REAL local constraints
            size_factor = random.uniform(Config.ELLIPSOID_SIZE_MIN, Config.ELLIPSOID_SIZE_MAX)
            ellipsoid_radius = max_local_radius * size_factor
            
            # Generate position using REAL surface data
            position = generate_ellipsoid_position(
                bounds,
                ellipsoid_radius,
                Config.SAFETY_MARGIN
            )
            
            if position is None:
                continue
            
            x, y, z = position
            
            new_vol = EllipsoidVolume(
                id=i + 1,
                x=x,
                y=y,
                z=z,
                radius_x=ellipsoid_radius,
                radius_y=ellipsoid_radius,
                radius_z=ellipsoid_radius
            )
            
            # Validate position against REAL surface geometry
            if is_valid_position(new_vol, volumes, bounds, debug=False):
                volumes.append(new_vol)
                
                # Create ellipsoid in OpenVSP
                ellipsoid_id = vsp.AddGeom("ELLIPSOID", "")
                vsp.SetParmValUpdate(ellipsoid_id, "Abs_Or_Relitive_flag", "XForm", 0)
                vsp.SetParmValUpdate(ellipsoid_id, "Trans_Attach_Flag", "Attach", 0)
                vsp.SetParmValUpdate(ellipsoid_id, "X_Location", "XForm", x)
                vsp.SetParmValUpdate(ellipsoid_id, "Y_Location", "XForm", y)
                vsp.SetParmValUpdate(ellipsoid_id, "Z_Location", "XForm", z)
                
                try:
                    vsp.SetParmValUpdate(ellipsoid_id, "A_Radius", "Design", ellipsoid_radius)
                    vsp.SetParmValUpdate(ellipsoid_id, "B_Radius", "Design", ellipsoid_radius)
                    vsp.SetParmValUpdate(ellipsoid_id, "C_Radius", "Design", ellipsoid_radius)
                except:
                    pass
                
                vsp.Update()
                
                u = x_to_u(x, bounds)
                print(f"    Volume {i+1} placed after {attempt+1} attempts → "
                      f"X={x:.2f} (u={u:.3f}), Y={y:.2f}, Z={z:.2f} | R={ellipsoid_radius:.3f}")
                
                # Verify placement with REAL surface check
                if not is_ellipsoid_inside_fuselage(new_vol, bounds, Config.SAFETY_MARGIN, debug=False):
                    print(f"    WARNING: Volume {i+1} verification failed!")
                    is_ellipsoid_inside_fuselage(new_vol, bounds, Config.SAFETY_MARGIN, debug=True)
                else:
                    print(f"    ✓ Volume {i+1} verified using REAL surface geometry")
                
                placed = True
                break
        
        if not placed:
            print(f"    X Could not place volume {i+1} after {max_attempts} attempts")
    
    return volumes


def save_configuration(conf_num: int,
                       volumes: List[EllipsoidVolume],
                       bounds: FuselageBounds,
                       output_dir: str):
    """
    Save configuration files with enhanced metadata
    """
    conf_dir = os.path.join(output_dir, f"conf{conf_num}")
    os.makedirs(conf_dir, exist_ok=True)
    
    # Save VSP3 file
    vsp_file = os.path.join(conf_dir, f"fuse_conf{conf_num}.vsp3")
    vsp.WriteVSPFile(vsp_file, vsp.SET_ALL)
    print(f"  Saved VSP3: {vsp_file}")
    
    # Save CSV file
    csv_file = os.path.join(conf_dir, "volumes_positions.csv")
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Volume_ID", "X", "Y", "Z", "Radius_X", "Radius_Y", "Radius_Z"])
        for vol in volumes:
            writer.writerow(vol.to_list())
    print(f"  Saved CSV: {csv_file}")
    
    # Save enhanced JSON file with REAL surface data
    json_file = os.path.join(conf_dir, "volumes_data.json")
    config_data = {
        "configuration": conf_num,
        "num_volumes": len(volumes),
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


# ====================
# Main Execution
# ====================
def main():
    """Main execution function"""
    
    print("=" * 70)
    print("OpenVSP Volume Placement System - REAL SURFACE GEOMETRY")
    print("=" * 70)
    print("Using vsp.CompPnt01() to sample actual surface points")
    print("=" * 70)
    
    # Create output directory
    os.makedirs(Config.OUTPUT_DIR, exist_ok=True)
    
    # Generate configurations
    for conf in range(1, Config.NUM_CONFIGS + 1):
        print(f"\n🔹 Generating configuration {conf}/{Config.NUM_CONFIGS}...")
        
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
        
        print(f"  ✓ Analyzed fuselage: L={bounds.length:.1f}m")
        print(f"  ✓ Real cross-sections sampled: {len(bounds.cross_sections)}")
        
        # Place volumes using REAL surface data
        volumes = place_ellipsoid_volumes(fuse_id, bounds, Config.NUM_VOLUMES)
        
        # Save configuration
        save_configuration(conf, volumes, bounds, Config.OUTPUT_DIR)
    
    print("\n" + "=" * 70)
    print(" ✓ TASK COMPLETED - USING REAL SURFACE GEOMETRY!")
    print(f"   → {Config.NUM_CONFIGS} configurations generated")
    print(f"   → Using vsp.CompPnt01() for actual surface points")
    print(f"   → {Config.U_SAMPLES} longitudinal × {Config.W_SAMPLES} circumferential samples")
    print(f"   → Output directory: {Config.OUTPUT_DIR}/")
    print(f"   → Each config contains: VSP3 + CSV + JSON with REAL surface data")
    print("=" * 70)


if __name__ == "__main__":
    main()