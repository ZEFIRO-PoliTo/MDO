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
    XSEC_SAMPLING_DENSITY = 20  # Number of samples along fuselage length


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
    """Represents a cross-section of the fuselage at a specific x-location"""
    x: float
    radius_y: float
    radius_z: float
    area: float


@dataclass
class FuselageBounds:
    """Enhanced fuselage bounds with actual cross-sectional data"""
    length: float
    x_min: float
    x_max: float
    cross_sections: List[CrossSection]
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]

    def get_radius_at_x(self, x: float) -> Tuple[float, float]:
        """
        Get effective radius at specific x position using actual cross-section data
        Uses linear interpolation between known cross-sections
        """
        if not self.cross_sections:
            return 1.0, 1.0
        
        # Clamp x to fuselage bounds
        x = max(self.x_min, min(self.x_max, x))
        
        # Find surrounding cross-sections
        for i, cs in enumerate(self.cross_sections):
            if cs.x >= x:
                if i == 0:
                    return cs.radius_y, cs.radius_z
                
                # Interpolate between cs[i-1] and cs[i]
                cs_prev = self.cross_sections[i-1]
                t = (x - cs_prev.x) / (cs.x - cs_prev.x) if cs.x != cs_prev.x else 0
                
                radius_y = cs_prev.radius_y + t * (cs.radius_y - cs_prev.radius_y)
                radius_z = cs_prev.radius_z + t * (cs.radius_z - cs_prev.radius_z)
                
                return radius_y, radius_z
        
        # If x is beyond last cross-section
        return self.cross_sections[-1].radius_y, self.cross_sections[-1].radius_z


def analyze_fuselage_geometry(fuse_id: str) -> FuselageBounds:
    """
    Analyze fuselage geometry using OpenVSP XSec functions
    
    Args:
        fuse_id: OpenVSP geometry ID of fuselage
    
    Returns:
        FuselageBounds object with actual cross-sectional data
    """
    print("  Analyzing fuselage geometry...")
    
    # Get accurate bounding box
    bbox_min_vec = vsp.GetGeomBBoxMin(fuse_id, 0, True)
    bbox_max_vec = vsp.GetGeomBBoxMax(fuse_id, 0, True)
    
    bbox_min = (bbox_min_vec.x(), bbox_min_vec.y(), bbox_min_vec.z())
    bbox_max = (bbox_max_vec.x(), bbox_max_vec.y(), bbox_max_vec.z())
    
    length = bbox_max[0] - bbox_min[0]
    
    print(f"    Bounding box: min{bbox_min}, max{bbox_max}")
    print(f"    Length: {length:.2f}")
    
    # Get XSecSurfs
    num_xsec_surfs = vsp.GetNumXSecSurfs(fuse_id)
    print(f"    Number of XSec surfaces: {num_xsec_surfs}")
    
    cross_sections = []
    
    if num_xsec_surfs > 0:
        # Get the main XSecSurf (usually index 0 for fuselage)
        xsec_surf_id = vsp.GetXSecSurf(fuse_id, 0)
        num_xsecs = vsp.GetNumXSec(xsec_surf_id)
        
        print(f"    Number of XSecs: {num_xsecs}")
        
        # Sample cross-sections along the fuselage
        for i in range(Config.XSEC_SAMPLING_DENSITY):
            # Normalize position along fuselage
            t = i / (Config.XSEC_SAMPLING_DENSITY - 1) if Config.XSEC_SAMPLING_DENSITY > 1 else 0
            x = bbox_min[0] + t * length
            
            # Get cross-section properties at this location
            radius_y, radius_z = estimate_local_radius(fuse_id, x, bbox_min, bbox_max)
            
            cs = CrossSection(
                x=x,
                radius_y=radius_y,
                radius_z=radius_z,
                area=math.pi * radius_y * radius_z
            )
            cross_sections.append(cs)
    
    else:
        # Fallback: create basic cross-sections if XSec data unavailable
        print("    Warning: No XSec data available, using fallback method")
        for i in range(Config.XSEC_SAMPLING_DENSITY):
            t = i / (Config.XSEC_SAMPLING_DENSITY - 1) if Config.XSEC_SAMPLING_DENSITY > 1 else 0
            x = bbox_min[0] + t * length
            
            # Simple taper estimation based on bounding box
            width = bbox_max[1] - bbox_min[1]
            height = bbox_max[2] - bbox_min[2]
            
            # Apply simple nose/tail taper
            if t < 0.2:
                taper = 0.2 + 0.8 * (t / 0.2)
            elif t > 0.8:
                taper = 0.2 + 0.8 * ((1.0 - t) / 0.2)
            else:
                taper = 1.0
            
            cs = CrossSection(
                x=x,
                radius_y=(width / 2) * taper,
                radius_z=(height / 2) * taper,
                area=math.pi * (width / 2) * (height / 2) * taper * taper
            )
            cross_sections.append(cs)
    
    return FuselageBounds(
        length=length,
        x_min=bbox_min[0],
        x_max=bbox_max[0],
        cross_sections=cross_sections,
        bbox_min=bbox_min,
        bbox_max=bbox_max
    )


def estimate_local_radius(fuse_id: str, x: float, bbox_min: tuple, bbox_max: tuple) -> Tuple[float, float]:
    """
    Estimate local radius at given x position
    This is a simplified approach - in practice, you might want to use
    more sophisticated methods like surface tessellation analysis
    """
    # Get fuselage parameters if available
    try:
        # Try to get fuselage-specific parameters
        length = bbox_max[0] - bbox_min[0]
        width = bbox_max[1] - bbox_min[1]
        height = bbox_max[2] - bbox_min[2]
        
        # Normalize x position
        t = (x - bbox_min[0]) / length if length > 0 else 0
        
        # Apply realistic fuselage taper
        if t < 0.15:  # Nose section
            taper = 0.1 + 0.9 * (t / 0.15)
        elif t > 0.85:  # Tail section
            taper = 0.1 + 0.9 * ((1.0 - t) / 0.15)
        else:  # Main body
            taper = 1.0
        
        radius_y = (width / 2) * taper
        radius_z = (height / 2) * taper
        
        return radius_y, radius_z
    
    except:
        # Fallback values
        return 1.0, 1.0


def is_ellipsoid_inside_fuselage(volume: EllipsoidVolume,
                                 bounds: FuselageBounds,
                                 safety_factor: float = 0.8,
                                 debug: bool = False) -> bool:
    """
    Check if an ellipsoid is fully contained within the fuselage
    using actual cross-sectional data
    """
    # Check longitudinal bounds
    if (volume.x - volume.radius_x < bounds.x_min or
            volume.x + volume.radius_x > bounds.x_max):
        if debug:
            print(f"    DEBUG: Volume {volume.id} LONGITUDINAL OVERFLOW!")
            print(f"           Volume X range: [{volume.x - volume.radius_x:.2f}, {volume.x + volume.radius_x:.2f}]")
            print(f"           Fuselage X range: [{bounds.x_min:.2f}, {bounds.x_max:.2f}]")
        return False

    # Get effective radius at this x position
    eff_radius_y, eff_radius_z = bounds.get_radius_at_x(volume.x)

    # Apply safety factor
    safe_radius_y = eff_radius_y * safety_factor
    safe_radius_z = eff_radius_z * safety_factor

    # Check Y direction
    y_overflow = abs(volume.y) + volume.radius_y > safe_radius_y
    z_overflow = abs(volume.z) + volume.radius_z > safe_radius_z

    if debug and (y_overflow or z_overflow):
        print(f"    DEBUG: Volume {volume.id} CROSS-SECTION OVERFLOW at X={volume.x:.2f}!")
        print(f"           Effective radii: Y={eff_radius_y:.2f}, Z={eff_radius_z:.2f}")
        print(f"           Safe radii (with {safety_factor:.1f} factor): Y={safe_radius_y:.2f}, Z={safe_radius_z:.2f}")
        print(f"           Volume Y extent: center={volume.y:.2f}, radius={volume.radius_y:.2f}, total={abs(volume.y) + volume.radius_y:.2f}")
        print(f"           Volume Z extent: center={volume.z:.2f}, radius={volume.radius_z:.2f}, total={abs(volume.z) + volume.radius_z:.2f}")
        if y_overflow:
            print(f"           Y OVERFLOW: {abs(volume.y) + volume.radius_y:.2f} > {safe_radius_y:.2f}")
        if z_overflow:
            print(f"           Z OVERFLOW: {abs(volume.z) + volume.radius_z:.2f} > {safe_radius_z:.2f}")

    # Check if ellipsoid fits within cross-section
    if y_overflow or z_overflow:
        return False

    return True


def generate_ellipsoid_position(bounds: FuselageBounds,
                                volume_size: float,
                                safety_margin: float = 0.8,
                                max_attempts: int = 100) -> Optional[Tuple[float, float, float]]:
    """
    Generate a valid position for an ellipsoid inside the fuselage
    using actual cross-sectional data
    """
    for _ in range(max_attempts):
        # Longitudinal position with margin
        margin_x = Config.LONGITUDINAL_MARGIN * bounds.length
        x_min_safe = bounds.x_min + margin_x
        x_max_safe = bounds.x_max - margin_x
        
        # Random x position
        x = random.uniform(x_min_safe, x_max_safe)
        
        # Get effective radius at this x
        eff_radius_y, eff_radius_z = bounds.get_radius_at_x(x)
        
        # Apply safety margin
        safe_radius_y = eff_radius_y * safety_margin - volume_size
        safe_radius_z = eff_radius_z * safety_margin - volume_size
        
        if safe_radius_y > 0 and safe_radius_z > 0:
            # Generate random position within elliptical cross-section
            angle = random.uniform(0, 2 * math.pi)
            # Use sqrt for uniform distribution in ellipse
            r = math.sqrt(random.uniform(0, 1))
            
            y = r * safe_radius_y * math.cos(angle)
            z = r * safe_radius_z * math.sin(angle)
            
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
    # Check if inside fuselage
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
                min_required = ((new_volume.radius_x + vol.radius_x) * 1.1)  # 1.1 is the margin from volumes_collide
                print(f"           Distance: {distance:.2f}, Required: {min_required:.2f}")
            return False
    
    return True


def place_ellipsoid_volumes(fuse_id: str,
                            bounds: FuselageBounds,
                            num_volumes: int = 10,
                            max_attempts: int = 500) -> List[EllipsoidVolume]:
    """
    Place multiple ellipsoid volumes inside the fuselage using actual geometry
    """
    volumes = []
    
    print(f"  Placing {num_volumes} volumes...")
    
    for i in range(num_volumes):
        # Variable size based on local fuselage dimensions
        placed = False
        
        for attempt in range(max_attempts):
            # Random longitudinal position to determine local size constraints
            x_test = random.uniform(bounds.x_min, bounds.x_max)
            local_radius_y, local_radius_z = bounds.get_radius_at_x(x_test)
            max_local_radius = min(local_radius_y, local_radius_z)
            
            # Size based on local constraints
            size_factor = random.uniform(Config.ELLIPSOID_SIZE_MIN, Config.ELLIPSOID_SIZE_MAX)
            ellipsoid_radius = max_local_radius * size_factor
            
            # Generate position
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
            
            if is_valid_position(new_vol, volumes, bounds, debug=True):
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
                
                print(f"    Volume {i+1} placed after {attempt+1} attempts → "
                      f"({x:.2f}, {y:.2f}, {z:.2f}) | Size: {ellipsoid_radius:.3f}")
                
                # Verify placement with detailed check
                if not is_ellipsoid_inside_fuselage(new_vol, bounds, Config.SAFETY_MARGIN, debug=False):
                    print(f"    WARNING: Volume {i+1} verification failed - may be outside fuselage!")
                    # Perform detailed debug check
                    is_ellipsoid_inside_fuselage(new_vol, bounds, Config.SAFETY_MARGIN, debug=True)
                else:
                    print(f"    ✓ Volume {i+1} verified as correctly placed inside fuselage")
                
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
    # Create configuration directory
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
    
    # Save enhanced JSON file
    json_file = os.path.join(conf_dir, "volumes_data.json")
    config_data = {
        "configuration": conf_num,
        "num_volumes": len(volumes),
        "fuselage_bounds": {
            "length": bounds.length,
            "x_min": bounds.x_min,
            "x_max": bounds.x_max,
            "bbox_min": bounds.bbox_min,
            "bbox_max": bounds.bbox_max,
            "num_cross_sections": len(bounds.cross_sections)
        },
        "volumes": [vol.to_dict() for vol in volumes],
        "cross_sections": [
            {
                "x": cs.x,
                "radius_y": cs.radius_y,
                "radius_z": cs.radius_z,
                "area": cs.area
            } for cs in bounds.cross_sections
        ]
    }
    with open(json_file, "w") as f:
        json.dump(config_data, f, indent=2)
    print(f"  Saved JSON: {json_file}")


# ====================
# Main Execution
# ====================
def main():
    """Main execution function"""
    
    print("=" * 60)
    print("Enhanced OpenVSP Volume Placement System")
    print("=" * 60)
    
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
        
        # Analyze fuselage geometry using OpenVSP functions
        bounds = analyze_fuselage_geometry(fuse_id)
        
        print(f"  Analyzed fuselage: L={bounds.length:.1f}m")
        print(f"  Cross-sections sampled: {len(bounds.cross_sections)}")
        
        # Place volumes
        volumes = place_ellipsoid_volumes(fuse_id, bounds, Config.NUM_VOLUMES)
        
        # Save configuration
        save_configuration(conf, volumes, bounds, Config.OUTPUT_DIR)
    
    print("\n" + "=" * 60)
    print(f" ENHANCED TASK COMPLETED!")
    print(f"   → {Config.NUM_CONFIGS} configurations generated")
    print(f"   → Using actual fuselage geometry analysis")
    print(f"   → Output directory: {Config.OUTPUT_DIR}/")
    print(f"   → Each config contains: VSP3 + CSV + Enhanced JSON metadata")
    print("=" * 60)


if __name__ == "__main__":
    main()