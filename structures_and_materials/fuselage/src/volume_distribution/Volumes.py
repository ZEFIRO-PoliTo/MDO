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
    INPUT_VSP = "input_fuse.vsp3"  # Input fuselage file
    OUTPUT_DIR = "configs"  # Main output directory
    NUM_CONFIGS = 10  # Number of configurations to generate
    NUM_VOLUMES = 10  # Number of volumes per configuration

    # Ellipsoid size parameters (relative to fuselage radius)
    ELLIPSOID_SIZE_MIN = 0.2  # Minimum size factor
    ELLIPSOID_SIZE_MAX = 0.4  # Maximum size factor

    # Fuselage dimensions (default values if not detected)
    DEFAULT_RADIUS_Y = 1.5  # Default Y radius
    DEFAULT_RADIUS_Z = 1.5  # Default Z radius

    # Placement parameters
    SAFETY_MARGIN = 0.85  # Keep volumes within 85% of radius
    LONGITUDINAL_MARGIN = 0.1  # Margin from fuselage ends (10% of length)


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
class FuselageBounds:
    """Represents the bounding box and shape of the fuselage"""
    length: float
    radius_y: float
    radius_z: float
    x_min: float
    x_max: float

    def get_radius_at_x(self, x_norm: float) -> Tuple[float, float]:
        """
        Get effective radius at normalized x position (0 to 1)
        Accounts for fuselage taper at nose and tail
        """
        # Simple taper function (can be refined based on actual fuselage shape)
        if x_norm < 0.15:  # Nose taper
            taper = 0.3 + 0.7 * (x_norm / 0.15)
        elif x_norm > 0.85:  # Tail taper
            taper = 0.3 + 0.7 * ((1.0 - x_norm) / 0.15)
        else:  # Cylindrical section
            taper = 1.0

        return self.radius_y * taper, self.radius_z * taper


# ====================
# Core Functions
# ====================
'''def create_simple_fuselage(length: float = 20.0,
                           radius: float = 1.5,
                           output_file: str = "input_fuse.vsp3"):
    """
    Create a simple fuselage for testing

    Args:
        length: Fuselage length
        radius: Fuselage radius (circular cross-section)
        output_file: Output file path
    """
    print("🔨 Creating base fuselage...")

    vsp.ClearVSPModel()
    fuse_id = vsp.AddGeom("FUSELAGE", "")

    # Set dimensions
    vsp.SetParmValUpdate(fuse_id, "Length", "Design", length)

    # Position at origin
    vsp.SetParmValUpdate(fuse_id, "X_Location", "XForm", 0.0)
    vsp.SetParmValUpdate(fuse_id, "Y_Location", "XForm", 0.0)
    vsp.SetParmValUpdate(fuse_id, "Z_Location", "XForm", 0.0)

    vsp.Update()
    vsp.WriteVSPFile(output_file, vsp.SET_ALL)

    print(f"✅ Fuselage created: {output_file}")
    print(f"   Length: {length}m, Approx. radius: {radius}m")'''


def get_fuselage_bounds(fuse_id: str) -> FuselageBounds:
    """
    Extract fuselage dimensional information

    Args:
        fuse_id: OpenVSP geometry ID of fuselage

    Returns:
        FuselageBounds object
    """
    # Get fuselage length
    length = vsp.GetParmVal(fuse_id, "Length", "Design")

    # Get position
    x_pos = vsp.GetParmVal(fuse_id, "X_Location", "XForm")

    # Use default radii (can be refined based on actual cross-sections)
    radius_y = Config.DEFAULT_RADIUS_Y
    radius_z = Config.DEFAULT_RADIUS_Z

    return FuselageBounds(
        length=length,
        radius_y=radius_y,
        radius_z=radius_z,
        x_min=x_pos,
        x_max=x_pos + length
    )


def is_ellipsoid_inside_fuselage(volume: EllipsoidVolume,
                                 bounds: FuselageBounds,
                                 safety_factor: float = 0.85) -> bool:
    """
    Check if an ellipsoid is fully contained within the fuselage

    Args:
        volume: EllipsoidVolume object
        bounds: FuselageBounds object
        safety_factor: Safety margin (0-1)

    Returns:
        True if ellipsoid is inside, False otherwise
    """
    # Check longitudinal bounds
    if (volume.x - volume.radius_x < bounds.x_min or
            volume.x + volume.radius_x > bounds.x_max):
        return False

    # Get effective radius at this x position
    x_norm = (volume.x - bounds.x_min) / bounds.length
    eff_radius_y, eff_radius_z = bounds.get_radius_at_x(x_norm)

    # Apply safety factor
    eff_radius_y *= safety_factor
    eff_radius_z *= safety_factor

    # Check if ellipsoid center + radius is within bounds
    if (abs(volume.y) + volume.radius_y > eff_radius_y or
            abs(volume.z) + volume.radius_z > eff_radius_z):
        return False

    return True


def generate_ellipsoid_position(bounds: FuselageBounds,
                                volume_size: float,
                                safety_margin: float = 0.85,
                                max_attempts: int = 100) -> Optional[Tuple[float, float, float]]:
    """
    Generate a valid position for an ellipsoid inside the fuselage,
    considering taper near nose/tail and applying extra safety margin.

    Args:
        bounds: FuselageBounds object
        volume_size: Size of the ellipsoid (radius)
        safety_margin: Base safety factor for placement
        max_attempts: Maximum attempts to find valid position

    Returns:
        Tuple of (x, y, z) coordinates or None if failed
    """

    extra_margin_factor = 0.9  # Additional safety margin (push ellipsoids further inside)

    for _ in range(max_attempts):
        # Base longitudinal margin
        margin_x = Config.LONGITUDINAL_MARGIN * bounds.length
        x_min_safe = bounds.x_min + margin_x + volume_size
        x_max_safe = bounds.x_max - margin_x - volume_size

        # Adjust x_max_safe based on taper at tail
        x_norm_max = (x_max_safe - bounds.x_min) / bounds.length
        eff_radius_y_max, eff_radius_z_max = bounds.get_radius_at_x(x_norm_max)
        x_max_safe -= max(0.0, float(volume_size - eff_radius_y_max), float(volume_size - eff_radius_z_max))

        # Random x position
        x = random.uniform(x_min_safe, x_max_safe)

        # Effective radius at this x
        x_norm = (x - bounds.x_min) / bounds.length
        eff_radius_y, eff_radius_z = bounds.get_radius_at_x(x_norm)

        # Apply safety margin + extra margin
        safe_radius_y = (eff_radius_y * safety_margin * extra_margin_factor) - volume_size
        safe_radius_z = (eff_radius_z * safety_margin * extra_margin_factor) - volume_size

        if safe_radius_y > 0 and safe_radius_z > 0:
            # Random position within safe bounds
            angle = random.uniform(0, 2 * math.pi)
            r = random.uniform(0, 1)

            y = r * safe_radius_y * math.cos(angle)
            z = r * safe_radius_z * math.sin(angle)

            return x, y, z

    return None




def place_ellipsoid_volumes(fuse_id: str,
                            bounds: FuselageBounds,
                            num_volumes: int = 10) -> List[EllipsoidVolume]:
    """
    Place multiple ellipsoid volumes inside the fuselage

    Args:
        fuse_id: Fuselage geometry ID
        bounds: FuselageBounds object
        num_volumes: Number of volumes to place

    Returns:
        List of placed EllipsoidVolume objects
    """
    volumes = []

    for i in range(num_volumes):
        # Random size for this ellipsoid
        size_factor = random.uniform(Config.ELLIPSOID_SIZE_MIN,
                                     Config.ELLIPSOID_SIZE_MAX)

        # Calculate actual size based on fuselage dimensions
        base_size = min(bounds.radius_y, bounds.radius_z)
        ellipsoid_radius = base_size * size_factor

        # Generate position
        position = generate_ellipsoid_position(
            bounds,
            ellipsoid_radius,
            Config.SAFETY_MARGIN
        )

        if position is None:
            print(f"  ⚠️ Could not place volume {i + 1}, using fallback position")
            # Fallback to center position
            x = bounds.x_min + bounds.length * 0.5
            y, z = 0.0, 0.0
        else:
            x, y, z = position

        # Create ellipsoid in OpenVSP
        ellipsoid_id = vsp.AddGeom("ELLIPSOID", "")

        # Set to absolute coordinates
        vsp.SetParmValUpdate(ellipsoid_id, "Abs_Or_Relitive_flag", "XForm", 0)
        vsp.SetParmValUpdate(ellipsoid_id, "Trans_Attach_Flag", "Attach", 0)

        # Set position
        vsp.SetParmValUpdate(ellipsoid_id, "X_Location", "XForm", x)
        vsp.SetParmValUpdate(ellipsoid_id, "Y_Location", "XForm", y)
        vsp.SetParmValUpdate(ellipsoid_id, "Z_Location", "XForm", z)

        # Set size (uniform scaling for simplicity)
        # Note: Actual parameter names may vary
        try:
            vsp.SetParmValUpdate(ellipsoid_id, "A_Radius", "Design", ellipsoid_radius)
            vsp.SetParmValUpdate(ellipsoid_id, "B_Radius", "Design", ellipsoid_radius)
            vsp.SetParmValUpdate(ellipsoid_id, "C_Radius", "Design", ellipsoid_radius)
        except:
            pass

        vsp.Update()

        # Store volume data
        volume = EllipsoidVolume(
            id=i + 1,
            x=x,
            y=y,
            z=z,
            radius_x=ellipsoid_radius,
            radius_y=ellipsoid_radius,
            radius_z=ellipsoid_radius
        )
        volumes.append(volume)

        print(f"  ✅ Volume {i + 1} → ({x:.2f}, {y:.2f}, {z:.2f}) | Size: {ellipsoid_radius:.3f}")

    return volumes


def save_configuration(conf_num: int,
                       volumes: List[EllipsoidVolume],
                       output_dir: str):
    """
    Save configuration files (VSP3, CSV, JSON)

    Args:
        conf_num: Configuration number
        volumes: List of EllipsoidVolume objects
        output_dir: Output directory path
    """
    # Create configuration directory
    conf_dir = os.path.join(output_dir, f"conf{conf_num}")
    os.makedirs(conf_dir, exist_ok=True)

    # Save VSP3 file
    vsp_file = os.path.join(conf_dir, f"fuse_conf{conf_num}.vsp3")
    vsp.WriteVSPFile(vsp_file, vsp.SET_ALL)
    print(f"💾 Saved VSP3: {vsp_file}")

    # Save CSV file
    csv_file = os.path.join(conf_dir, "volumes_positions.csv")
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Volume_ID", "X", "Y", "Z", "Radius_X", "Radius_Y", "Radius_Z"])
        for vol in volumes:
            writer.writerow(vol.to_list())
    print(f"💾 Saved CSV: {csv_file}")

    # Save JSON file for additional metadata
    json_file = os.path.join(conf_dir, "volumes_data.json")
    config_data = {
        "configuration": conf_num,
        "num_volumes": len(volumes),
        "volumes": [vol.to_dict() for vol in volumes]
    }
    with open(json_file, "w") as f:
        json.dump(config_data, f, indent=2)
    print(f"💾 Saved JSON: {json_file}")


# ====================
# Main Execution
# ====================
def main():
    """Main execution function"""

    print("=" * 60)
    print("OpenVSP Volume Placement System")
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
            raise RuntimeError("❌ No fuselage found in model!")

        print(f"  Found fuselage: {vsp.GetGeomName(fuse_id)}")

        # Get fuselage bounds
        bounds = get_fuselage_bounds(fuse_id)
        print(f"  Fuselage: L={bounds.length:.1f}m, R_y={bounds.radius_y:.1f}m, R_z={bounds.radius_z:.1f}m")

        # Place volumes
        volumes = place_ellipsoid_volumes(fuse_id, bounds, Config.NUM_VOLUMES)

        # Save configuration
        save_configuration(conf, volumes, Config.OUTPUT_DIR)

    print("\n" + "=" * 60)
    print(f"✅ TASK COMPLETED!")
    print(f"   → {Config.NUM_CONFIGS} configurations generated")
    print(f"   → Output directory: {Config.OUTPUT_DIR}/")
    print(f"   → Each config contains: VSP3 file + CSV positions + JSON metadata")
    print("=" * 60)


if __name__ == "__main__":
    main()
