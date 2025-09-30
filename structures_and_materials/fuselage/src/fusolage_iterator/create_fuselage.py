from openvsp import vsp
import random
import math
import csv
from typing import List, Tuple


def place_volumes(fid: str, length: float, width: float, height: float, n: int = 10, margin: float = 0.05) -> List[Tuple[float, float, float]]:
    """
    Place `n` ellipsoid volumes inside the fuselage using random positions constrained
    by the fuselage bounds.
    """
    positions: List[Tuple[float, float, float]] = []

    half_len = length / 2.0
    yradius = width / 2.0
    zradius = height / 2.0

    x_min = -half_len + margin * length
    x_max = half_len - margin * length

    for i in range(n):
        # sample x uniformly
        x = random.uniform(x_min, x_max)

        # sample a point uniformly within ellipse cross-section
        r = math.sqrt(random.random())
        theta = random.uniform(0, 2 * math.pi)
        y = r * math.cos(theta) * yradius * (1.0 - margin)
        z = r * math.sin(theta) * zradius * (1.0 - margin)

        vol_id = vsp.AddGeom("ELLIPSOID", fid)
        vsp.SetParmValUpdate(vol_id, "XLoc", "XForm", x)
        vsp.SetParmValUpdate(vol_id, "YLoc", "XForm", y)
        vsp.SetParmValUpdate(vol_id, "ZLoc", "XForm", z)

        positions.append((x, y, z))

    vsp.Update()
    return positions


def check_collisions(volume_positions: List[Tuple[float, float, float]],
                     length: float,
                     width: float,
                     height: float,
                     min_dist: float = 0.1) -> bool:
    """
    Check whether each volume is inside fuselage and ensure pairwise spacing > min_dist.
    """
    half_len = length / 2.0
    yradius = width / 2.0
    zradius = height / 2.0

    valid = True
    for idx, (x, y, z) in enumerate(volume_positions):
        # Check fuselage length
        if not (-half_len <= x <= half_len):
            print(f"[Warning] Volume {idx} outside fuselage length at x={x:.3f}")
            valid = False

        # Check ellipse cross-section
        ellipse_val = (y / yradius) ** 2 + (z / zradius) ** 2
        if ellipse_val > 1.0:
            print(f"[Warning] Volume {idx} outside fuselage cross-section (val={ellipse_val:.3f})")
            valid = False

    # Pairwise min distance check
    for i in range(len(volume_positions)):
        for j in range(i + 1, len(volume_positions)):
            x1, y1, z1 = volume_positions[i]
            x2, y2, z2 = volume_positions[j]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
            if dist < min_dist:
                print(f"[Warning] Volumes {i} and {j} too close (d={dist:.3f})")
                valid = False

    # Separation constraints for must-be-apart pairs
    if must_be_apart:
        for i, j in must_be_apart:
            x1, y1, z1 = volume_positions[i]
            x2, y2, z2 = volume_positions[j]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
            if dist < apart_dist:
                print(f"[Constraint] Volumes {i} and {j} must be apart by {apart_dist}, but d={dist:.3f}")
                valid = False

    return valid


def optimize_fuselage(fid: str, positions: List[Tuple[float, float, float]], margin: float = 0.2) -> Tuple[float, float, float]:
    """
    Shrink fuselage to the smallest possible size containing all volumes.
    """
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]

    length = (max(xs) - min(xs)) * (1 + margin)
    width = (max(abs(y) for y in ys)) * 2 * (1 + margin)
    height = (max(abs(z) for z in zs)) * 2 * (1 + margin)

    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.SetParmValUpdate(fid, "Diameter", "Design", width)
    vsp.SetParmValUpdate(fid, "Height", "Design", height)

    vsp.Update()
    print(f"Optimized fuselage -> Length={length:.3f}, Width={width:.3f}, Height={height:.3f}")
    return length, width, height


def create_fuselage(length, width, height, filename="fuselage.vsp3"):
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")

    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.SetParmValUpdate(fid, "Diameter", "Design", width)
    vsp.SetParmValUpdate(fid, "Height", "Design", height)

    vsp.Update()

    positions = place_volumes(fid, length=length, width=width, height=height, n=10)

    # Optimize fuselage around placed volumes 
    length, width, height = optimize_fuselage(fid, positions, margin=0.2)

    # Example: volumes 0 and 1, 2 and 3 must be apart by at least 0.5 units
    must_be_apart = [(0, 1), (2, 3)]
    valid = check_collisions(
        positions,
        length=length,
        width=width,
        height=height,
        min_dist=0.1,
        must_be_apart=must_be_apart,
        apart_dist=0.5
    )

    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage saved to {filename}")
    if valid:
        print("Volumes valid inside optimized fuselage.")
    else:
        print("Some volumes invalid after optimization.")

def batch_generate_fuselages(
    length_range: Tuple[float, float],
    width_range: Tuple[float, float],
    height_range: Tuple[float, float],
    num_samples: int = 10,
    output_prefix: str = "fuselage",
    log_csv: str = "fuselage_batch_log.csv",
    step_size: Tuple[float, float, float] = None
):
    """
    Generate multiple fuselages by sampling parameters within given ranges.
    Log parameters for each fuselage in a CSV file.
    If step_size is provided, use grid sampling instead of random.
    """
    records = []
    if step_size:
        # Grid sampling
        lengths = [length_range[0] + i * step_size[0] for i in range(int((length_range[1]-length_range[0])/step_size[0])+1)]
        widths  = [width_range[0] + i * step_size[1] for i in range(int((width_range[1]-width_range[0])/step_size[1])+1)]
        heights = [height_range[0] + i * step_size[2] for i in range(int((height_range[1]-height_range[0])/step_size[2])+1)]
        idx = 1
        for l in lengths:
            for w in widths:
                for h in heights:
                    filename = f"{output_prefix}_{idx}.vsp3"
                    print(f"\nGenerating fuselage {idx}: Length={l:.2f}, Width={w:.2f}, Height={h:.2f}")
                    create_fuselage(l, w, h, filename)
                    records.append([filename, l, w, h])
                    idx += 1
    else:
        # Random sampling
        for i in range(num_samples):
            length = random.uniform(*length_range)
            width = random.uniform(*width_range)
            height = random.uniform(*height_range)
            filename = f"{output_prefix}_{i+1}.vsp3"
            print(f"\nGenerating fuselage {i+1}: Length={length:.2f}, Width={width:.2f}, Height={height:.2f}")
            create_fuselage(length, width, height, filename)
            records.append([filename, length, width, height])
    # Write log
    with open(log_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["filename", "length", "width", "height"])
        writer.writerows(records)
    print(f"\nBatch log written to {log_csv}")

if __name__ == "__main__":
    # Example usage:
    # python create_fuselage.py --batch
    # python create_fuselage.py --batch --steps 1.0 0.5 0.5
    # python create_fuselage.py <length> <width> <height> [output_file]
    
    if len(sys.argv) >= 2 and sys.argv[1] == "--batch":
        length_range = (10.0, 15.0)
        width_range = (2.0, 4.0)
        height_range = (2.0, 4.0)
        num_samples = 10
        step_size = None
        if "--steps" in sys.argv:
            idx = sys.argv.index("--steps")
            step_size = (float(sys.argv[idx+1]), float(sys.argv[idx+2]), float(sys.argv[idx+3]))
            batch_generate_fuselages(length_range, width_range, height_range, output_prefix="fuselage", log_csv="fuselage_batch_log.csv", step_size=step_size)
        else:
            batch_generate_fuselages(length_range, width_range, height_range, num_samples=num_samples, output_prefix="fuselage", log_csv="fuselage_batch_log.csv")
    elif len(sys.argv) < 4:
        print("Usage: python create_fuselage.py <length> <width> <height> [output_file]")
        print("   or: python create_fuselage.py --batch")
        print("   or: python create_fuselage.py --batch --steps <length_step> <width_step> <height_step>")
        sys.exit(1)
    else:
        length = float(sys.argv[1])
        width = float(sys.argv[2])
        height = float(sys.argv[3])
        filename = sys.argv[4] if len(sys.argv) > 4 else "fuselage.vsp3"
        create_fuselage(length, width, height, filename)
