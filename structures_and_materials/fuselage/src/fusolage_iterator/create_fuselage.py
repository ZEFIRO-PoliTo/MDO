from openvsp import vsp
import random
import math
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

    # Check collisions after optimization
    valid = check_collisions(positions, length=length, width=width, height=height, min_dist=0.1)

    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage saved to {filename}")
    if valid:
        print("Volumes valid inside optimized fuselage.")
    else:
        print("Some volumes invalid after optimization.")


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 4:
        print("Usage: python create_fuselage.py <length> <width> <height> [output_file]")
        sys.exit(1)

    length = float(sys.argv[1])
    width = float(sys.argv[2])
    height = float(sys.argv[3])
    filename = sys.argv[4] if len(sys.argv) > 4 else "fuselage.vsp3"

    create_fuselage(length, width, height, filename)
