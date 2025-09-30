from openvsp import vsp
import random
import math

def place_volumes(fid, n=10, length=10, width=2, height=2):
    """
    Place n ellipsoid volumes inside the fuselage with random positions, ensuring they are inside the fuselage bounds.

    Args:
        fid (str): Fuselage geometry ID
        n (int): Number of ellipsoid volumes to place
        length, width, height: Fuselage dimensions

    Returns:
        list of (x, y, z): Positions of the placed ellipsoids
    """
    positions = []
    for i in range(n):
        # Place ellipsoids inside fuselage bounds
        x = random.uniform(-length/2 + 0.1, length/2 - 0.1)
        y = random.uniform(-width/2 + 0.1, width/2 - 0.1)
        z = random.uniform(-height/2 + 0.1, height/2 - 0.1)

        vol_id = vsp.AddGeom("ELLIPSOID", fid)
        vsp.SetParmValUpdate(vol_id, "XLoc", "XForm", x)
        vsp.SetParmValUpdate(vol_id, "YLoc", "XForm", y)
        vsp.SetParmValUpdate(vol_id, "ZLoc", "XForm", z)

        positions.append((x, y, z))

    vsp.Update()
    return positions

def check_collisions(volume_positions, min_dist=0.1, length=10, width=2, height=2):
    """
    Check for pairwise collisions between volumes and for overlap with fuselage boundary.
    Enforces minimum distance constraint between all volume centers.

    Args:
        volume_positions (list of (x, y, z)): Volume positions
        min_dist (float): Minimum allowed distance between centers
        length, width, height: Fuselage dimensions

    Returns:
        bool: True if no collisions found, False otherwise
    """
    n = len(volume_positions)
    valid = True
    # Check volume-to-volume collisions (minimum distance constraint)
    for i in range(n):
        for j in range(i + 1, n):
            x1, y1, z1 = volume_positions[i]
            x2, y2, z2 = volume_positions[j]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
            if dist < min_dist:
                print(f"[Violation] Volumes {i} and {j} are too close (distance={dist:.3f} < min_dist={min_dist})")
                valid = False
    # Check volume-to-fuselage boundary
    for idx, (x, y, z) in enumerate(volume_positions):
        if not (-length/2 <= x <= length/2 and -width/2 <= y <= width/2 and -height/2 <= z <= height/2):
            print(f"[Warning] Volume {idx} is outside fuselage bounds!")
            valid = False
    return valid

def create_fuselage(length, width, height, filename="fuselage.vsp3"):
    """
    Create fuselage, place volumes, and check collisions.
    """
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")

    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.SetParmValUpdate(fid, "Diameter", "Design", width)
    vsp.SetParmValUpdate(fid, "Height", "Design", height)

    vsp.Update()

    # Pass fuselage dimensions to place_volumes and check_collisions
    positions = place_volumes(fid, n=10, length=length, width=width, height=height)
    valid = check_collisions(positions, min_dist=0.1, length=length, width=width, height=height)

    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage with volumes saved to {filename}")

    if valid:
        print("All volumes placed without overlaps and inside fuselage.")
    else:
        print("Some volumes are overlapping, too close, or outside fuselage.")

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
