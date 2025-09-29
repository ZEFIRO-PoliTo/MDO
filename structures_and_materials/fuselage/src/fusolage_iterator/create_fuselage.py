from openvsp import vsp
import random
import math


def place_volumes(fid, n=10):
    """
    Place n ellipsoid volumes inside the fuselage with random positions.

    Args:
        fid (str): Fuselage geometry ID
        n (int): Number of ellipsoid volumes to place

    Returns:
        list of (x, y, z): Positions of the placed ellipsoids
    """
    positions = []
    for i in range(n):
        # Example fuselage bounds (relative random placement)
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(-1.0, 1.0)

        vol_id = vsp.AddGeom("ELLIPSOID", fid)

        vsp.SetParmValUpdate(vol_id, "XLoc", "XForm", x)
        vsp.SetParmValUpdate(vol_id, "YLoc", "XForm", y)
        vsp.SetParmValUpdate(vol_id, "ZLoc", "XForm", z)

        positions.append((x, y, z))

    vsp.Update()
    return positions


def check_collisions(volume_positions, min_dist=0.1):
    """
    Check for pairwise collisions between volumes using bounding sphere approximation.

    Args:
        volume_positions (list of (x, y, z)): Volume positions
        min_dist (float): Minimum allowed distance between centers

    Returns:
        bool: True if no collisions found, False otherwise
    """
    n = len(volume_positions)
    valid = True
    for i in range(n):
        for j in range(i + 1, n):
            x1, y1, z1 = volume_positions[i]
            x2, y2, z2 = volume_positions[j]
            dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
            if dist < min_dist:
                print(f"[Warning] Volumes {i} and {j} are too close (distance={dist:.3f})")
                valid = False
    return valid


def create_fuselage(length, width, height, filename="fuselage.vsp3"):
    """
    Create fuselage, place volumes, and check collisions.
    """
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")

    # Set parameters (real OpenVSP API call)
    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.SetParmValUpdate(fid, "Diameter", "Design", width)   # fuselage width ≈ diameter
    vsp.SetParmValUpdate(fid, "Height", "Design", height)    # only if fuselage supports it

    vsp.Update()

    # Place volumes
    positions = place_volumes(fid, n=10)

    # Check for collisions
    valid = check_collisions(positions, min_dist=0.1)

    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage with volumes saved to {filename}")

    if valid:
        print("All volumes placed without overlaps.")
    else:
        print("Some volumes are overlapping or too close.")


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
