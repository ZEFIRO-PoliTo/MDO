"""
integrator.py
Integrates fuselage geometry with internal volume placement and collision checking.
Adds placed volumes into the OpenVSP model as ELLIPSOIDs (safe, consistent parameters).
"""

import csv
import os
import random
import openvsp as vsp

from shapes import Box   # used for in-memory volume shapes / placement
from collision import check_collision
from placed_volume import analyze_fuselage_geometry, FuselageBounds

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
CSV_PATH = "../fusolage_iterator/fuselage_batch_log.csv"
FUSELAGE_DIR = "../fusolage_iterator"
NUM_INTERNAL_VOLUMES = 5  # same as before (adjust as you like)


# ------------------------------------------------------------
# Utility: safe set parm (only if parm exists)
# ------------------------------------------------------------
def safe_set_parm(geom_id: int, parm_name: str, group: str, value) -> bool:
    """
    Attempt to set a parm only if it exists. Return True if set, False otherwise.
    This prevents 'Can't Find Parm' errors (code 4).
    """
    try:
        pid = vsp.FindParm(geom_id, parm_name, group)
    except Exception:
        # Some OpenVSP Python wrappers may raise; treat as not-found
        pid = -1

    if pid is None:
        pid = -1

    if pid != -1 and pid >= 0:
        try:
            vsp.SetParmValUpdate(geom_id, parm_name, group, value)
            return True
        except Exception:
            return False
    return False


# ------------------------------------------------------------
# Add an ellipsoid representing the volume (safe and consistent)
# ------------------------------------------------------------
def add_ellipsoid_for_volume(vol, attach_to_geom: int = None) -> int:
    """
    Add an ELLIPSOID to the OpenVSP model and set location + radii.
    Returns the new geom id, or -1 if something failed.
    """
    # Add ellipsoid (use empty parent; if you prefer attach to fuselage, pass attach_to_geom)
    try:
        ell_id = vsp.AddGeom("ELLIPSOID", "")
    except Exception as e:
        print(f"[ERROR] AddGeom(ELLIPSOID) failed: {e}")
        return -1

    # Some OpenVSP builds expect Update() before parms are accessible
    vsp.Update()

    # Attach flags if present (safe calls)
    safe_set_parm(ell_id, "Abs_Or_Relitive_flag", "XForm", 0)
    safe_set_parm(ell_id, "Trans_Attach_Flag", "Attach", 0)

    # Location - prefer X_Location / Y_Location / Z_Location with group "XForm"
    safe_set_parm(ell_id, "X_Location", "XForm", vol.cx)
    safe_set_parm(ell_id, "Y_Location", "XForm", vol.cy)
    safe_set_parm(ell_id, "Z_Location", "XForm", vol.cz)

    # Radii (A_Radius, B_Radius, C_Radius) in 'Design' group
    # Use half-dimensions from the Box: lx, ly, lz are full lengths -> radii = half
    a_r = (vol.lx / 2.0) if hasattr(vol, "lx") else (vol.radius_x if hasattr(vol, "radius_x") else 0.1)
    b_r = (vol.ly / 2.0) if hasattr(vol, "ly") else (vol.radius_y if hasattr(vol, "radius_y") else 0.1)
    c_r = (vol.lz / 2.0) if hasattr(vol, "lz") else (vol.radius_z if hasattr(vol, "radius_z") else 0.1)

    safe_set_parm(ell_id, "A_Radius", "Design", a_r)
    safe_set_parm(ell_id, "B_Radius", "Design", b_r)
    safe_set_parm(ell_id, "C_Radius", "Design", c_r)

    # Optional: give it a descriptive name if supported
    try:
        vsp.SetGeomName(ell_id, f"Volume_{vol.cx:.2f}_{vol.cy:.2f}_{vol.cz:.2f}")
    except Exception:
        pass

    vsp.Update()
    return ell_id


# ------------------------------------------------------------
# Process a single fuselage file
# ------------------------------------------------------------
def process_fuselage(vsp_file: str):
    """Load fuselage, place volumes, check collisions, and export modified model."""
    try:
        # Load model
        vsp.ClearVSPModel()
        vsp.ReadVSPFile(vsp_file)
        vsp.Update()

        # Find fuselage geometry
        geoms = vsp.FindGeoms()
        fuse_id = None
        for gid in geoms:
            name = vsp.GetGeomName(gid)
            if "fuse" in name.lower():
                fuse_id = gid
                break

        if fuse_id is None:
            print(f"[FAILURE] {vsp_file}: No fuselage found!")
            return

        # Get fuselage bounding box and cross-section data
        bounds = analyze_fuselage_geometry(fuse_id)

        # Generate internal volumes (in-memory Box shapes)
        volumes = []
        for i in range(NUM_INTERNAL_VOLUMES):
            x = random.uniform(bounds.x_min, bounds.x_max)
            # Choose Y/Z sampling inside local effective radius (conservative)
            # Keep within reasonable bounds so we don't place outside fuselage
            # (these numbers are intentionally small — adjust for your model)
            y = random.uniform(-0.5, 0.5)
            z = random.uniform(-0.5, 0.5)
            lx, ly, lz = 0.2, 0.2, 0.2
            volumes.append(Box(x, y, z, lx, ly, lz))

        # Check collisions among the placed volumes (in-memory)
        collides = check_collision(volumes)
        print(f"[INFO] {vsp_file}: Collision detected = {collides}")

        # Add volumes into the OpenVSP model safely (use ellipsoids to avoid param mismatch)
        added_ids = []
        for v in volumes:
            gid = add_ellipsoid_for_volume(v, attach_to_geom=fuse_id)
            if gid != -1:
                added_ids.append(gid)

        # Finalize and save new file
        vsp.Update()
        out_path = vsp_file.replace(".vsp3", "_with_volumes.vsp3")
        vsp.WriteVSPFile(out_path)

        print(f"[SUCCESS] Saved modified fuselage: {out_path}")

    except Exception as e:
        print(f"[ERROR] {vsp_file}: {e}")


# ------------------------------------------------------------
# Main script
# ------------------------------------------------------------
def main():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(f"CSV not found at {CSV_PATH}")

    print(f"Reading fuselage list from {CSV_PATH}...\n")

    with open(CSV_PATH, newline="") as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # skip header line
        for row in reader:
            if len(row) < 1:
                continue  # skip malformed lines
            filename = row[0].strip()
            vsp_file = os.path.join(FUSELAGE_DIR, filename)
            if not os.path.exists(vsp_file):
                print(f"[SKIPPED] {vsp_file} not found.")
                continue

            process_fuselage(vsp_file)


if __name__ == "__main__":
    main()
