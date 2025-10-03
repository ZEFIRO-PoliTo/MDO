# integrator.py
"""
Integrator for fuselage + volume placement + collision constraints.

Generates fuselage, places volumes (boxes/ellipsoids), enforces collisions
and minimum distance constraints, and exports to OpenVSP.
"""

import os
import random
import json
import csv
import openvsp as vsp

from shapes import VolumeShape, Box, OrientedBox, Ellipsoid
from collision import FuselageBounds, analyze_fuselage_geometry
from placed_volume import place_volumes

# -------------------------
# Configuration
# -------------------------
class Config:
    INPUT_VSP = os.path.join(os.path.dirname(__file__), "input_fuse.vsp3")
    OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "output")
    NUM_CONFIGS = 5
    NUM_VOLUMES = 10
    SHAPE_TYPES = ["box", "box", "oriented_box", "ellipsoid", "box",
                   "oriented_box", "box", "ellipsoid", "box", "box"]
    CONSTRAINED_PAIRS = [(0, 5), (1, 6), (2, 7), (3, 8), (4, 9)]  # example distance constraints


# -------------------------
# Export helpers
# -------------------------
def export_to_csv(volumes: list, csv_file: str):
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "type", "cx", "cy", "cz", "lx/2rx", "ly/2ry", "lz/2rz"])
        for i, v in enumerate(volumes):
            if isinstance(v, Box) or isinstance(v, OrientedBox):
                writer.writerow([i, type(v).__name__, v.cx, v.cy, v.cz, v.lx, v.ly, v.lz])
            elif isinstance(v, Ellipsoid):
                writer.writerow([i, type(v).__name__, v.cx, v.cy, v.cz, v.rx, v.ry, v.rz])


def export_to_json(volumes: list, json_file: str):
    data = []
    for v in volumes:
        d = v.to_dict()
        d["type"] = type(v).__name__
        data.append(d)
    with open(json_file, "w") as f:
        json.dump(data, f, indent=2)


# -------------------------
# OpenVSP placement
# -------------------------
def place_volumes_in_vsp(volumes: list):
    for i, vol in enumerate(volumes):
        if isinstance(vol, Box) or isinstance(vol, OrientedBox):
            geom_id = vsp.AddGeom("BOX", "")
            vsp.SetParmValUpdate(geom_id, "X_Location", "XForm", vol.cx)
            vsp.SetParmValUpdate(geom_id, "Y_Location", "XForm", vol.cy)
            vsp.SetParmValUpdate(geom_id, "Z_Location", "XForm", vol.cz)
            vsp.SetParmValUpdate(geom_id, "X_Length", "BoxGeom", vol.lx)
            vsp.SetParmValUpdate(geom_id, "Y_Length", "BoxGeom", vol.ly)
            vsp.SetParmValUpdate(geom_id, "Z_Length", "BoxGeom", vol.lz)
            # TODO: implement rotation if OrientedBox
        elif isinstance(vol, Ellipsoid):
            geom_id = vsp.AddGeom("ELLIPSOID", "")
            vsp.SetParmValUpdate(geom_id, "X_Location", "XForm", vol.cx)
            vsp.SetParmValUpdate(geom_id, "Y_Location", "XForm", vol.cy)
            vsp.SetParmValUpdate(geom_id, "Z_Location", "XForm", vol.cz)
            vsp.SetParmValUpdate(geom_id, "A_Radius", "Design", vol.rx)
            vsp.SetParmValUpdate(geom_id, "B_Radius", "Design", vol.ry)
            vsp.SetParmValUpdate(geom_id, "C_Radius", "Design", vol.rz)
    vsp.Update()


# -------------------------
# Main
# -------------------------
def main():
    os.makedirs(Config.OUTPUT_DIR, exist_ok=True)

    for conf_idx in range(1, Config.NUM_CONFIGS + 1):
        print(f"\n=== Generating configuration {conf_idx}/{Config.NUM_CONFIGS} ===")

        # Clear VSP model
        vsp.ClearVSPModel()

        # Load fuselage
        if not os.path.exists(Config.INPUT_VSP):
            raise FileNotFoundError(f"Input fuselage {Config.INPUT_VSP} not found")
        vsp.ReadVSPFile(Config.INPUT_VSP)
        vsp.Update()

        # Find fuselage geometry
        geom_ids = vsp.FindGeoms()
        fuse_id = None
        for gid in geom_ids:
            name = vsp.GetGeomName(gid)
            if "fuselage" in name.lower() or "fuse" in name.lower():
                fuse_id = gid
                break
        if fuse_id is None:
            raise RuntimeError("No fuselage found in model")

        print(f"  Found fuselage: {vsp.GetGeomName(fuse_id)}")

        # Analyze fuselage bounds
        bounds = analyze_fuselage_geometry(fuse_id)

        # Place volumes
        volumes = place_volumes(bounds,
                                num_volumes=Config.NUM_VOLUMES,
                                shape_types=Config.SHAPE_TYPES,
                                constrained_pairs=Config.CONSTRAINED_PAIRS)

        # Export to OpenVSP
        place_volumes_in_vsp(volumes)

        # Save VSP3
        vsp_file = os.path.join(Config.OUTPUT_DIR, f"fuse_conf{conf_idx}.vsp3")
        vsp.WriteVSPFile(vsp_file, vsp.SET_ALL)
        print(f"  Saved VSP3: {vsp_file}")

        # Save CSV and JSON metadata
        csv_file = os.path.join(Config.OUTPUT_DIR, f"volumes_conf{conf_idx}.csv")
        export_to_csv(volumes, csv_file)
        json_file = os.path.join(Config.OUTPUT_DIR, f"volumes_conf{conf_idx}.json")
        export_to_json(volumes, json_file)
        print(f"  Saved CSV: {csv_file}")
        print(f"  Saved JSON: {json_file}")

    print("\n=== All configurations generated ===")


if __name__ == "__main__":
    main()
