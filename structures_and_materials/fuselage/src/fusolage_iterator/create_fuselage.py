try:
    from openvsp import vsp
except ImportError:
    import vsp_mock as vsp

def create_fuselage(length, width, height, filename="fuselage.vsp3"):
    vsp.ClearVSPModel()
    fid = vsp.AddGeom("FUSELAGE", "")

    # Set parameters (real OpenVSP API call, the mock version just prints)
    vsp.SetParmValUpdate(fid, "Length", "Design", length)
    vsp.SetParmValUpdate(fid, "Diameter", "Design", width)  # fuselage width ≈ diameter
    vsp.SetParmValUpdate(fid, "Height", "Design", height)   # only if fuselage supports it

    vsp.Update()
    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage saved to {filename}")

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
