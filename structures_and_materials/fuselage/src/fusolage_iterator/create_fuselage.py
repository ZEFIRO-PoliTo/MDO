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

