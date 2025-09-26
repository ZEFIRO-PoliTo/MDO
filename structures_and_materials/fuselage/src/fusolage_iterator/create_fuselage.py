try:
    from openvsp import vsp
except ImportError:
    import vsp_mock as vsp

def create_fuselage(length, width, height, filename="fuselage.vsp3"):
    vsp.ClearVSPModel()          # reset scene
    fid = vsp.AddGeom("FUSELAGE", "")
    # minimal update
    vsp.Update()
    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    print(f"Fuselage saved to {filename}")
