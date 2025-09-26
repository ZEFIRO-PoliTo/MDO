def ClearVSPModel():
    print("[MOCK] ClearVSPModel called")

def AddGeom(name, group=""):
    print(f"[MOCK] AddGeom called with {name}")
    return 1  # mock geometry ID

def Update():
    print("[MOCK] Update called")

def WriteVSPFile(filename, options=None):
    print(f"[MOCK] WriteVSPFile called: {filename}")

def SetParmValUpdate(geom_id, parm_name, parm_group, val):
    print(f"[MOCK] SetParmValUpdate called: {parm_name}={val}")
