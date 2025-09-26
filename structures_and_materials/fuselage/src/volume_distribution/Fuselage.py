import openvsp as vsp

# Model Reset
vsp.ClearVSPModel()

# Create a simple fuselage
fuse_id = vsp.AddGeom("FUSELAGE", "")

# Define the length of the fuselage
vsp.SetParmValUpdate(fuse_id, "Length", "Design", 20.0)

# Starting position (optional, default 0)
vsp.SetParmValUpdate(fuse_id, "X_Location", "XForm", 0.0)
vsp.SetParmValUpdate(fuse_id, "Y_Location", "XForm", 0.0)
vsp.SetParmValUpdate(fuse_id, "Z_Location", "XForm", 0.0)
# Update model
vsp.Update()

# Save the file in the right path
vsp.WriteVSPFile("input_fuse.vsp3", vsp.SET_ALL)
print(f"✅ Fusolage created and saved")
