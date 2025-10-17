import openvsp as vsp

def create_fuselage(length: float, width: float, height: float, filename: str = "output.vsp3") -> None:
    """
    Create a fuselage with the specified length, width, and height.
    Saves the model as a VSP3 file and prints the real bounding box.
    This version scales the terminal XSecs to better match the target dimensions.
    """
    # Clear current model
    vsp.ClearVSPModel()
    
    # Add fuselage geometry
    fid = vsp.AddGeom("FUSELAGE", "")
    
    # Set fuselage length
    vsp.SetParmValUpdate(fid, "Length", "Design", length)   #THIS IS NATIVE
    
    # Update model to generate initial XSecs
    vsp.Update()
    
    # Get first XSecSurf
    xsec_surf_id = vsp.GetXSecSurf(fid, 0)
    num_xsecs = vsp.GetNumXSec(xsec_surf_id)
    
    # Linear scaling to better match target dimensions
    for i in range(num_xsecs):
        t = i / (num_xsecs - 1) if num_xsecs > 1 else 0.0  # normalized position along fuselage
        vsp.ChangeXSecShape(xsec_surf_id, i, vsp.XS_ELLIPSE)
        xsec_id = vsp.GetXSec(xsec_surf_id, i)

        # Gradual scaling: for nose/tail, scale ~0.9 of target
        scale_factor = 0.9 + 0.1 * t if i == 0 else 0.9 + 0.1 * (1 - t) if i == num_xsecs - 1 else 1.0

        vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Width"), width * scale_factor)     #THESE ARE NOT NATIVE OF A FUSELAGE, this is a subsection
        vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Ellipse_Height"), height * scale_factor)   #THESE ARE NOT NATIVE OF A FUSELAGE
    
    # Update and save the model
    vsp.Update()
    vsp.WriteVSPFile(filename, vsp.SET_ALL)
    
    # Real bounding box
    bbox_min = vsp.GetGeomBBoxMin(fid, 0, True)
    bbox_max = vsp.GetGeomBBoxMax(fid, 0, True)
    actual_length = bbox_max.x() - bbox_min.x()
    actual_width = bbox_max.y() - bbox_min.y()
    actual_height = bbox_max.z() - bbox_min.z()
    
    print(f"\nFuselage saved as: {filename}")
    print(f"Length (input): {length}, Width (input): {width}, Height (input): {height}")
    print("\nReal bounding box:")
    print(f"  Min: ({bbox_min.x():.2f}, {bbox_min.y():.2f}, {bbox_min.z():.2f})")
    print(f"  Max: ({bbox_max.x():.2f}, {bbox_max.y():.2f}, {bbox_max.z():.2f})")
    print(f"  Actual dimensions: Length={actual_length:.2f}, Width={actual_width:.2f}, Height={actual_height:.2f}")

if __name__ == "__main__":
    print("Enter fuselage parameters:")

    while True:
        try:
            length = float(input("Length (m): "))
            if length <= 0:
                raise ValueError
            break
        except ValueError:
            print("Please enter a valid positive number for length.")
    
    while True:
        try:
            width = float(input("Width (m): "))
            if width <= 0:
                raise ValueError
            break
        except ValueError:
            print("Please enter a valid positive number for width.")
    
    while True:
        try:
            height = float(input("Height (m): "))
            if height <= 0:
                raise ValueError
            break
        except ValueError:
            print("Please enter a valid positive number for height.")
    
    create_fuselage(length, width, height)