def cuboid_inside_fuselage(volume, fuselage_dims):
    # Simple AABB check for cuboid inside fuselage bounds
    fx, fy, fz = fuselage_dims
    half_fx, half_fy, half_fz = fx/2, fy/2, fz/2
    x_ok = (-half_fx <= volume.x - volume.length/2) and (volume.x + volume.length/2 <= half_fx)
    y_ok = (-half_fy <= volume.y - volume.width/2) and (volume.y + volume.width/2 <= half_fy)
    z_ok = (-half_fz <= volume.z - volume.height/2) and (volume.z + volume.height/2 <= half_fz)
    return x_ok and y_ok and z_ok

def cuboid_collision(vol1, vol2):
    # AABB collision check for two cuboids
    return (
        abs(vol1.x - vol2.x) * 2 < (vol1.length + vol2.length) and
        abs(vol1.y - vol2.y) * 2 < (vol1.width + vol2.width) and
        abs(vol1.z - vol2.z) * 2 < (vol1.height + vol2.height)
    )