"""
collision.py
Collision and constraint checking.
"""

import numpy as np
from shapes import VolumeShape, Ellipsoid, Box, OrientedBox, EPS

def aabb_overlap(a_min,a_max,b_min,b_max): return not (np.any(a_max<b_min-EPS) or np.any(b_max<a_min-EPS))

def obb_intersect(box1: OrientedBox, box2: OrientedBox):
    # Standard SAT for 3x3 oriented boxes (simplified)
    C1,C2 = np.array([box1.cx,box1.cy,box1.cz]), np.array([box2.cx,box2.cy,box2.cz])
    R1,R2 = box1.rotation_matrix(), box2.rotation_matrix()
    A,B = [R1[:,0],R1[:,1],R1[:,2]], [R2[:,0],R2[:,1],R2[:,2]]
    a,b = box1.half_sizes(), box2.half_sizes()
    R = np.zeros((3,3))
    for i in range(3):
        for j in range(3): R[i,j]=float(np.dot(A[i],B[j]))
    t_world = C2-C1
    t = np.array([np.dot(t_world,A[i]) for i in range(3)])
    absR = np.abs(R)+1e-12
    for i in range(3):
        if abs(t[i])> a[i]+b[0]*absR[i,0]+b[1]*absR[i,1]+b[2]*absR[i,2]+EPS: return False
    for j in range(3):
        if abs(t[0]*R[0,j]+t[1]*R[1,j]+t[2]*R[2,j])> a[0]*absR[0,j]+a[1]*absR[1,j]+a[2]*absR[2,j]+b[j]+EPS: return False
    for i in range(3):
        for j in range(3):
            ra = a[(i+1)%3]*absR[(i+2)%3,j]+a[(i+2)%3]*absR[(i+1)%3,j]
            rb = b[(j+1)%3]*absR[i,(j+2)%3]+b[(j+2)%3]*absR[i,(j+1)%3]
            tij = abs(t[(i+2)%3]*R[(i+1)%3,j]-t[(i+1)%3]*R[(i+2)%3,j])
            if tij>ra+rb+EPS: return False
    return True

def box_box_collision(v1:VolumeShape,v2:VolumeShape)->bool:
    if isinstance(v1,OrientedBox) and isinstance(v2,OrientedBox):
        a_min,a_max = v1.get_aabb(); b_min,b_max = v2.get_aabb()
        if not aabb_overlap(a_min,a_max,b_min,b_max): return False
        return obb_intersect(v1,v2)
    else:
        a_min,a_max = v1.get_aabb(); b_min,b_max=v2.get_aabb()
        return aabb_overlap(a_min,a_max,b_min,b_max)

def ellipsoid_box_collision(ell:Ellipsoid,box:VolumeShape)->bool:
    sc = np.array([ell.cx,ell.cy,ell.cz])
    sph_r = ell.bounding_sphere_radius()
    bmin,bmax = box.get_aabb()
    closest = np.maximum(bmin,np.minimum(sc,bmax))
    d2 = np.sum((closest-sc)**2)
    return d2<=(sph_r+EPS)**2

def check_collision(volumes:list)->bool:
    """Return True if any collision exists between volumes"""
    n = len(volumes)
    for i in range(n):
        for j in range(i+1,n):
            v1,v2 = volumes[i],volumes[j]
            if isinstance(v1,Ellipsoid) and isinstance(v2,Ellipsoid):
                rsum = v1.bounding_sphere_radius()+v2.bounding_sphere_radius()
                if np.linalg.norm(np.array([v1.cx,v1.cy,v1.cz])-np.array([v2.cx,v2.cy,v2.cz]))<rsum: return True
            elif isinstance(v1,Ellipsoid) or isinstance(v2,Ellipsoid):
                ell,box = (v1,v2) if isinstance(v1,Ellipsoid) else (v2,v1)
                if ellipsoid_box_collision(ell,box): return True
            else:
                if box_box_collision(v1,v2): return True
    return False
