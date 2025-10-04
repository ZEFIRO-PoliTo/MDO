"""
placed_volume.py
Fuselage analysis and volume placement helper
"""

from dataclasses import dataclass
from typing import List, Tuple
import math
import openvsp as vsp

@dataclass
class CrossSection:
    x: float
    radius_y: float
    radius_z: float
    area: float

@dataclass
class FuselageBounds:
    length: float
    x_min: float
    x_max: float
    cross_sections: List[CrossSection]
    bbox_min: Tuple[float,float,float]
    bbox_max: Tuple[float,float,float]

def estimate_local_radius(fuse_id:str,x:float,bbox_min:tuple,bbox_max:tuple)->Tuple[float,float]:
    length = bbox_max[0]-bbox_min[0]
    width = bbox_max[1]-bbox_min[1]
    height = bbox_max[2]-bbox_min[2]
    t = (x-bbox_min[0])/length if length>0 else 0
    if t<0.15: taper=0.1+0.9*(t/0.15)
    elif t>0.85: taper=0.1+0.9*((1.0-t)/0.15)
    else: taper=1.0
    return width/2*taper, height/2*taper

def analyze_fuselage_geometry(fuse_id:str)->FuselageBounds:
    bbox_min_vec = vsp.GetGeomBBoxMin(fuse_id,0,True)
    bbox_max_vec = vsp.GetGeomBBoxMax(fuse_id,0,True)
    bbox_min = (bbox_min_vec.x(),bbox_min_vec.y(),bbox_min_vec.z())
    bbox_max = (bbox_max_vec.x(),bbox_max_vec.y(),bbox_max_vec.z())
    length = bbox_max[0]-bbox_min[0]
    cross_sections=[]
    num_samples=20
    for i in range(num_samples):
        t = i/(num_samples-1) if num_samples>1 else 0
        x = bbox_min[0]+t*length
        ry,rz = estimate_local_radius(fuse_id,x,bbox_min,bbox_max)
        cross_sections.append(CrossSection(x,ry,rz,math.pi*ry*rz))
    return FuselageBounds(length=length,x_min=bbox_min[0],x_max=bbox_max[0],cross_sections=cross_sections,bbox_min=bbox_min,bbox_max=bbox_max)
