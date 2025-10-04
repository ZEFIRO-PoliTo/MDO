"""
shapes.py
Geometric shape definitions and basic collision utilities used by
the volume placement / collision-detection pipeline.
"""

from dataclasses import dataclass, asdict
from typing import Tuple
import numpy as np
import math

EPS = 1e-9

def euler_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll); sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw); sy = math.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

@dataclass
class VolumeShape:
    def get_corners(self) -> np.ndarray:
        raise NotImplementedError
    def get_aabb(self) -> Tuple[np.ndarray, np.ndarray]:
        corners = self.get_corners()
        return np.min(corners, axis=0), np.max(corners, axis=0)
    def contains_point(self, pt: Tuple[float,float,float]) -> bool:
        raise NotImplementedError
    def to_dict(self):
        return asdict(self)

@dataclass
class Ellipsoid(VolumeShape):
    cx: float; cy: float; cz: float
    rx: float; ry: float; rz: float
    def get_corners(self) -> np.ndarray:
        c = np.array([self.cx, self.cy, self.cz])
        signs = np.array([[sx,sy,sz] for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)])
        radii = np.array([self.rx, self.ry, self.rz])
        return c + signs*radii
    def contains_point(self, pt): 
        x,y,z = pt
        val = ((x-self.cx)/(self.rx+EPS))**2 + ((y-self.cy)/(self.ry+EPS))**2 + ((z-self.cz)/(self.rz+EPS))**2
        return val <= 1.0
    def bounding_sphere_radius(self): return max(self.rx,self.ry,self.rz)

@dataclass
class Box(VolumeShape):
    cx: float; cy: float; cz: float
    lx: float; ly: float; lz: float
    def half_sizes(self): return np.array([self.lx/2,self.ly/2,self.lz/2])
    def get_corners(self):
        c = np.array([self.cx,self.cy,self.cz])
        hx,hy,hz = self.half_sizes()
        signs = np.array([[sx,sy,sz] for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)])
        return c + signs*np.array([hx,hy,hz])
    def contains_point(self, pt):
        px,py,pz = pt
        amin,amax = self.get_aabb()
        return (amin[0]-EPS<=px<=amax[0]+EPS) and (amin[1]-EPS<=py<=amax[1]+EPS) and (amin[2]-EPS<=pz<=amax[2]+EPS)

@dataclass
class OrientedBox(VolumeShape):
    cx: float; cy: float; cz: float
    lx: float; ly: float; lz: float
    roll: float=0; pitch: float=0; yaw: float=0
    def half_sizes(self): return np.array([self.lx/2,self.ly/2,self.lz/2])
    def rotation_matrix(self): return euler_to_rotmat(self.roll,self.pitch,self.yaw)
    def get_corners(self):
        c = np.array([self.cx,self.cy,self.cz])
        hx,hy,hz = self.half_sizes()
        local = np.array([[sx*hx,sy*hy,sz*hz] for sx in (-1,1) for sy in (-1,1) for sz in (-1,1)])
        R = self.rotation_matrix()
        return (R @ local.T).T + c
    def contains_point(self, pt):
        p = np.array(pt); c = np.array([self.cx,self.cy,self.cz])
        R = self.rotation_matrix()
        pl = R.T @ (p-c)
        hx,hy,hz = self.half_sizes()
        return abs(pl[0])<=hx+EPS and abs(pl[1])<=hy+EPS and abs(pl[2])<=hz+EPS
