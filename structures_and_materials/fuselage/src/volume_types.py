from dataclasses import dataclass

@dataclass
class Volume:
    shape: str  # 'ellipsoid' or 'cuboid'
    length: float
    width: float
    height: float
    x: float
    y: float
    z: float
    constraints: dict = None  # e.g. {'min_dist': 0.5, 'must_be_apart': [1, 2]}