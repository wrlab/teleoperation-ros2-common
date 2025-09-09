import math

def rad_to_deg(x: float) -> float:
    return math.degrees(x)

def deg_to_rad(x: float) -> float:
    return math.radians(x)

def list_rad_to_deg(values):
    return [math.degrees(v) for v in values]

def list_deg_to_rad(values):
    return [math.radians(v) for v in values]


