"""Small geodesy helpers (local tangent plane, good for missions under ~10 km)."""
import math

EARTH_RADIUS_M = 6371008.8


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def enu_offset(lat0: float, lon0: float, lat1: float, lon1: float):
    """East/North offset in metres from (lat0, lon0) to (lat1, lon1)."""
    lat0r = math.radians(lat0)
    d_east = math.radians(lon1 - lon0) * math.cos(lat0r) * EARTH_RADIUS_M
    d_north = math.radians(lat1 - lat0) * EARTH_RADIUS_M
    return d_east, d_north


def distance_bearing_enu(lat0, lon0, lat1, lon1):
    """Distance (m) and bearing (rad, ENU convention: 0 = East, CCW positive)."""
    e, n = enu_offset(lat0, lon0, lat1, lon1)
    return math.hypot(e, n), math.atan2(n, e)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
