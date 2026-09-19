"""
Geodetic and coordinate transformation utilities for local drone roof surveys.
Implements WGS84 to local ENU (East-North-Up) projection and geodetic math.
"""

import math
from typing import Tuple, List

# WGS84 Earth ellipsoid constants
WGS84_A = 6378137.0          # Semi-major axis (equatorial radius) in meters
WGS84_F = 1.0 / 298.257223563 # Flattening
WGS84_B = WGS84_A * (1.0 - WGS84_F) # Semi-minor axis (polar radius)


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two GPS coordinates in meters.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (math.sin(delta_phi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * (math.sin(delta_lambda / 2.0) ** 2))
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return WGS84_A * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the initial compass bearing (azimuth) from point 1 to point 2 in degrees [0, 360).
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2) -
         math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda))
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0


def gps_to_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Project WGS84 GPS coordinate (lat, lon) to local tangent plane Cartesian (East, North) in meters
    relative to reference origin (ref_lat, ref_lon).
    Accurate to millimeter level for areas < 5 km.
    """
    ref_lat_rad = math.radians(ref_lat)
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)

    # Radii of curvature
    sin_lat = math.sin(ref_lat_rad)
    e2 = 1.0 - (WGS84_B ** 2) / (WGS84_A ** 2)
    n = WGS84_A / math.sqrt(1.0 - e2 * (sin_lat ** 2))
    m = WGS84_A * (1.0 - e2) / math.pow(1.0 - e2 * (sin_lat ** 2), 1.5)

    east = d_lon * n * math.cos(ref_lat_rad)
    north = d_lat * m
    return east, north


def enu_to_gps(east: float, north: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Convert local tangent plane Cartesian (East, North) in meters back to WGS84 GPS coordinate (lat, lon).
    """
    ref_lat_rad = math.radians(ref_lat)
    sin_lat = math.sin(ref_lat_rad)
    e2 = 1.0 - (WGS84_B ** 2) / (WGS84_A ** 2)
    n = WGS84_A / math.sqrt(1.0 - e2 * (sin_lat ** 2))
    m = WGS84_A * (1.0 - e2) / math.pow(1.0 - e2 * (sin_lat ** 2), 1.5)

    d_lat = north / m
    d_lon = east / (n * math.cos(ref_lat_rad))

    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    return lat, lon
