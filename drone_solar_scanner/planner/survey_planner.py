"""
Automatic Roof Photogrammetry and Solar Panel Survey Path Planner.
Generates optimal boustrophedon (lawnmower) flight grids, camera triggers,
and waypoint missions from roof GPS boundary polygons.
"""

import math
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional, Dict, Any
from shapely.geometry import Polygon, LineString, Point
from shapely import affinity

from .geodesy import gps_to_enu, enu_to_gps, calculate_bearing, haversine_distance


@dataclass
class CameraSpecs:
    """Optical payload / camera specifications for ground footprint calculation."""
    fov_horizontal_deg: float = 70.0  # Typical standard drone camera (e.g., DJI M3E, 24mm equiv.)
    fov_vertical_deg: float = 52.0
    aspect_ratio: float = 4.0 / 3.0


@dataclass
class SurveyConfig:
    """Mission and safety parameters for roof solar panel inspection."""
    altitude_agl: float = 15.0       # Flight altitude above roof level in meters
    cruise_speed: float = 3.0        # Flight speed during inspection in m/s
    forward_overlap: float = 0.70    # 70% forward overlap along flight lines
    side_overlap: float = 0.60       # 60% lateral overlap between adjacent passes
    setback_distance: float = 1.0    # Inward safety buffer from roof edge in meters
    roof_azimuth_deg: Optional[float] = None  # Alignment angle (None = auto-aligned to longest edge)
    gimbal_pitch_deg: float = -90.0  # Nadir camera pitch


@dataclass
class SurveyWaypoint:
    """Survey mission waypoint with geodetic and flight parameters."""
    index: int
    lat: float
    lon: float
    alt: float
    speed: float
    action: str              # 'TAKEOFF', 'WAYPOINT', 'CAMERA_TRIGGER', 'RTL'
    heading: float          # Drone yaw direction in degrees [0, 360)
    gimbal_pitch: float = -90.0
    line_index: int = 0     # Sweep track index

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class RoofSurveyPlanner:
    """
    Computes optimal boustrophedon scanning routes over solar panel roof polygons.
    """

    def __init__(self, camera: Optional[CameraSpecs] = None, config: Optional[SurveyConfig] = None):
        self.camera = camera or CameraSpecs()
        self.config = config or SurveyConfig()

    def calculate_footprint(self, altitude_agl: float) -> Tuple[float, float, float, float]:
        """
        Calculates ground footprint dimensions and required track spacings.
        Returns: (ground_width, ground_height, line_spacing, trigger_interval) in meters.
        """
        fov_h_rad = math.radians(self.camera.fov_horizontal_deg)
        fov_v_rad = math.radians(self.camera.fov_vertical_deg)

        ground_width = 2.0 * altitude_agl * math.tan(fov_h_rad / 2.0)
        ground_height = 2.0 * altitude_agl * math.tan(fov_v_rad / 2.0)

        # Lateral distance between flight tracks
        line_spacing = ground_width * (1.0 - self.config.side_overlap)
        # Longitudinal trigger distance along tracks
        trigger_interval = ground_height * (1.0 - self.config.forward_overlap)

        return ground_width, ground_height, line_spacing, trigger_interval

    def _determine_optimal_azimuth(self, polygon_enu: Polygon) -> float:
        """Finds the orientation of the longest edge of the polygon to minimize turns."""
        coords = list(polygon_enu.exterior.coords)
        max_len = 0.0
        best_angle = 0.0

        for i in range(len(coords) - 1):
            p1 = coords[i]
            p2 = coords[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            length = math.hypot(dx, dy)
            if length > max_len:
                max_len = length
                # Compass bearing: 0 is North (+Y), 90 is East (+X)
                best_angle = math.degrees(math.atan2(dx, dy)) % 360.0

        return best_angle

    def plan_mission(self, roof_polygon_gps: List[Tuple[float, float]]) -> List[SurveyWaypoint]:
        """
        Generate complete autonomous inspection waypoints for the given roof GPS polygon.
        
        Args:
            roof_polygon_gps: List of (latitude, longitude) tuples describing the roof boundary.
            
        Returns:
            List of SurveyWaypoint objects ready for MAVLink upload or DJI mission export.
        """
        if len(roof_polygon_gps) < 3:
            raise ValueError("A polygon requires at least 3 GPS boundary coordinates.")

        ref_lat, ref_lon = roof_polygon_gps[0]

        # 1. Project GPS coordinates to local ENU Cartesian plane
        enu_coords = [gps_to_enu(lat, lon, ref_lat, ref_lon) for lat, lon in roof_polygon_gps]
        poly_enu = Polygon(enu_coords)

        if not poly_enu.is_valid:
            poly_enu = poly_enu.buffer(0)

        # 2. Apply inward safety setback buffer if possible
        if self.config.setback_distance > 0:
            buffered = poly_enu.buffer(-self.config.setback_distance)
            if not buffered.is_empty and buffered.area > 5.0:
                survey_poly = buffered
            else:
                survey_poly = poly_enu
        else:
            survey_poly = poly_enu

        # 3. Determine sweep rotation angle
        if self.config.roof_azimuth_deg is not None:
            azimuth = self.config.roof_azimuth_deg
        else:
            azimuth = self._determine_optimal_azimuth(poly_enu)

        # Mathematical rotation angle for Cartesian system (counter-clockwise from +X)
        cartesian_rot = 90.0 - azimuth

        # Rotate survey polygon around centroid to align flight lines with X-axis
        centroid = survey_poly.centroid
        rotated_poly = affinity.rotate(survey_poly, -cartesian_rot, origin=centroid)

        # 4. Calculate footprint & flight grid dimensions
        g_w, g_h, line_spacing, trigger_interval = self.calculate_footprint(self.config.altitude_agl)
        line_spacing = max(line_spacing, 1.5)  # Enforce reasonable minimum spacing
        trigger_interval = max(trigger_interval, 1.0)

        minx, miny, maxx, maxy = rotated_poly.bounds

        # 5. Generate parallel slice lines along Y-axis
        sweep_lines_enu: List[List[Tuple[float, float]]] = []
        y_cursor = miny + (line_spacing / 2.0)
        line_idx = 0

        while y_cursor <= maxy:
            cut_line = LineString([(minx - 10.0, y_cursor), (maxx + 10.0, y_cursor)])
            intersection = rotated_poly.intersection(cut_line)

            if not intersection.is_empty:
                segments = []
                if intersection.geom_type == 'LineString':
                    segments.append(intersection)
                elif intersection.geom_type == 'MultiLineString':
                    segments.extend(intersection.geoms)

                for seg in segments:
                    p_start, p_end = list(seg.coords)[0], list(seg.coords)[-1]
                    # Discretize line into camera trigger waypoints
                    seg_len = math.hypot(p_end[0] - p_start[0], p_end[1] - p_start[1])
                    if seg_len < 1.0:
                        continue

                    num_triggers = max(1, int(math.ceil(seg_len / trigger_interval)))
                    pts = []
                    for s in range(num_triggers + 1):
                        frac = s / float(num_triggers)
                        px = p_start[0] + frac * (p_end[0] - p_start[0])
                        py = p_start[1] + frac * (p_end[1] - p_start[1])
                        pts.append((px, py))

                    # Alternating serpentine / boustrophedon direction
                    if line_idx % 2 == 1:
                        pts.reverse()

                    sweep_lines_enu.append(pts)

            y_cursor += line_spacing
            line_idx += 1

        if not sweep_lines_enu:
            raise ValueError("No survey tracks generated. The roof polygon may be too small for the specified altitude/spacing.")

        # 6. Rotate points back to original ENU orientation and convert to GPS waypoints
        waypoints: List[SurveyWaypoint] = []
        wp_idx = 0

        # Initial Takeoff Point at first survey coordinate
        first_p_rot = sweep_lines_enu[0][0]
        p_pt = Point(first_p_rot)
        orig_pt = affinity.rotate(p_pt, cartesian_rot, origin=centroid)
        first_lat, first_lon = enu_to_gps(orig_pt.x, orig_pt.y, ref_lat, ref_lon)

        waypoints.append(SurveyWaypoint(
            index=wp_idx,
            lat=first_lat,
            lon=first_lon,
            alt=self.config.altitude_agl,
            speed=self.config.cruise_speed,
            action="TAKEOFF",
            heading=azimuth,
            gimbal_pitch=self.config.gimbal_pitch_deg,
            line_index=-1
        ))
        wp_idx += 1

        for track_id, track_pts in enumerate(sweep_lines_enu):
            for pt_rot in track_pts:
                p_pt = Point(pt_rot)
                orig_pt = affinity.rotate(p_pt, cartesian_rot, origin=centroid)
                wp_lat, wp_lon = enu_to_gps(orig_pt.x, orig_pt.y, ref_lat, ref_lon)

                # Compute heading towards next point if available
                heading = azimuth
                if len(waypoints) > 1:
                    prev_wp = waypoints[-1]
                    if haversine_distance(prev_wp.lat, prev_wp.lon, wp_lat, wp_lon) > 0.3:
                        heading = calculate_bearing(prev_wp.lat, prev_wp.lon, wp_lat, wp_lon)
                        prev_wp.heading = heading

                waypoints.append(SurveyWaypoint(
                    index=wp_idx,
                    lat=wp_lat,
                    lon=wp_lon,
                    alt=self.config.altitude_agl,
                    speed=self.config.cruise_speed,
                    action="CAMERA_TRIGGER",
                    heading=heading,
                    gimbal_pitch=self.config.gimbal_pitch_deg,
                    line_index=track_id
                ))
                wp_idx += 1

        # Final Return-To-Launch (RTL) Waypoint
        last_wp = waypoints[-1]
        waypoints.append(SurveyWaypoint(
            index=wp_idx,
            lat=first_lat,
            lon=first_lon,
            alt=self.config.altitude_agl,
            speed=self.config.cruise_speed,
            action="RTL",
            heading=last_wp.heading,
            gimbal_pitch=0.0,
            line_index=-2
        ))

        return waypoints
