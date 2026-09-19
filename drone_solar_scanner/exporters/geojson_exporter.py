"""
GeoJSON Survey Exporter.
Exports roof boundary polygons, planned flight tracks, and inspection waypoints
as standard RFC 7946 GeoJSON FeatureCollections for GIS applications.
"""

import json
from typing import List, Tuple, Dict, Any
from ..planner.survey_planner import SurveyWaypoint


class GeoJSONExporter:
    """Exports roof survey data and flight missions to GeoJSON."""

    @classmethod
    def export(cls, waypoints: List[SurveyWaypoint], roof_polygon_gps: List[Tuple[float, float]]) -> Dict[str, Any]:
        """
        Builds a GeoJSON FeatureCollection with:
        - Roof Boundary Polygon
        - Flight Path LineString
        - Waypoint Point Features
        """
        features: List[Dict[str, Any]] = []

        # 1. Roof Boundary Feature
        # GeoJSON coordinates are in [longitude, latitude] order
        poly_coords = [[lon, lat] for lat, lon in roof_polygon_gps]
        if poly_coords and poly_coords[0] != poly_coords[-1]:
            poly_coords.append(poly_coords[0])  # Close ring

        features.append({
            "type": "Feature",
            "properties": {
                "name": "Roof Boundary",
                "featureType": "roof_boundary",
                "stroke": "#f59e0b",
                "stroke-width": 3,
                "fill": "#f59e0b",
                "fill-opacity": 0.2
            },
            "geometry": {
                "type": "Polygon",
                "coordinates": [poly_coords]
            }
        })

        # 2. Flight Path LineString Feature
        line_coords = [[wp.lon, wp.lat, wp.alt] for wp in waypoints]
        features.append({
            "type": "Feature",
            "properties": {
                "name": "Autonomous Flight Path",
                "featureType": "flight_path",
                "stroke": "#0284c7",
                "stroke-width": 2.5
            },
            "geometry": {
                "type": "LineString",
                "coordinates": line_coords
            }
        })

        # 3. Waypoint Points
        for wp in waypoints:
            features.append({
                "type": "Feature",
                "properties": {
                    "index": wp.index,
                    "action": wp.action,
                    "alt_agl": wp.alt,
                    "speed": wp.speed,
                    "heading": wp.heading,
                    "line_index": wp.line_index,
                    "featureType": "waypoint"
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [wp.lon, wp.lat, wp.alt]
                }
            })

        return {
            "type": "FeatureCollection",
            "features": features
        }

    @classmethod
    def save_to_file(cls, waypoints: List[SurveyWaypoint], roof_polygon_gps: List[Tuple[float, float]], file_path: str) -> str:
        """Saves survey mission to a .geojson file."""
        geojson_dict = cls.export(waypoints, roof_polygon_gps)
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(geojson_dict, f, indent=2)
        return file_path
