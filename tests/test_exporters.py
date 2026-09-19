"""
Unit tests for mission format exporters (QGroundControl, DJI WPML/KML, MAVLink, GeoJSON).
Verifies structural and schema compliance of exported mission files.
"""

import json
import xml.etree.ElementTree as ET
from drone_solar_scanner.planner.survey_planner import SurveyWaypoint
from drone_solar_scanner.exporters.qgc_plan_exporter import QGCPlanExporter
from drone_solar_scanner.exporters.dji_wpml_exporter import DJIWPMLExporter
from drone_solar_scanner.exporters.mavlink_waypoint_exporter import MAVLinkWaypointExporter
from drone_solar_scanner.exporters.geojson_exporter import GeoJSONExporter


SAMPLE_WAYPOINTS = [
    SurveyWaypoint(index=0, lat=39.920, lon=32.850, alt=15.0, speed=3.0, action="TAKEOFF", heading=45.0),
    SurveyWaypoint(index=1, lat=39.921, lon=32.851, alt=15.0, speed=3.0, action="CAMERA_TRIGGER", heading=45.0),
    SurveyWaypoint(index=2, lat=39.920, lon=32.850, alt=15.0, speed=3.0, action="RTL", heading=45.0),
]
SAMPLE_POLYGON = [(39.920, 32.850), (39.922, 32.850), (39.922, 32.852), (39.920, 32.852)]


def test_qgc_plan_exporter():
    plan = QGCPlanExporter.export(SAMPLE_WAYPOINTS, cruise_speed=3.5)
    assert plan["fileType"] == "Plan"
    assert plan["version"] == 1
    assert "mission" in plan
    assert plan["mission"]["cruiseSpeed"] == 3.5
    assert len(plan["mission"]["items"]) == 3

    # Check commands
    items = plan["mission"]["items"]
    assert items[0]["command"] == QGCPlanExporter.MAV_CMD_NAV_TAKEOFF
    assert items[1]["command"] == QGCPlanExporter.MAV_CMD_NAV_WAYPOINT
    assert items[2]["command"] == QGCPlanExporter.MAV_CMD_NAV_RETURN_TO_LAUNCH


def test_dji_wpml_exporter():
    kml_str = DJIWPMLExporter.export_kml(SAMPLE_WAYPOINTS, mission_name="TestRoofMission")
    # Verify valid XML
    root = ET.fromstring(kml_str)
    assert "Document" in root[0].tag or root.tag.endswith("kml")
    assert "TestRoofMission" in kml_str
    assert "wpml:missionConfig" in kml_str
    assert "LineString" in kml_str


def test_mavlink_waypoint_exporter():
    txt = MAVLinkWaypointExporter.export_text(SAMPLE_WAYPOINTS)
    lines = txt.strip().split("\n")
    assert lines[0] == "QGC WPL 110"
    assert len(lines) == 4  # Header + 3 waypoints

    # Check columns format: INDEX CURRENT FRAME CMD P1 P2 P3 P4 LAT LON ALT AUTOCONTINUE
    parts = lines[1].split("\t")
    assert len(parts) == 12
    assert parts[0] == "0"
    assert parts[3] == "22" # TAKEOFF


def test_geojson_exporter():
    geo = GeoJSONExporter.export(SAMPLE_WAYPOINTS, SAMPLE_POLYGON)
    assert geo["type"] == "FeatureCollection"
    assert len(geo["features"]) >= 4 # Polygon + LineString + Points

    types = [f["geometry"]["type"] for f in geo["features"]]
    assert "Polygon" in types
    assert "LineString" in types
    assert "Point" in types
