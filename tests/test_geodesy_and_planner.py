"""
Unit tests for Geodesy transformations and RoofSurveyPlanner.
Verifies projection math, footprint calculations, and boustrophedon sweep tracks.
"""

import math
import pytest

from drone_solar_scanner.planner.geodesy import (
    haversine_distance,
    calculate_bearing,
    gps_to_enu,
    enu_to_gps
)
from drone_solar_scanner.planner.survey_planner import (
    RoofSurveyPlanner,
    CameraSpecs,
    SurveyConfig,
    SurveyWaypoint
)


def test_haversine_distance():
    # 1 deg latitude difference at equator is approx 111,195 meters
    d = haversine_distance(0.0, 0.0, 1.0, 0.0)
    assert pytest.approx(d, rel=0.01) == 111195.0

    # Same point distance must be 0
    assert haversine_distance(39.92, 32.85, 39.92, 32.85) == 0.0


def test_calculate_bearing():
    # Heading due north
    assert pytest.approx(calculate_bearing(0.0, 0.0, 1.0, 0.0), abs=0.1) == 0.0
    # Heading due east
    assert pytest.approx(calculate_bearing(0.0, 0.0, 0.0, 1.0), abs=0.1) == 90.0
    # Heading due south
    assert pytest.approx(calculate_bearing(1.0, 0.0, 0.0, 0.0), abs=0.1) == 180.0
    # Heading due west
    assert pytest.approx(calculate_bearing(0.0, 1.0, 0.0, 0.0), abs=0.1) == 270.0


def test_enu_gps_bidirectional_conversion():
    ref_lat, ref_lon = 39.9208, 32.8541
    # Move 100m East, 50m North
    lat, lon = enu_to_gps(100.0, 50.0, ref_lat, ref_lon)

    # Convert back to ENU
    east, north = gps_to_enu(lat, lon, ref_lat, ref_lon)
    assert pytest.approx(east, abs=1e-3) == 100.0
    assert pytest.approx(north, abs=1e-3) == 50.0


def test_survey_planner_grid_generation():
    # Factory roof polygon (approx 60m x 40m)
    roof_poly = [
        (39.9200, 32.8500),
        (39.9206, 32.8500),
        (39.9206, 32.8506),
        (39.9200, 32.8506)
    ]

    camera = CameraSpecs(fov_horizontal_deg=70.0, fov_vertical_deg=52.0)
    config = SurveyConfig(
        altitude_agl=15.0,
        cruise_speed=3.0,
        forward_overlap=0.70,
        side_overlap=0.60,
        setback_distance=1.0
    )

    planner = RoofSurveyPlanner(camera=camera, config=config)
    waypoints = planner.plan_mission(roof_poly)

    # Must contain Takeoff, Camera Triggers, and RTL
    assert len(waypoints) >= 4
    assert waypoints[0].action == "TAKEOFF"
    assert waypoints[-1].action == "RTL"
    assert any(wp.action == "CAMERA_TRIGGER" for wp in waypoints)

    # Waypoints must stay within reasonable bounds around the roof
    for wp in waypoints:
        assert 39.919 < wp.lat < 39.922
        assert 32.849 < wp.lon < 32.852
        assert wp.alt == 15.0
        assert wp.speed == 3.0
        assert 0.0 <= wp.heading <= 360.0
