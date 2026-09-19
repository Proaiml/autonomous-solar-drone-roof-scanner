"""
End-to-End System Test for Universal Drone Autonomous Solar Roof Scanner.
Executes complete autonomous pipeline:
Roof Polygon -> Survey Grid -> Drone Flight Simulation -> PyTorch AI Dust Classification -> Geotagged Reports.
"""

import os
import json
import pytest
from drone_solar_scanner.planner.survey_planner import RoofSurveyPlanner, SurveyConfig
from drone_solar_scanner.vision.detector import SolarDustDetector
from drone_solar_scanner.vision.stream_processor import VideoStreamProcessor
from drone_solar_scanner.drone.virtual_controller import VirtualDroneController
from drone_solar_scanner.reporting.reporter import SolarInspectionReporter


def test_end_to_end_autonomous_scan_and_reporting(tmp_path):
    # 1. Define Roof Polygon (40m x 25m rooftop)
    roof_polygon = [
        (39.920800, 32.854100),
        (39.921100, 32.854100),
        (39.921100, 32.854500),
        (39.920800, 32.854500)
    ]

    # 2. Plan Mission
    config = SurveyConfig(altitude_agl=12.0, cruise_speed=3.0, forward_overlap=0.70, side_overlap=0.60)
    planner = RoofSurveyPlanner(config=config)
    waypoints = planner.plan_mission(roof_polygon)
    assert len(waypoints) > 5

    # 3. Initialize AI Vision Pipeline
    detector = SolarDustDetector(model_weights_path="resnet18_solar_dust.pth")
    snap_dir = str(tmp_path / "snapshots")
    stream_processor = VideoStreamProcessor(detector, output_dir=snap_dir)

    # 4. Initialize Drone Controller
    drone = VirtualDroneController(home_lat=roof_polygon[0][0], home_lon=roof_polygon[0][1])
    assert drone.connect()

    inspections = []
    clean_sample = "tests/sample_data/clean/Imgclean_0_0.jpg"
    dusty_sample = "tests/sample_data/dusty/Imgdirty_1001_1.jpg"

    def waypoint_callback(seq: int, wp):
        if wp.action == "CAMERA_TRIGGER":
            telemetry = drone.get_telemetry()
            # Alternate samples to simulate real roof with some dirty panels
            sample = dusty_sample if (seq % 2 == 0) else clean_sample
            record = stream_processor.inspect_at_point(telemetry, sample_image_override=sample)
            inspections.append(record)

    drone.register_waypoint_reached_callback(waypoint_callback)
    drone.upload_mission(waypoints)
    drone.arm_and_takeoff(config.altitude_agl)
    drone.start_mission()
    drone.disconnect()

    assert len(inspections) > 0, "Inspections must be recorded at camera trigger waypoints."
    assert all(os.path.exists(item.snapshot_path) for item in inspections)

    # 5. Generate and Verify Reports
    reporter = SolarInspectionReporter(inspections, roof_polygon, site_name="Test Industrial Site")
    stats = reporter.compute_summary_stats()

    assert stats["total_inspected"] == len(inspections)
    assert stats["clean_count"] + stats["dusty_count"] == stats["total_inspected"]
    assert pytest.approx(stats["soiling_ratio_pct"] + stats["clean_ratio_pct"], abs=0.1) == 100.0
    assert stats["estimated_power_loss_pct"] >= 0.0

    # 6. Export Reports
    json_path = str(tmp_path / "report.json")
    csv_path = str(tmp_path / "summary.csv")
    html_path = str(tmp_path / "map.html")

    reporter.export_json(json_path)
    reporter.export_csv(csv_path)
    reporter.generate_html_map(html_path)

    assert os.path.exists(json_path) and os.path.getsize(json_path) > 0
    assert os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
    assert os.path.exists(html_path) and os.path.getsize(html_path) > 0

    # Verify JSON content
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
        assert data["site_name"] == "Test Industrial Site"
        assert len(data["inspections"]) == len(inspections)

    # Verify HTML contains Leaflet map and polygon
    with open(html_path, "r", encoding="utf-8") as f:
        html_str = f.read()
        assert "L.map" in html_str
        assert "Solar Roof AI Inspection Dashboard" in html_str
