"""
Autonomous Solar Roof Drone Scanner - Command Line Interface (CLI).
Provides tools for automated survey grid planning, commercial drone mission export,
live or simulated flight scanning, and AI solar panel dust analysis.
"""

import sys
import os
import json
import argparse
from typing import List, Tuple

from drone_solar_scanner.planner.survey_planner import RoofSurveyPlanner, CameraSpecs, SurveyConfig
from drone_solar_scanner.exporters.qgc_plan_exporter import QGCPlanExporter
from drone_solar_scanner.exporters.dji_wpml_exporter import DJIWPMLExporter
from drone_solar_scanner.exporters.mavlink_waypoint_exporter import MAVLinkWaypointExporter
from drone_solar_scanner.exporters.geojson_exporter import GeoJSONExporter
from drone_solar_scanner.vision.detector import SolarDustDetector
from drone_solar_scanner.vision.stream_processor import VideoStreamProcessor
from drone_solar_scanner.drone.virtual_controller import VirtualDroneController
from drone_solar_scanner.drone.mavlink_controller import MAVLinkDroneController
from drone_solar_scanner.reporting.reporter import SolarInspectionReporter


# Default sample rooftop polygon in Ankara (approx 50m x 30m factory roof)
SAMPLE_ROOF_POLYGON = [
    (39.920800, 32.854100),
    (39.921200, 32.854100),
    (39.921200, 32.854700),
    (39.920800, 32.854700)
]


def cmd_plan(args):
    """Generates boustrophedon flight grid and exports to commercial drone formats."""
    print("=" * 60)
    print("  AUTONOMOUS ROOF SURVEY PATH PLANNER")
    print("=" * 60)

    if args.polygon_file:
        with open(args.polygon_file, "r") as f:
            poly_data = json.load(f)
            polygon = [(pt[0], pt[1]) for pt in poly_data]
    else:
        polygon = SAMPLE_ROOF_POLYGON
        print("[Info] Using default sample roof polygon coordinates.")

    config = SurveyConfig(
        altitude_agl=args.altitude,
        cruise_speed=args.speed,
        forward_overlap=args.forward_overlap,
        side_overlap=args.side_overlap,
        setback_distance=args.setback,
        roof_azimuth_deg=args.azimuth
    )

    planner = RoofSurveyPlanner(config=config)
    waypoints = planner.plan_mission(polygon)

    print(f"Generated {len(waypoints)} survey waypoints.")
    print(f"Flight Altitude: {args.altitude}m AGL | Cruise Speed: {args.speed} m/s")

    out_dir = args.output_dir
    os.makedirs(out_dir, exist_ok=True)

    # 1. QGC Plan
    qgc_file = os.path.join(out_dir, "survey_mission.plan")
    QGCPlanExporter.save_to_file(waypoints, qgc_file, cruise_speed=args.speed)
    print(f"[Export] QGroundControl / PX4 Plan: {qgc_file}")

    # 2. DJI WPML / KML
    dji_file = os.path.join(out_dir, "dji_pilot2_mission.kml")
    DJIWPMLExporter.save_to_file(waypoints, dji_file, mission_name=args.mission_name)
    print(f"[Export] DJI Pilot 2 WPML / KML: {dji_file}")

    # 3. MAVLink Waypoints
    mav_file = os.path.join(out_dir, "mission_items.waypoints")
    MAVLinkWaypointExporter.save_to_file(waypoints, mav_file)
    print(f"[Export] MAVLink Waypoint File: {mav_file}")

    # 4. GIS GeoJSON
    geo_file = os.path.join(out_dir, "survey_tracks.geojson")
    GeoJSONExporter.save_to_file(waypoints, polygon, geo_file)
    print(f"[Export] GIS GeoJSON: {geo_file}")

    print("\nMission export ready for any commercial drone ground station!")


def cmd_scan(args):
    """Executes roof scanning mission and generates AI inspection report."""
    print("=" * 60)
    print("  AUTONOMOUS SOLAR PANEL INSPECTION SCAN")
    print("=" * 60)

    # 1. Plan Waypoints
    polygon = SAMPLE_ROOF_POLYGON
    config = SurveyConfig(altitude_agl=args.altitude, cruise_speed=args.speed)
    planner = RoofSurveyPlanner(config=config)
    waypoints = planner.plan_mission(polygon)
    print(f"Planned {len(waypoints)} inspection waypoints across roof.")

    # 2. Load AI Detector
    print(f"Loading ResNet-18 detector weights from: {args.weights}...")
    detector = SolarDustDetector(model_weights_path=args.weights)
    stream_proc = VideoStreamProcessor(detector, output_dir=os.path.join(args.output_dir, "snapshots"))

    # 3. Select Drone Controller
    if args.mavlink:
        print(f"Connecting to live drone telemetry via MAVLink ({args.mavlink})...")
        drone = MAVLinkDroneController()
        connected = drone.connect(args.mavlink)
        if not connected:
            print("[Error] Failed to connect to MAVLink telemetry. Aborting.")
            return
    else:
        print("Initializing high-fidelity virtual drone flight simulator...")
        drone = VirtualDroneController(home_lat=polygon[0][0], home_lon=polygon[0][1])
        drone.connect()

    inspections = []

    # Prepare sample images for simulation if running offline
    clean_sample = "tests/sample_data/clean/Imgclean_0_0.jpg" if os.path.exists("tests/sample_data/clean/Imgclean_0_0.jpg") else None
    dusty_sample = "tests/sample_data/dusty/Imgdirty_1001_1.jpg" if os.path.exists("tests/sample_data/dusty/Imgdirty_1001_1.jpg") else None

    def on_waypoint_reached(seq: int, wp):
        if wp.action == "CAMERA_TRIGGER":
            telemetry = drone.get_telemetry()
            # In simulation, alternate clean and dusty samples to demonstrate realistic soiling detection
            sample_img = dusty_sample if (seq % 3 == 0 and dusty_sample) else clean_sample
            record = stream_proc.inspect_at_point(telemetry, sample_image_override=sample_img)
            inspections.append(record)
            print(f"  [Scan #{record.inspection_id:02d}] WP:{seq:02d} | Status: {record.detection.label:<5} "
                  f"(Conf: {record.detection.confidence*100:.1f}%) | "
                  f"GPS: ({record.lat:.6f}, {record.lon:.6f})")

    drone.register_waypoint_reached_callback(on_waypoint_reached)
    drone.upload_mission(waypoints)
    drone.arm_and_takeoff(args.altitude)
    drone.start_mission()
    drone.disconnect()

    # 4. Generate Reports
    print("\nGenerating Inspection Analytics & Geospatial Dashboard...")
    reporter = SolarInspectionReporter(inspections, polygon, site_name=args.site_name)
    stats = reporter.compute_summary_stats()

    json_path = os.path.join(args.output_dir, "inspection_report.json")
    csv_path = os.path.join(args.output_dir, "inspection_summary.csv")
    html_path = os.path.join(args.output_dir, "roof_inspection_map.html")

    reporter.export_json(json_path)
    reporter.export_csv(csv_path)
    reporter.generate_html_map(html_path)

    print("-" * 60)
    print(f"Total Panels Inspected   : {stats['total_inspected']}")
    print(f"Clean Panels             : {stats['clean_count']} ({stats['clean_ratio_pct']}%)")
    print(f"Dusty Panels             : {stats['dusty_count']} ({stats['soiling_ratio_pct']}%)")
    print(f"Est. Solar Power Loss    : -{stats['estimated_power_loss_pct']}%")
    print(f"Maintenance Priority     : {stats['urgency']}")
    print(f"Action Recommendation    : {stats['maintenance_recommendation']}")
    print("-" * 60)
    print(f"JSON Report   : {json_path}")
    print(f"CSV Summary   : {csv_path}")
    print(f"HTML Map View : {html_path}")
    print("=" * 60)


def cmd_classify(args):
    """Classifies a single image using the ResNet-18 detector."""
    detector = SolarDustDetector(model_weights_path=args.weights)
    result = detector.predict(args.image)
    print(f"Image: {args.image}")
    print(f"Prediction : {result.label} (is_dusty={result.is_dusty})")
    print(f"Confidence : {result.confidence * 100:.2f}%")
    print(f"Clean Prob : {result.clean_prob * 100:.2f}%")
    print(f"Dusty Prob : {result.dusty_prob * 100:.2f}%")


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Solar Roof Drone Scanner & Contamination Inspection System"
    )
    subparsers = parser.add_subparsers(dest="command", help="Available subcommands")

    # Command: plan
    p_plan = subparsers.add_parser("plan", help="Plan roof survey flight grid and export to drone formats")
    p_plan.add_argument("--polygon-file", type=str, default=None, help="JSON file containing [(lat, lon), ...]")
    p_plan.add_argument("--altitude", type=float, default=15.0, help="Flight altitude AGL in meters (default: 15.0)")
    p_plan.add_argument("--speed", type=float, default=3.0, help="Cruise speed in m/s (default: 3.0)")
    p_plan.add_argument("--forward-overlap", type=float, default=0.70, help="Forward overlap ratio (default: 0.70)")
    p_plan.add_argument("--side-overlap", type=float, default=0.60, help="Side overlap ratio (default: 0.60)")
    p_plan.add_argument("--setback", type=float, default=1.0, help="Inward roof buffer in meters (default: 1.0)")
    p_plan.add_argument("--azimuth", type=float, default=None, help="Roof panel row angle (deg). None for auto.")
    p_plan.add_argument("--mission-name", type=str, default="Solar_Roof_Mission", help="Mission identifier")
    p_plan.add_argument("--output-dir", type=str, default="outputs/planned_mission", help="Directory to save exported plans")

    # Command: scan
    p_scan = subparsers.add_parser("scan", help="Run automated inspection scan (virtual simulator or live MAVLink)")
    p_scan.add_argument("--mavlink", type=str, default=None, help="MAVLink connection string (e.g. 'udp:127.0.0.1:14550' or 'COM3'). None for simulator.")
    p_scan.add_argument("--weights", type=str, default="resnet18_solar_dust.pth", help="Path to PyTorch model weights")
    p_scan.add_argument("--altitude", type=float, default=15.0, help="Flight altitude AGL in meters")
    p_scan.add_argument("--speed", type=float, default=3.0, help="Flight speed in m/s")
    p_scan.add_argument("--site-name", type=str, default="Ankara Industrial Solar Roof", help="Site name for report")
    p_scan.add_argument("--output-dir", type=str, default="outputs/flight_run", help="Output directory for reports & snapshots")

    # Command: classify
    p_cls = subparsers.add_parser("classify", help="Classify solar panel dust on a single image")
    p_cls.add_argument("--image", type=str, required=True, help="Path to solar panel image")
    p_cls.add_argument("--weights", type=str, default="resnet18_solar_dust.pth", help="Path to model weights")

    args = parser.parse_args()

    if args.command == "plan":
        cmd_plan(args)
    elif args.command == "scan":
        cmd_scan(args)
    elif args.command == "classify":
        cmd_classify(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
