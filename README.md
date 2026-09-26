# Universal Commercial Drone Autonomous Solar Roof Scanner & Inspection System

[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.5+-ee4c2c.svg)](https://pytorch.org/)
[![MAVLink](https://img.shields.io/badge/MAVLink-v2.0-orange.svg)](https://mavlink.io/)
[![DJI WPML](https://img.shields.io/badge/DJI-Pilot%202%20WPML-green.svg)](https://developer.dji.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

An end-to-end system for autonomous rooftop solar panel inspection using commercial drones. Integrates geodetic boustrophedon grid path planning, universal autopilot communication (MAVLink, DJI Pilot 2 WPML/KML, QGroundControl, Mission Planner), and a deep learning PyTorch ResNet-18 model (`resnet18_solar_dust.pth`) for real-time solar panel dust and contamination detection.

> **Status:** the planner, exporters, MAVLink link and a full simulated mission are covered by 14 automated tests. Real flights, mission import on DJI controllers and the model's field accuracy have not been validated yet — see [Validation status and limits](#validation-status-and-limits).

---

## Key Features

1. **Broad Drone Compatibility (by protocol)**:
   - **MAVLink Protocol**: Works with MAVLink autopilots (ArduPilot, PX4 on Pixhawk, Cube, Holybro boards) and MAVLink radio links (e.g. Herelink, SiYi, Skydroid) via USB Serial (`COMx`, `/dev/ttyUSB0`), UDP (`14550`), or TCP. Individual hardware models have not been tested one by one.
   - **DJI Pilot 2 (WPML / KML 2.2)**: Generates a KML mission with WPML waypoint tags intended for DJI Matrice 300/350 RTK, Mavic 3 Enterprise/Thermal and Matrice 30. DJI Pilot 2 imports KMZ packages (`wpmz/template.kml` + `waylines.wpml`); packaging and import on a real controller are not yet verified.
   - **QGroundControl / Mission Planner (`.plan` / `.waypoints`)**: Native JSON and WPL 110 format export.
   - **GIS GeoJSON**: Standard RFC 7946 feature collections for QGIS, ArcGIS, and Leaflet.
   - **High-Fidelity Flight Simulator**: Built-in kinematic simulator for hardware-free end-to-end testing.

2. **Autonomous Rooftop Path Planner**:
   - Computes boustrophedon (lawnmower) survey tracks from GPS roof boundary polygons.
   - Exact local ENU (East-North-Up) metric tangent plane projection via WGS84 ellipsoid geodetics.
   - Automatic solar panel row alignment (azimuth orientation) to minimize flight turns and maximize coverage.
   - Inward safety setback buffering and obstacle margins.
   - Longitudinal and lateral ground footprint calculations based on camera FOV and overlap ratios (e.g. 70% forward, 60% side overlap).

3. **Deep Learning Solar Panel Dust Detector**:
   - PyTorch ResNet-18 architecture with custom MLP classification head.
   - Binary classification: `Clean` vs `Dusty` with softmax confidence scores.
   - Real-time video ingestion from RTSP (DJI Livestream / SiYi / Herelink), USB/HDMI capture cards, or local feeds.
   - Geotagging engine syncing every camera trigger with instantaneous drone latitude, longitude, altitude, and heading.

4. **Analytics & Interactive GIS Dashboard**:
   - Calculates soiling ratio (%), estimated solar power output derate ($P_{loss}$), and cleaning urgency levels (`LOW`, `MEDIUM`, `HIGH`).
   - Exports structured JSON reports, CSV inspection sheets, and standalone interactive Leaflet GIS HTML maps with satellite imagery and photo popups.

---

## Architecture

```
                                  Roof Polygon GPS [(lat, lon), ...]
                                                  │
                                                  ▼
                                      ┌───────────────────────┐
                                      │   RoofSurveyPlanner   │
                                      │ (Boustrophedon Grid)  │
                                      └───────────┬───────────┘
                                                  │
             ┌────────────────────────────────────┼────────────────────────────────────┐
             ▼                                    ▼                                    ▼
┌─────────────────────────┐          ┌─────────────────────────┐          ┌─────────────────────────┐
│     QGCPlanExporter     │          │     DJIWPMLExporter     │          │ MAVLinkWaypointExporter │
│   (.plan JSON v1.0)     │          │  (DJI Pilot 2 WPML/KML) │          │     (QGC WPL 110)       │
└─────────────────────────┘          └─────────────────────────┘          └─────────────────────────┘
             │                                    │                                    │
             └────────────────────────────────────┼────────────────────────────────────┘
                                                  ▼
                                      ┌───────────────────────┐
                                      │ BaseDroneController   │
                                      │ (MAVLink / Virtual)   │
                                      └───────────┬───────────┘
                                                  │ Telemetry Stream (Lat, Lon, Alt, Heading)
                                                  ▼
                                      ┌───────────────────────┐
                                      │ VideoStreamProcessor  │◄──── RTSP / HDMI / USB Camera
                                      │  + SolarDustDetector  │
                                      │ (PyTorch ResNet-18)   │
                                      └───────────┬───────────┘
                                                  │ Geotagged Inspections
                                                  ▼
                                      ┌───────────────────────┐
                                      │ SolarInspectionReport │
                                      │ (JSON, CSV, HTML Map) │
                                      └───────────────────────┘
```

---

## Installation

```bash
# Clone the repository
git clone https://github.com/Proaiml/autonomous-solar-drone-roof-scanner.git
cd autonomous-solar-drone-roof-scanner

# Install dependencies
pip install -r requirements.txt
```

---

## Quick Start (CLI)

### 1. Plan a Roof Survey Mission & Export Formats
Generate optimal boustrophedon waypoints for a rooftop and export to QGC `.plan`, DJI `.kml`, MAVLink `.waypoints`, and `.geojson`:

```bash
python cli.py plan --altitude 15.0 --speed 3.0 --forward-overlap 0.70 --side-overlap 0.60 --output-dir outputs/planned_mission
```

Output files created in `outputs/planned_mission/`:
- `survey_mission.plan` (QGroundControl / Auterion / PX4)
- `dji_pilot2_mission.kml` (DJI Pilot 2 on M300/M350 RTK, Mavic 3 Enterprise)
- `mission_items.waypoints` (Mission Planner / ArduPilot)
- `survey_tracks.geojson` (GIS layers)

### 2. Run Autonomous Inspection Flight & Generate AI Report
Execute simulated or live inspection scan across the rooftop:

```bash
# Offline simulation run
python cli.py scan --altitude 15.0 --speed 3.0 --output-dir outputs/flight_run

# Live MAVLink drone connection (e.g. over telemetry radio COM port or companion UDP)
python cli.py scan --mavlink udp:127.0.0.1:14550 --altitude 15.0 --speed 3.0 --output-dir outputs/flight_run
```

Output artifacts generated:
- `outputs/flight_run/inspection_report.json`: Detailed analytical metrics and contamination breakdown.
- `outputs/flight_run/inspection_summary.csv`: Tabular spreadsheet of every panel inspection coordinate.
- `outputs/flight_run/roof_inspection_map.html`: Interactive Leaflet GIS dashboard with satellite basemap, color-coded panel markers, and inspection photo popups.
- `outputs/flight_run/snapshots/`: Geotagged annotated panel images.

### 3. Classify a Single Solar Panel Image
```bash
python cli.py classify --image tests/sample_data/dusty/Imgdirty_1001_1.jpg
```

---

## Python API Usage

```python
from drone_solar_scanner.planner import RoofSurveyPlanner, SurveyConfig
from drone_solar_scanner.vision import SolarDustDetector
from drone_solar_scanner.drone import VirtualDroneController
from drone_solar_scanner.reporting import SolarInspectionReporter

# 1. Define Roof Boundary
roof_polygon = [
    (39.9208, 32.8541),
    (39.9212, 32.8541),
    (39.9212, 32.8547),
    (39.9208, 32.8547)
]

# 2. Plan Mission
planner = RoofSurveyPlanner(config=SurveyConfig(altitude_agl=15.0, cruise_speed=3.0))
waypoints = planner.plan_mission(roof_polygon)

# 3. Initialize AI Model
detector = SolarDustDetector(model_weights_path="resnet18_solar_dust.pth")

# 4. Connect to Drone & Execute Mission
drone = VirtualDroneController()
drone.connect()
drone.upload_mission(waypoints)
drone.start_mission()
```

---

## Training the Dust Model

`main.py` trains the classifier that `resnet18_solar_dust.pth` comes from:

- Data: an `ImageFolder` directory with one sub-folder per class (`Clean`, `Dusty`); set the dataset and output paths at the top and bottom of `main.py`.
- Split: random 80% train / 20% test.
- Model: ImageNet-pretrained ResNet-18 with frozen backbone; only the new head (512 → 50 → 20 → 10 → 2, ReLU and Dropout 0.3) is trained.
- Training: Adam (lr 1e-3, weight decay 1e-4), batch 64, 20 epochs, cross-entropy; inputs resized to 224×224 with ImageNet normalization.
- The script plots train and test loss. It does not report accuracy; measure precision/recall on a held-out set before trusting the detector in the field.

```bash
python main.py
```

## Verification & Test Suite

The codebase includes an automated test suite (14 tests, about 20 s on a CPU) covering geodetic math, ResNet-18 model weights, mission exporters, MAVLink packet exchange, and full end-to-end simulation.

To run tests:
```bash
pytest -v tests/
```

Test coverage:
- `test_geodesy_and_planner.py`: Verifies WGS84 / ENU conversions, Haversine distances, azimuth bearings, and boustrophedon sweep slicing.
- `test_model_inference.py`: Verifies `resnet18_solar_dust.pth` weights loading, class probabilities, and inference on real sample images.
- `test_exporters.py`: Validates schema compliance of QGC `.plan` (JSON), DJI WPML (XML), MAVLink WPL 110, and GeoJSON.
- `test_mavlink_comm.py`: Tests `pymavlink` telemetry packet exchange and heartbeat handshake.
- `test_end_to_end_mission.py`: Full mission integration test validating waypoint navigation, frame acquisition, snapshot generation, and HTML map output.

---

## Validation status and limits

| Area | Status |
|---|---|
| Survey geometry (WGS84/ENU, footprint, sweep lines, setback) | Automated tests |
| QGC `.plan`, MAVLink WPL 110, GeoJSON export | Schema tests |
| DJI WPML/KML export | XML is generated and checked; KMZ packaging and import on a DJI controller not verified |
| MAVLink link | Heartbeat and telemetry exchange tested with `pymavlink`; no flight on real hardware yet |
| Full mission | End-to-end test in the built-in simulator (navigation, frame capture, snapshots, HTML map) |
| Dust model | Loads and classifies the six sample images; training data and accuracy on an independent test set are not documented |

Before a real flight: check the exported mission in the ground station, fly the first mission in an open area with a pilot ready to take over, and confirm local drone regulations. The detector's output should be reviewed by a person until its accuracy has been measured on images from your own site and camera.

## License
MIT License. Developed by İlhan Koçaslan.
