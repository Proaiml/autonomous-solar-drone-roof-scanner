"""
Comprehensive Solar Roof Inspection Reporter and Visual Dashboard Generator.
Outputs structured JSON, CSV, and interactive Leaflet GIS HTML maps with satellite imagery,
soiling ratios, power loss estimations, and maintenance recommendations.
"""

import os
import json
import csv
from typing import List, Tuple, Dict, Any
from ..vision.stream_processor import GeotaggedInspection


class SolarInspectionReporter:
    """
    Analyzes inspection records and generates analytics and interactive GIS reports.
    """

    def __init__(
        self,
        inspections: List[GeotaggedInspection],
        roof_polygon_gps: List[Tuple[float, float]],
        site_name: str = "Solar Roof Facility #1"
    ):
        self.inspections = inspections
        self.roof_polygon_gps = roof_polygon_gps
        self.site_name = site_name

    def compute_summary_stats(self) -> Dict[str, Any]:
        """Calculates contamination metrics, soiling ratio, and estimated power loss."""
        total = len(self.inspections)
        if total == 0:
            return {
                "total_inspected": 0,
                "clean_count": 0,
                "dusty_count": 0,
                "soiling_ratio_pct": 0.0,
                "clean_ratio_pct": 100.0,
                "avg_confidence": 0.0,
                "estimated_power_loss_pct": 0.0,
                "maintenance_recommendation": "No data available."
            }

        dusty_count = sum(1 for item in self.inspections if item.detection.is_dusty)
        clean_count = total - dusty_count
        soiling_ratio = (dusty_count / total) * 100.0
        clean_ratio = (clean_count / total) * 100.0
        avg_conf = sum(item.detection.confidence for item in self.inspections) / total

        # Industry standard: Dusty panels typically suffer 15-28% power derate
        est_power_loss = (dusty_count / total) * 22.5

        if soiling_ratio >= 35.0:
            recommendation = "CRITICAL CONTAMINATION: Immediate panel washing recommended to restore energy yield."
            urgency = "HIGH"
        elif soiling_ratio >= 15.0:
            recommendation = "MODERATE SOILING: Schedule maintenance cleaning within 7 to 14 days."
            urgency = "MEDIUM"
        else:
            recommendation = "OPTIMAL CONDITION: PV array operating normally. No immediate washing needed."
            urgency = "LOW"

        return {
            "total_inspected": total,
            "clean_count": clean_count,
            "dusty_count": dusty_count,
            "soiling_ratio_pct": round(soiling_ratio, 2),
            "clean_ratio_pct": round(clean_ratio, 2),
            "avg_confidence": round(avg_conf, 4),
            "estimated_power_loss_pct": round(est_power_loss, 2),
            "urgency": urgency,
            "maintenance_recommendation": recommendation
        }

    def export_json(self, file_path: str) -> str:
        """Saves detailed JSON inspection report."""
        stats = self.compute_summary_stats()
        data = {
            "site_name": self.site_name,
            "summary": stats,
            "roof_boundary": [{"lat": lat, "lon": lon} for lat, lon in self.roof_polygon_gps],
            "inspections": [item.to_dict() for item in self.inspections]
        }
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        return file_path

    def export_csv(self, file_path: str) -> str:
        """Saves inspection points table to CSV."""
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        with open(file_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Inspection_ID", "Timestamp", "Latitude", "Longitude",
                "Altitude_m", "Heading_deg", "Classification", "Confidence",
                "Clean_Prob", "Dusty_Prob", "Snapshot_File"
            ])
            for item in self.inspections:
                writer.writerow([
                    item.inspection_id,
                    item.timestamp,
                    f"{item.lat:.7f}",
                    f"{item.lon:.7f}",
                    f"{item.alt:.2f}",
                    f"{item.heading:.1f}",
                    item.detection.label,
                    f"{item.detection.confidence:.4f}",
                    f"{item.detection.clean_prob:.4f}",
                    f"{item.detection.dusty_prob:.4f}",
                    os.path.basename(item.snapshot_path) if item.snapshot_path else ""
                ])
        return file_path

    def generate_html_map(self, file_path: str) -> str:
        """Generates a standalone, interactive Leaflet GIS dashboard map."""
        stats = self.compute_summary_stats()
        center_lat = self.roof_polygon_gps[0][0] if self.roof_polygon_gps else 39.92
        center_lon = self.roof_polygon_gps[0][1] if self.roof_polygon_gps else 32.85

        roof_poly_js = json.dumps([[lat, lon] for lat, lon in self.roof_polygon_gps])
        flight_path_js = json.dumps([[item.lat, item.lon] for item in self.inspections])

        markers_data = []
        for item in self.inspections:
            markers_data.append({
                "id": item.inspection_id,
                "lat": item.lat,
                "lon": item.lon,
                "label": item.detection.label,
                "conf": round(item.detection.confidence * 100, 1),
                "is_dusty": item.detection.is_dusty,
                "img": os.path.basename(item.snapshot_path) if item.snapshot_path else ""
            })
        markers_js = json.dumps(markers_data)

        badge_color = "#ef4444" if stats["urgency"] == "HIGH" else ("#f59e0b" if stats["urgency"] == "MEDIUM" else "#10b981")

        html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Autonomous Solar Drone Roof Inspection Report</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0f172a; color: #f8fafc; height: 100vh; display: flex; flex-direction: column; }}
    header {{ background: #1e293b; padding: 16px 24px; border-bottom: 1px solid #334155; display: flex; justify-content: space-between; align-items: center; }}
    .title-group h1 {{ font-size: 20px; font-weight: 700; color: #38bdf8; }}
    .title-group p {{ font-size: 13px; color: #94a3b8; margin-top: 2px; }}
    .stats-bar {{ display: flex; gap: 16px; background: #0f172a; padding: 12px 24px; border-bottom: 1px solid #1e293b; overflow-x: auto; }}
    .stat-card {{ background: #1e293b; padding: 10px 18px; border-radius: 8px; border: 1px solid #334155; min-width: 140px; }}
    .stat-label {{ font-size: 11px; text-transform: uppercase; color: #94a3b8; letter-spacing: 0.5px; }}
    .stat-val {{ font-size: 20px; font-weight: 700; margin-top: 4px; }}
    .status-badge {{ display: inline-block; padding: 4px 10px; border-radius: 9999px; font-size: 12px; font-weight: 600; background: {badge_color}; color: #ffffff; }}
    #map {{ flex: 1; width: 100%; }}
    .legend {{ background: rgba(15, 23, 42, 0.85); backdrop-filter: blur(8px); padding: 12px 16px; border-radius: 8px; border: 1px solid #334155; color: #f8fafc; font-size: 12px; }}
    .legend-item {{ display: flex; align-items: center; gap: 8px; margin-bottom: 6px; }}
    .legend-dot {{ width: 12px; height: 12px; border-radius: 50%; display: inline-block; }}
  </style>
</head>
<body>
  <header>
    <div class="title-group">
      <h1>Solar Roof AI Inspection Dashboard</h1>
      <p>Site: {self.site_name} | Universal Drone Autonomous Scan</p>
    </div>
    <div>
      <span class="status-badge">{stats['urgency']} PRIORITY</span>
    </div>
  </header>

  <div class="stats-bar">
    <div class="stat-card">
      <div class="stat-label">Total Panels</div>
      <div class="stat-val" style="color: #38bdf8;">{stats['total_inspected']}</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Clean Panels</div>
      <div class="stat-val" style="color: #10b981;">{stats['clean_count']}</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Dusty / Soiled</div>
      <div class="stat-val" style="color: #ef4444;">{stats['dusty_count']}</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Soiling Ratio</div>
      <div class="stat-val" style="color: #f59e0b;">{stats['soiling_ratio_pct']}%</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Est. Power Loss</div>
      <div class="stat-val" style="color: #f43f5e;">-{stats['estimated_power_loss_pct']}%</div>
    </div>
    <div class="stat-card" style="flex: 1;">
      <div class="stat-label">Action Recommendation</div>
      <div style="font-size: 13px; font-weight: 500; margin-top: 6px; color: #cbd5e1;">{stats['maintenance_recommendation']}</div>
    </div>
  </div>

  <div id="map"></div>

  <script>
    var map = L.map('map').setView([{center_lat}, {center_lon}], 19);

    // OpenStreetMap & Satellite base layers
    var osm = L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
      maxZoom: 21,
      attribution: '© OpenStreetMap contributors'
    }}).addTo(map);

    var satellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
      maxZoom: 21,
      attribution: 'Esri Satellite'
    }});

    L.control.layers({{"OpenStreetMap": osm, "Esri Satellite": satellite}}).addTo(map);

    // Draw Roof Polygon
    var roofCoords = {roof_poly_js};
    if (roofCoords.length > 0) {{
      var roofPolygon = L.polygon(roofCoords, {{
        color: '#f59e0b',
        weight: 3,
        fillColor: '#f59e0b',
        fillOpacity: 0.15
      }}).addTo(map);
      map.fitBounds(roofPolygon.getBounds(), {{ padding: [30, 30] }});
    }}

    // Draw Drone Flight Path
    var flightPath = {flight_path_js};
    if (flightPath.length > 0) {{
      L.polyline(flightPath, {{
        color: '#0284c7',
        weight: 2,
        dashArray: '4, 4'
      }}).addTo(map);
    }}

    // Inspection Markers
    var markers = {markers_js};
    markers.forEach(function(m) {{
      var markerColor = m.is_dusty ? '#ef4444' : '#10b981';
      var marker = L.circleMarker([m.lat, m.lon], {{
        radius: 6,
        fillColor: markerColor,
        color: '#ffffff',
        weight: 1.5,
        opacity: 1,
        fillOpacity: 0.9
      }}).addTo(map);

      var popupHtml = '<div style="font-family: sans-serif; min-width: 180px;">' +
        '<b style="color: ' + markerColor + ';">' + m.label.toUpperCase() + ' PANEL</b><br>' +
        '<span>Confidence: ' + m.conf + '%</span><br>' +
        '<span>GPS: ' + m.lat.toFixed(6) + ', ' + m.lon.toFixed(6) + '</span><br>' +
        (m.img ? '<img src="' + m.img + '" style="width: 100%; border-radius: 4px; margin-top: 6px; border: 1px solid #ccc;"/>' : '') +
        '</div>';

      marker.bindPopup(popupHtml);
    }});

    // Legend
    var legend = L.control({{ position: 'bottomright' }});
    legend.onAdd = function(map) {{
      var div = L.DomUtil.create('div', 'legend');
      div.innerHTML = '<strong>Inspection Legend</strong><br>' +
        '<div class="legend-item"><span class="legend-dot" style="background:#10b981;"></span> Clean Panel</div>' +
        '<div class="legend-item"><span class="legend-dot" style="background:#ef4444;"></span> Dusty / Soiled Panel</div>' +
        '<div class="legend-item"><span class="legend-dot" style="background:#f59e0b;"></span> Roof Boundary</div>' +
        '<div class="legend-item"><span class="legend-dot" style="background:#0284c7;"></span> Autonomous Flight Grid</div>';
      return div;
    }};
    legend.addTo(map);
  </script>
</body>
</html>
"""
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(html_content)
        return file_path
