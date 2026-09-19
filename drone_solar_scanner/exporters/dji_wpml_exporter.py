"""
DJI Pilot 2 / WPML (Waypoint Mission Language) KML Mission Exporter.
Produces official DJI KML 2.2 mission files compatible with DJI Matrice 300/350 RTK,
Mavic 3 Enterprise, Mavic 3 Thermal, Matrice 30 series, and Google Earth.
"""

from typing import List
from xml.sax.saxutils import escape
from ..planner.survey_planner import SurveyWaypoint


class DJIWPMLExporter:
    """Exports survey waypoints to DJI Pilot 2 WPML / KML 2.2 format."""

    @classmethod
    def export_kml(cls, waypoints: List[SurveyWaypoint], mission_name: str = "Solar_Roof_Survey") -> str:
        """Generates standard DJI WPML XML string."""
        if not waypoints:
            raise ValueError("No waypoints to export.")

        # Filter survey and trigger waypoints (excluding pure RTL, as DJI has finishAction)
        nav_points = [wp for wp in waypoints if wp.action != "RTL"]
        if not nav_points:
            nav_points = waypoints

        cruise_speed = nav_points[0].speed if nav_points else 3.0
        safe_name = escape(mission_name)

        # Build coordinate string for LineString
        coords_str = " ".join([f"{wp.lon:.7f},{wp.lat:.7f},{wp.alt:.1f}" for wp in nav_points])

        placemarks_xml = []
        for idx, wp in enumerate(nav_points):
            placemark = f"""      <Placemark>
        <Point>
          <coordinates>{wp.lon:.7f},{wp.lat:.7f}</coordinates>
        </Point>
        <wpml:index>{idx}</wpml:index>
        <wpml:executeHeight>{wp.alt:.2f}</wpml:executeHeight>
        <wpml:waypointSpeed>{wp.speed:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>manually</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>{int(wp.heading)}</wpml:waypointHeadingAngle>
        </wpml:waypointHeadingParam>
        <wpml:gimbalPitch>{wp.gimbal_pitch:.1f}</wpml:gimbalPitch>
        <wpml:actionGroup>
          <wpml:actionGroupId>{idx}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{idx}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{idx}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
          </wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
          </wpml:action>
        </wpml:actionGroup>
      </Placemark>"""
            placemarks_xml.append(placemark)

        all_placemarks = "\n".join(placemarks_xml)

        kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.6">
  <Document>
    <name>{safe_name}</name>
    <wpml:author>AutonomousSolarDroneScanner</wpml:author>
    <wpml:createTime>2026-09-19T12:00:00Z</wpml:createTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>20</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{cruise_speed:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>67</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
      </wpml:droneInfo>
    </wpml:missionConfig>
    <Folder>
      <name>Waypoints</name>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:autoFlightSpeed>{cruise_speed:.1f}</wpml:autoFlightSpeed>
      <Placemark>
        <name>FlightPath</name>
        <LineString>
          <coordinates>{coords_str}</coordinates>
        </LineString>
      </Placemark>
{all_placemarks}
    </Folder>
  </Document>
</kml>
"""
        return kml_content

    @classmethod
    def save_to_file(cls, waypoints: List[SurveyWaypoint], file_path: str, mission_name: str = "Solar_Roof_Survey") -> str:
        """Saves waypoints to a DJI KML / WPML file."""
        content = cls.export_kml(waypoints, mission_name=mission_name)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(content)
        return file_path
