"""
QGroundControl (.plan) Mission Exporter.
Converts generated survey waypoints to official QGC Plan JSON format (v1.0),
compatible with QGroundControl, Auterion Mission Control, PX4, and ArduPilot.
"""

import json
from typing import List, Dict, Any
from ..planner.survey_planner import SurveyWaypoint


class QGCPlanExporter:
    """Exports survey waypoints to QGroundControl .plan JSON format."""

    # Standard MAVLink Commands
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_DO_MOUNT_CONTROL = 205
    MAV_CMD_DO_CHANGE_SPEED = 178
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

    @classmethod
    def export(cls, waypoints: List[SurveyWaypoint], cruise_speed: float = 3.0) -> Dict[str, Any]:
        """
        Converts waypoints into a QGC .plan dictionary.
        """
        if not waypoints:
            raise ValueError("No waypoints to export.")

        home_wp = waypoints[0]
        mission_items: List[Dict[str, Any]] = []

        for idx, wp in enumerate(waypoints):
            if wp.action == "TAKEOFF":
                cmd = cls.MAV_CMD_NAV_TAKEOFF
                p1, p2, p3, p4 = 0.0, 0.0, 0.0, float(wp.heading)
            elif wp.action == "RTL":
                cmd = cls.MAV_CMD_NAV_RETURN_TO_LAUNCH
                p1, p2, p3, p4 = 0.0, 0.0, 0.0, 0.0
            else:
                cmd = cls.MAV_CMD_NAV_WAYPOINT
                p1, p2, p3, p4 = 0.0, 1.0, 0.0, float(wp.heading)  # p2 = acceptance radius 1m

            item = {
                "autoContinue": True,
                "command": cmd,
                "doJumpId": idx + 1,
                "frame": cls.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                "params": [p1, p2, p3, p4, float(wp.lat), float(wp.lon), float(wp.alt)],
                "type": "SimpleItem"
            }
            mission_items.append(item)

        plan = {
            "fileType": "Plan",
            "version": 1,
            "groundStation": "AutonomousSolarDroneScanner",
            "mission": {
                "cruiseSpeed": cruise_speed,
                "hoverSpeed": 1.5,
                "plannedHomePosition": [float(home_wp.lat), float(home_wp.lon), 0.0],
                "vehicleType": 2,  # Multi-Rotor
                "items": mission_items
            }
        }
        return plan

    @classmethod
    def save_to_file(cls, waypoints: List[SurveyWaypoint], file_path: str, cruise_speed: float = 3.0) -> str:
        """Saves waypoints directly to a .plan JSON file."""
        plan_dict = cls.export(waypoints, cruise_speed=cruise_speed)
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(plan_dict, f, indent=2)
        return file_path
