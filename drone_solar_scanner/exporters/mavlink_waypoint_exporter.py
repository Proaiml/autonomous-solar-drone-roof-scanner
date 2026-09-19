"""
MAVLink standard waypoint (.waypoints / .txt) exporter.
Converts survey waypoints into standard QGC WPL 110 format used by
Mission Planner, QGroundControl, and ArduPilot/PX4 ground stations.
"""

from typing import List
from ..planner.survey_planner import SurveyWaypoint


class MAVLinkWaypointExporter:
    """Exports waypoints to standard MAVLink 'QGC WPL 110' text format."""

    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3

    @classmethod
    def export_text(cls, waypoints: List[SurveyWaypoint]) -> str:
        """Converts waypoints into QGC WPL 110 text lines."""
        if not waypoints:
            raise ValueError("No waypoints to export.")

        lines = ["QGC WPL 110"]
        for idx, wp in enumerate(waypoints):
            current = 1 if idx == 0 else 0
            if wp.action == "TAKEOFF":
                cmd = cls.MAV_CMD_NAV_TAKEOFF
                p1, p2, p3, p4 = 0.0, 0.0, 0.0, float(wp.heading)
            elif wp.action == "RTL":
                cmd = cls.MAV_CMD_NAV_RETURN_TO_LAUNCH
                p1, p2, p3, p4 = 0.0, 0.0, 0.0, 0.0
            else:
                cmd = cls.MAV_CMD_NAV_WAYPOINT
                p1, p2, p3, p4 = 0.0, 1.0, 0.0, float(wp.heading)

            line = (
                f"{idx}\t{current}\t{cls.MAV_FRAME_GLOBAL_RELATIVE_ALT}\t{cmd}\t"
                f"{p1:.6f}\t{p2:.6f}\t{p3:.6f}\t{p4:.6f}\t"
                f"{wp.lat:.8f}\t{wp.lon:.8f}\t{wp.alt:.4f}\t1"
            )
            lines.append(line)

        return "\n".join(lines) + "\n"

    @classmethod
    def save_to_file(cls, waypoints: List[SurveyWaypoint], file_path: str) -> str:
        """Saves waypoints to a .waypoints / .txt file."""
        text = cls.export_text(waypoints)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(text)
        return file_path
