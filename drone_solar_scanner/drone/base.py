"""
Base Drone Controller Interface and Telemetry Models.
Defines the universal abstraction for all commercial drone autopilots.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Callable
from ..planner.survey_planner import SurveyWaypoint


@dataclass
class DroneTelemetry:
    """Real-time drone state and navigation telemetry."""
    lat: float = 0.0
    lon: float = 0.0
    alt_msl: float = 0.0           # Altitude Above Mean Sea Level (m)
    alt_relative: float = 0.0      # Altitude Above Ground/Home (m)
    heading: float = 0.0           # Compass yaw degrees [0, 360)
    pitch: float = 0.0             # Gimbal / body pitch (deg)
    roll: float = 0.0              # Body roll (deg)
    ground_speed: float = 0.0      # Horizontal velocity (m/s)
    battery_pct: float = 100.0     # Remaining battery (0-100%)
    battery_voltage: float = 24.0  # Battery voltage (V)
    flight_mode: str = "UNKNOWN"
    is_armed: bool = False
    is_connected: bool = False
    current_wp_index: int = 0


class BaseDroneController(ABC):
    """Abstract interface for commercial drone autopilots (MAVLink, DJI SDK, Simulator)."""

    @abstractmethod
    def connect(self, connection_string: str) -> bool:
        """Establishes connection to drone or telemetry link."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Closes telemetry connection safely."""
        pass

    @abstractmethod
    def arm_and_takeoff(self, target_altitude: float) -> bool:
        """Arms propulsion system and climbs to safe initial survey altitude."""
        pass

    @abstractmethod
    def upload_mission(self, waypoints: List[SurveyWaypoint]) -> bool:
        """Uploads autonomous survey waypoints to the drone flight controller."""
        pass

    @abstractmethod
    def start_mission(self) -> bool:
        """Switches flight controller to AUTO mode to begin executing the mission."""
        pass

    @abstractmethod
    def return_to_launch(self) -> bool:
        """Commands drone to immediately abort and execute Return-To-Launch."""
        pass

    @abstractmethod
    def get_telemetry(self) -> DroneTelemetry:
        """Returns the latest synchronized telemetry snapshot."""
        pass

    @abstractmethod
    def register_waypoint_reached_callback(self, callback: Callable[[int, SurveyWaypoint], None]) -> None:
        """Registers a listener triggered whenever a survey waypoint is reached."""
        pass
