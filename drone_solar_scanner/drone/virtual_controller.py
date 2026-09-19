"""
Virtual Drone Controller for Hardware-in-the-Loop (HIL) and Software Simulation.
Accurately simulates kinematic drone flight, telemetry updates, battery depletion,
and waypoint triggers for end-to-end system testing.
"""

import time
import math
from typing import List, Optional, Callable
from .base import BaseDroneController, DroneTelemetry
from ..planner.survey_planner import SurveyWaypoint
from ..planner.geodesy import haversine_distance, calculate_bearing


class VirtualDroneController(BaseDroneController):
    """Simulates drone flight dynamics and telemetry for offline testing and verification."""

    def __init__(self, home_lat: float = 39.92077, home_lon: float = 32.85411):
        self.telemetry = DroneTelemetry(
            lat=home_lat,
            lon=home_lon,
            alt_msl=100.0,
            alt_relative=0.0,
            heading=0.0,
            ground_speed=0.0,
            battery_pct=100.0,
            battery_voltage=25.2,
            flight_mode="DISARMED",
            is_armed=False,
            is_connected=False
        )
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.waypoints: List[SurveyWaypoint] = []
        self._wp_callback: Optional[Callable[[int, SurveyWaypoint], None]] = None
        self._is_flying = False

    def connect(self, connection_string: str = "virtual:sitl") -> bool:
        self.telemetry.is_connected = True
        self.telemetry.flight_mode = "STANDBY"
        print(f"[VirtualDrone] Simulator connected to {connection_string}.")
        return True

    def disconnect(self) -> None:
        self._is_flying = False
        self.telemetry.is_connected = False
        print("[VirtualDrone] Simulator disconnected.")

    def arm_and_takeoff(self, target_altitude: float) -> bool:
        if not self.telemetry.is_connected:
            return False
        self.telemetry.is_armed = True
        self.telemetry.flight_mode = "TAKEOFF"
        print(f"[VirtualDrone] Motors ARMED. Climbing to {target_altitude:.1f}m...")
        self.telemetry.alt_relative = target_altitude
        self.telemetry.alt_msl = 100.0 + target_altitude
        self.telemetry.flight_mode = "GUIDED"
        return True

    def upload_mission(self, waypoints: List[SurveyWaypoint]) -> bool:
        if not self.telemetry.is_connected:
            return False
        self.waypoints = list(waypoints)
        print(f"[VirtualDrone] Uploaded {len(self.waypoints)} mission waypoints to virtual autopilot.")
        return True

    def start_mission(self, sim_speed_multiplier: float = 1.0) -> bool:
        """
        Executes the uploaded mission sequentially, updating telemetry
        and triggering waypoint inspection callbacks.
        """
        if not self.telemetry.is_connected or not self.waypoints:
            return False

        self._is_flying = True
        self.telemetry.flight_mode = "AUTO"
        print(f"[VirtualDrone] Starting autonomous survey mission ({len(self.waypoints)} waypoints)...")

        for idx, wp in enumerate(self.waypoints):
            if not self._is_flying:
                print("[VirtualDrone] Mission aborted.")
                break

            self.telemetry.current_wp_index = idx

            # Calculate movement to this waypoint
            dist = haversine_distance(self.telemetry.lat, self.telemetry.lon, wp.lat, wp.lon)
            bearing = calculate_bearing(self.telemetry.lat, self.telemetry.lon, wp.lat, wp.lon) if dist > 0.1 else self.telemetry.heading

            self.telemetry.heading = wp.heading if wp.heading is not None else bearing
            self.telemetry.lat = wp.lat
            self.telemetry.lon = wp.lon
            self.telemetry.alt_relative = wp.alt
            self.telemetry.ground_speed = wp.speed

            # Simulate battery drain (approx 0.1% per waypoint)
            self.telemetry.battery_pct = max(5.0, self.telemetry.battery_pct - 0.15)
            self.telemetry.battery_voltage = 21.0 + (self.telemetry.battery_pct / 100.0) * 4.2

            # Trigger inspection action callback
            if self._wp_callback:
                self._wp_callback(idx, wp)

        print("[VirtualDrone] Autonomous mission completed successfully.")
        self.telemetry.flight_mode = "RTL"
        self.telemetry.lat = self.home_lat
        self.telemetry.lon = self.home_lon
        self.telemetry.alt_relative = 0.0
        self.telemetry.is_armed = False
        self.telemetry.flight_mode = "LANDED"
        return True

    def return_to_launch(self) -> bool:
        print("[VirtualDrone] RTL activated.")
        self._is_flying = False
        self.telemetry.flight_mode = "RTL"
        self.telemetry.lat = self.home_lat
        self.telemetry.lon = self.home_lon
        self.telemetry.alt_relative = 0.0
        self.telemetry.is_armed = False
        return True

    def get_telemetry(self) -> DroneTelemetry:
        return self.telemetry

    def register_waypoint_reached_callback(self, callback: Callable[[int, SurveyWaypoint], None]) -> None:
        self._wp_callback = callback
