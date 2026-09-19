"""
MAVLink Drone Controller implementation using pymavlink.
Compatible with ArduPilot (Copter), PX4 Autopilot, Pixhawk, Cube, Herelink,
SiYi, Skydroid, and all MAVLink-compliant telemetry bridges.
"""

import time
import threading
from typing import List, Optional, Callable
from pymavlink import mavutil

from .base import BaseDroneController, DroneTelemetry
from ..planner.survey_planner import SurveyWaypoint


class MAVLinkDroneController(BaseDroneController):
    """
    Handles hardware communication with MAVLink-enabled commercial drones.
    Supports Serial (COM/TTY), UDP (14550), and TCP connections.
    """

    def __init__(self, target_system: int = 1, target_component: int = 1):
        self.master: Optional[mavutil.mavlink_connection] = None
        self.target_system = target_system
        self.target_component = target_component
        self.telemetry = DroneTelemetry()
        self.waypoints: List[SurveyWaypoint] = []
        self._running = False
        self._telemetry_thread: Optional[threading.Thread] = None
        self._wp_callback: Optional[Callable[[int, SurveyWaypoint], None]] = None

    def connect(self, connection_string: str = "udp:127.0.0.1:14550", baud: int = 57600, timeout: float = 10.0) -> bool:
        """
        Connect to drone telemetry link.
        Examples:
          - 'udp:127.0.0.1:14550' (QGroundControl / SITL / Herelink)
          - 'COM3' or '/dev/ttyUSB0' (Direct USB / SiYi / Telemetry Radio)
          - 'tcp:127.0.0.1:5760'
        """
        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            print(f"[MAVLink] Connecting to {connection_string} (waiting for heartbeat)...")

            # Wait for first heartbeat
            msg = self.master.wait_heartbeat(timeout=timeout)
            if not msg:
                print(f"[MAVLink] Warning: No heartbeat received from {connection_string} within {timeout}s.")
                return False

            self.target_system = self.master.target_system
            self.target_component = self.master.target_component
            self.telemetry.is_connected = True
            print(f"[MAVLink] Connected! System ID: {self.target_system}, Component ID: {self.target_component}")

            # Start background message listener
            self._running = True
            self._telemetry_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self._telemetry_thread.start()

            # Request data streams (position, attitude, status)
            self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000) # 10 Hz
            self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000)
            self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000) # 2 Hz

            return True
        except Exception as e:
            print(f"[MAVLink] Connection error: {e}")
            self.telemetry.is_connected = False
            return False

    def _request_message_interval(self, message_id: int, interval_us: int):
        """Sends MAV_CMD_SET_MESSAGE_INTERVAL to autopilot."""
        if not self.master:
            return
        try:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0
            )
        except Exception:
            pass

    def _listen_loop(self):
        """Processes incoming MAVLink messages and updates telemetry."""
        while self._running and self.master:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    continue

                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    self.telemetry.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.telemetry.flight_mode = mavutil.mode_string_v10(msg)

                elif msg_type == "GLOBAL_POSITION_INT":
                    self.telemetry.lat = msg.lat / 1e7
                    self.telemetry.lon = msg.lon / 1e7
                    self.telemetry.alt_msl = msg.alt / 1e3
                    self.telemetry.alt_relative = msg.relative_alt / 1e3
                    self.telemetry.heading = msg.hdg / 100.0
                    self.telemetry.ground_speed = (msg.vx**2 + msg.vy**2)**0.5 / 100.0

                elif msg_type == "ATTITUDE":
                    import math
                    self.telemetry.pitch = math.degrees(msg.pitch)
                    self.telemetry.roll = math.degrees(msg.roll)

                elif msg_type == "SYS_STATUS":
                    self.telemetry.battery_pct = float(msg.battery_remaining)
                    self.telemetry.battery_voltage = msg.voltage_battery / 1e3

                elif msg_type == "MISSION_ITEM_REACHED":
                    wp_seq = msg.seq
                    self.telemetry.current_wp_index = wp_seq
                    if self._wp_callback and 0 <= wp_seq < len(self.waypoints):
                        self._wp_callback(wp_seq, self.waypoints[wp_seq])

            except Exception:
                pass

    def disconnect(self) -> None:
        """Closes telemetry connection safely."""
        self._running = False
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=1.0)
        if self.master:
            self.master.close()
            self.master = None
        self.telemetry.is_connected = False
        print("[MAVLink] Telemetry link closed.")

    def arm_and_takeoff(self, target_altitude: float) -> bool:
        """Arms autopilot motors and initiates takeoff."""
        if not self.master:
            return False

        print(f"[MAVLink] Arming motors...")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = Arm
            0, 0, 0, 0, 0, 0
        )

        time.sleep(1.0)
        print(f"[MAVLink] Commanding takeoff to {target_altitude:.1f}m AGL...")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude
        )
        return True

    def upload_mission(self, waypoints: List[SurveyWaypoint]) -> bool:
        """
        Uploads survey waypoint list to drone via MAVLink mission protocol.
        Follows standard MISSION_COUNT -> MISSION_REQUEST -> MISSION_ITEM_INT -> MISSION_ACK.
        """
        if not self.master:
            return False

        self.waypoints = list(waypoints)
        count = len(self.waypoints)
        print(f"[MAVLink] Uploading {count} mission items to autopilot...")

        # 1. Clear existing mission
        self.master.mav.mission_clear_all_send(self.target_system, self.target_component)
        ack = self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3.0)

        # 2. Announce count
        self.master.mav.mission_count_send(self.target_system, self.target_component, count)

        # 3. Handle mission requests
        for _ in range(count + 5):
            msg = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], blocking=True, timeout=5.0)
            if not msg:
                print("[MAVLink] Upload timed out waiting for mission request.")
                return False

            if msg.get_type() == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print(f"[MAVLink] Mission successfully accepted by autopilot ({count} items)!")
                    return True
                else:
                    print(f"[MAVLink] Mission upload rejected! Code: {msg.type}")
                    return False

            seq = msg.seq
            if seq >= count:
                continue

            wp = self.waypoints[seq]
            # Map action to MAVLink command
            if wp.action == "TAKEOFF":
                cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            elif wp.action == "RTL":
                cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            else:
                cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            current = 1 if seq == 0 else 0

            self.master.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                seq,
                frame,
                cmd,
                current,
                1,  # autocontinue
                0.0, 1.0, 0.0, float(wp.heading),  # param1-4
                int(wp.lat * 1e7),
                int(wp.lon * 1e7),
                float(wp.alt)
            )

        # Wait for final ACK
        ack = self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3.0)
        return bool(ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED)

    def start_mission(self) -> bool:
        """Sets drone mode to AUTO to begin autonomous mission execution."""
        if not self.master:
            return False
        print("[MAVLink] Switching to AUTO mode...")
        # Try ArduPilot mode first, then standard MAVLink
        mode_id = self.master.mode_mapping().get('AUTO') if hasattr(self.master, 'mode_mapping') and self.master.mode_mapping() else None
        if mode_id is not None:
            self.master.set_mode(mode_id)
        else:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                3, # PX4 Auto Mission / ArduPilot Auto
                0, 0, 0, 0, 0
            )
        return True

    def return_to_launch(self) -> bool:
        """Sets drone mode to Return-To-Launch (RTL)."""
        if not self.master:
            return False
        print("[MAVLink] Emergency RTL commanded!")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        return True

    def get_telemetry(self) -> DroneTelemetry:
        return self.telemetry

    def register_waypoint_reached_callback(self, callback: Callable[[int, SurveyWaypoint], None]) -> None:
        self._wp_callback = callback
