"""
Integration test for MAVLink protocol communication and telemetry exchange.
Sets up an in-memory / local UDP connection using pymavlink.
"""

import time
import threading
from pymavlink import mavutil
from drone_solar_scanner.drone.mavlink_controller import MAVLinkDroneController


def simulated_autopilot_server(port: int = 14555, stop_event: threading.Event = None):
    """
    Background thread simulating a drone flight controller (ArduPilot/PX4)
    streaming MAVLink telemetry over UDP to the Ground Station port.
    """
    # Drone sends outbound UDP to port 14555
    server = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
    boot_time = time.time()

    while not stop_event.is_set():
        ms_since_boot = int((time.time() - boot_time) * 1000) % 4000000000

        # 1. Send Heartbeat (ArduCopter, Multi-Rotor, Guided/Auto, Armed)
        server.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED | mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            3,  # Custom mode Auto
            mavutil.mavlink.MAV_STATE_ACTIVE
        )

        # 2. Send Global Position Telemetry
        server.mav.global_position_int_send(
            ms_since_boot,
            int(39.9208 * 1e7),
            int(32.8541 * 1e7),
            int(115.0 * 1e3),
            int(15.0 * 1e3),
            300, 0, 0,
            9000  # 90.00 deg heading
        )

        # 3. Send System Status (Battery)
        server.mav.sys_status_send(
            0, 0, 0, 500, 24500, -1, 88, 0, 0, 0, 0, 0, 0
        )

        time.sleep(0.05)

    server.close()


def test_mavlink_connection_and_telemetry():
    """Verifies that MAVLinkDroneController establishes connection and parses telemetry packets."""
    port = 14555
    stop_event = threading.Event()
    server_thread = threading.Thread(target=simulated_autopilot_server, args=(port, stop_event), daemon=True)
    server_thread.start()

    time.sleep(0.1)

    # Controller listens on inbound UDP port
    client = MAVLinkDroneController()
    connected = client.connect(f"udpin:127.0.0.1:{port}", timeout=3.0)

    assert connected, "MAVLink controller should connect to the local autopilot simulator."
    assert client.telemetry.is_connected

    # Allow telemetry packets to be processed by background thread
    time.sleep(0.4)

    telem = client.get_telemetry()
    assert telem.is_connected
    assert telem.is_armed
    assert abs(telem.lat - 39.9208) < 1e-4
    assert abs(telem.lon - 32.8541) < 1e-4
    assert abs(telem.alt_relative - 15.0) < 1e-1
    assert abs(telem.heading - 90.0) < 1.0
    assert telem.battery_pct == 88.0

    client.disconnect()
    stop_event.set()
    server_thread.join(timeout=1.0)
