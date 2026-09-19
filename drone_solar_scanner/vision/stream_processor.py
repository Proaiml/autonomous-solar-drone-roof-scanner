"""
Drone Video Stream Capture and Geotagging Processor.
Ingests live RTSP (DJI/SiYi/Herelink), USB/HDMI capture cards, or simulation images,
and pairs detections with real-time drone geodetic telemetry.
"""

import os
import time
from dataclasses import dataclass, asdict
from typing import Optional, Union, List, Dict, Any
import cv2
import numpy as np
from PIL import Image

from .detector import SolarDustDetector, DetectionResult
from ..drone.base import DroneTelemetry


@dataclass
class GeotaggedInspection:
    """Inspection record combining drone position and AI classification result."""
    inspection_id: int
    timestamp: float
    lat: float
    lon: float
    alt: float
    heading: float
    detection: DetectionResult
    snapshot_path: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["detection"] = self.detection.to_dict()
        return data


class VideoStreamProcessor:
    """
    Manages camera frame acquisition and geotagged solar panel inspection.
    """

    def __init__(self, detector: SolarDustDetector, output_dir: str = "outputs/inspections"):
        self.detector = detector
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.cap: Optional[cv2.VideoCapture] = None
        self._counter = 0

    def open_stream(self, source: Union[str, int]) -> bool:
        """
        Opens RTSP stream URL (e.g. 'rtsp://192.168.1.100:8554/live') or camera index (e.g. 0).
        """
        try:
            self.cap = cv2.VideoCapture(source)
            if not self.cap.isOpened():
                print(f"[VideoStream] Could not open video source: {source}")
                return False
            print(f"[VideoStream] Connected to stream: {source}")
            return True
        except Exception as e:
            print(f"[VideoStream] Error opening stream: {e}")
            return False

    def close_stream(self):
        """Closes video stream."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None

    def inspect_at_point(
        self,
        telemetry: DroneTelemetry,
        sample_image_override: Optional[Union[str, Image.Image, np.ndarray]] = None
    ) -> GeotaggedInspection:
        """
        Captures frame from stream (or uses provided sample image for simulation),
        performs AI dust classification, saves snapshot, and records geotagged telemetry.
        """
        self._counter += 1
        current_time = time.time()
        frame: Optional[np.ndarray] = None

        if sample_image_override is not None:
            if isinstance(sample_image_override, str):
                with open(sample_image_override, "rb") as f:
                    file_bytes = np.frombuffer(f.read(), dtype=np.uint8)
                    frame = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
            elif isinstance(sample_image_override, Image.Image):
                frame = cv2.cvtColor(np.array(sample_image_override), cv2.COLOR_RGB2BGR)
            elif isinstance(sample_image_override, np.ndarray):
                frame = sample_image_override
        elif self.cap and self.cap.isOpened():
            ret, captured = self.cap.read()
            if ret:
                frame = captured

        # Fallback synthetic frame if no camera feed available
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, f"Simulated Frame #{self._counter}", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        # Run AI detection
        result = self.detector.predict(frame)

        # Save snapshot using unicode-safe cv2.imencode
        status_str = "DUSTY" if result.is_dusty else "CLEAN"
        snapshot_filename = f"insp_{self._counter:04d}_{status_str}_{int(result.confidence * 100)}.jpg"
        snapshot_path = os.path.join(self.output_dir, snapshot_filename)

        # Annotate snapshot with telemetry & classification
        annotated = frame.copy()
        tag_color = (0, 0, 230) if result.is_dusty else (0, 200, 0)
        overlay_text = f"Status: {result.label} ({result.confidence*100:.1f}%)"
        telemetry_text = f"GPS: {telemetry.lat:.6f}, {telemetry.lon:.6f} | Alt: {telemetry.alt_relative:.1f}m"

        cv2.rectangle(annotated, (10, 10), (630, 70), (20, 20, 20), -1)
        cv2.putText(annotated, overlay_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, tag_color, 2)
        cv2.putText(annotated, telemetry_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

        success, encoded = cv2.imencode(".jpg", annotated)
        if success:
            with open(snapshot_path, "wb") as f:
                f.write(encoded.tobytes())

        return GeotaggedInspection(
            inspection_id=self._counter,
            timestamp=current_time,
            lat=telemetry.lat,
            lon=telemetry.lon,
            alt=telemetry.alt_relative,
            heading=telemetry.heading,
            detection=result,
            snapshot_path=snapshot_path
        )
