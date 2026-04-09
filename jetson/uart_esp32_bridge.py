#!/usr/bin/env python3
"""ESP32 UART bridge for Jetson Nano.

This module provides a resilient serial bridge to send commands to ESP32 and
read STATUS frames from the robot controller.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any, Dict, Optional

import serial  # type: ignore[import-not-found]


LOGGER = logging.getLogger("esp32_bridge")


class ESP32Bridge:
    """Simple JSON-over-UART bridge to ESP32."""

    def __init__(
        self,
        port: str = "/dev/ttyTHS1",
        baudrate: int = 115200,
        timeout: float = 0.1,
        write_timeout: float = 0.2,
        reconnect_delay: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.reconnect_delay = reconnect_delay

        self._ser: Optional[serial.Serial] = None
        self._last_status_time = 0.0
        self._line_ending = "\n"

        # Payload templates for consistent schema.
        self._command_template: Dict[str, Any] = {
            "type": "COMMAND",
            "cmd": "",
            "timestamp": 0,
        }
        self._manual_template: Dict[str, Any] = {
            "type": "COMMAND",
            "cmd": "MANUAL",
            "throttle": 0,
            "steering": 90,
            "duration_ms": 100,
            "timestamp": 0,
        }

        self.connect()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self) -> bool:
        """Open serial port; return True if connected."""
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
            LOGGER.info("Connected to ESP32 at %s (%d baud)", self.port, self.baudrate)
            return True
        except serial.SerialException as exc:
            self._ser = None
            LOGGER.error("Cannot open serial port %s: %s", self.port, exc)
            return False

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
        self._ser = None

    def ensure_connection(self) -> bool:
        """Ensure serial link is available, with one reconnect attempt."""
        if self.is_open:
            return True

        LOGGER.warning("Serial disconnected, retrying in %.1fs", self.reconnect_delay)
        time.sleep(self.reconnect_delay)
        return self.connect()

    def send_raw(self, data: str) -> bool:
        """Write raw text as one line to UART."""
        if not self.ensure_connection():
            return False

        assert self._ser is not None
        payload = data if data.endswith(self._line_ending) else data + self._line_ending

        try:
            self._ser.write(payload.encode("utf-8"))
            self._ser.flush()
            LOGGER.debug("TX: %s", payload.strip())
            return True
        except serial.SerialException as exc:
            LOGGER.error("UART write failed: %s", exc)
            self.close()
            return False

    def send_command(self, cmd_type: str, **kwargs: Any) -> bool:
        """Build and send standard COMMAND frame."""
        payload = dict(self._command_template)
        payload["cmd"] = str(cmd_type).upper()
        payload["timestamp"] = int(time.time())
        payload.update(kwargs)

        try:
            encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        except (TypeError, ValueError) as exc:
            LOGGER.error("Cannot encode command payload: %s", exc)
            return False

        return self.send_raw(encoded)

    def send_stop(self, reason: str, confidence: float = 1.0) -> bool:
        return self.send_command("STOP", reason=reason, confidence=float(confidence))

    def send_autonomous(self, mode: str = "obstacle_avoidance") -> bool:
        return self.send_command("AUTONOMOUS", mode=mode)

    def send_manual(self, throttle: int, steering: int, duration_ms: int = 100) -> bool:
        frame = dict(self._manual_template)
        frame["throttle"] = max(0, min(255, int(throttle)))
        frame["steering"] = max(55, min(125, int(steering)))
        frame["duration_ms"] = max(10, int(duration_ms))
        frame["timestamp"] = int(time.time())

        try:
            encoded = json.dumps(frame, separators=(",", ":"), ensure_ascii=True)
        except (TypeError, ValueError) as exc:
            LOGGER.error("Cannot encode MANUAL payload: %s", exc)
            return False

        return self.send_raw(encoded)

    def read_status(self) -> Optional[Dict[str, Any]]:
        """Read one status line in non-blocking mode.

        Returns parsed JSON dict or None when no complete frame is available.
        """
        if not self.ensure_connection():
            return None

        assert self._ser is not None
        try:
            raw = self._ser.readline()
        except serial.SerialException as exc:
            LOGGER.error("UART read failed: %s", exc)
            self.close()
            return None

        if not raw:
            return None

        text = raw.decode("utf-8", errors="ignore").strip()
        if not text:
            return None

        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            LOGGER.warning("RX non-JSON line: %s", text)
            return None

        if isinstance(data, dict) and data.get("type") == "STATUS":
            self._last_status_time = time.time()

        return data if isinstance(data, dict) else None

    def monitor_connection(self, timeout_sec: float = 3.0) -> bool:
        """Return True if STATUS has been seen within timeout window."""
        if not self.is_open:
            return False
        if self._last_status_time == 0.0:
            return False
        return (time.time() - self._last_status_time) <= float(timeout_sec)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    bridge = ESP32Bridge()
    if bridge.send_stop("person_detected", confidence=0.95):
        print("Stop command sent")
        time.sleep(2)
        bridge.send_autonomous()

    # Try reading one status frame.
    status = bridge.read_status()
    if status:
        print(json.dumps(status, ensure_ascii=False, indent=2))