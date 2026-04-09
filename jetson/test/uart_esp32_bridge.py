import argparse
import json
import sys
import time
from pathlib import Path

import serial  # type: ignore[import-not-found]

CONFIG_PATH = Path(__file__).with_name("uart_esp32_config.json")


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


class ESP32Bridge:
    def __init__(self, config: dict):
        self.config = config
        self.line_ending = config.get("line_ending", "\n")
        self.ser = serial.Serial(
            port=config.get("port", "/dev/ttyTHS1"),
            baudrate=int(config.get("baudrate", 115200)),
            timeout=float(config.get("timeout_sec", 0.1)),
            write_timeout=float(config.get("write_timeout_sec", 0.2)),
        )

    def send_command(self, cmd: str, **kwargs):
        payload = {
            "cmd": cmd,
            "timestamp": int(time.time()),
        }
        payload.update(kwargs)

        data = json.dumps(payload, separators=(",", ":")) + self.line_ending
        self.ser.write(data.encode("utf-8"))
        self.ser.flush()
        print(f"[TX] {data.strip()}")

    def read_status_once(self):
        line = self.ser.readline()
        if not line:
            return None

        text = line.decode("utf-8", errors="ignore").strip()
        if not text:
            return None

        try:
            obj = json.loads(text)
            print("[RX]", json.dumps(obj, ensure_ascii=False))
            return obj
        except json.JSONDecodeError:
            print(f"[RX-RAW] {text}")
            return {"raw": text}


def main():
    parser = argparse.ArgumentParser(description="Jetson <-> ESP32 UART bridge")
    parser.add_argument("--config", default=str(CONFIG_PATH), help="Path to UART config JSON")
    parser.add_argument("--cmd", choices=["STOP", "AUTONOMOUS", "MANUAL"], help="Command to send")
    parser.add_argument("--throttle", type=int, default=0, help="Throttle for MANUAL (0-255)")
    parser.add_argument("--steering", type=int, default=90, help="Steering for MANUAL (55-125)")
    parser.add_argument("--listen", action="store_true", help="Listen status from ESP32")
    parser.add_argument("--listen-seconds", type=float, default=3.0, help="How long to listen")

    args = parser.parse_args()

    config = load_config(Path(args.config))

    try:
        bridge = ESP32Bridge(config)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open serial port: {e}")
        sys.exit(1)

    try:
        if args.cmd == "STOP":
            bridge.send_command("STOP")
        elif args.cmd == "AUTONOMOUS":
            bridge.send_command("AUTONOMOUS")
        elif args.cmd == "MANUAL":
            throttle = max(0, min(255, args.throttle))
            steering = max(55, min(125, args.steering))
            bridge.send_command("MANUAL", throttle=throttle, steering=steering)

        if args.listen:
            t0 = time.time()
            while (time.time() - t0) < args.listen_seconds:
                bridge.read_status_once()
                time.sleep(0.02)
    finally:
        bridge.ser.close()


if __name__ == "__main__":
    main()
