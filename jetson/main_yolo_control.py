#!/usr/bin/env python3
"""YOLO-based unattended object tracking with ESP32 command bridge.

State machine:
- PATROL: robot runs autonomously
- TRACKING: robot stops and monitors one object
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import cv2  # type: ignore[import-not-found]
from ultralytics import YOLO  # type: ignore[import-not-found]

from uart_esp32_bridge import ESP32Bridge


# Available classes (only)
OBJECT_CLASSES = ["backpack", "handbag", "suitcase", "laptop"]
PERSON_CLASS = "person"

# Thresholds / constants
OBJECT_CONF = 0.55
PERSON_CONF = 0.50
T_LIFE_MAX = 300.0
T_ALONE_ALERT = 180.0
T_HUMAN_INTERACT_SAFE = 10.0
T_HUMAN_NEAR_SAFE = 30.0
ZONE_SCALE = 1.6

STOP_KEEPALIVE_INTERVAL = 1.5
PERSON_NEAR_DISTANCE_RATIO = 0.5


LOGGER = logging.getLogger("yolo_control")


class Mode(str, Enum):
    PATROL = "PATROL"
    TRACKING = "TRACKING"


@dataclass
class Detection:
    cls_name: str
    conf: float
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2

    @property
    def center(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)

    @property
    def size(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return (max(1.0, x2 - x1), max(1.0, y2 - y1))


@dataclass
class TrackingState:
    obj: Detection
    t_life: float = 0.0
    t_alone: float = 0.0
    t_human: float = 0.0
    alert_sent: bool = False


def iou(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0

    area_a = max(1.0, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1.0, (bx2 - bx1) * (by2 - by1))
    return inter / (area_a + area_b - inter)


def center_distance(a: Detection, b: Detection) -> float:
    ax, ay = a.center
    bx, by = b.center
    return math.hypot(ax - bx, ay - by)


def expand_bbox(bbox: Tuple[float, float, float, float], scale: float, frame_shape: Sequence[int]) -> Tuple[float, float, float, float]:
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    bw = max(1.0, x2 - x1) * scale
    bh = max(1.0, y2 - y1) * scale

    nx1 = max(0.0, cx - bw * 0.5)
    ny1 = max(0.0, cy - bh * 0.5)
    nx2 = min(float(w - 1), cx + bw * 0.5)
    ny2 = min(float(h - 1), cy + bh * 0.5)
    return (nx1, ny1, nx2, ny2)


def parse_detections(result) -> List[Detection]:
    detections: List[Detection] = []
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return detections

    names = result.names if hasattr(result, "names") else {}
    for box in boxes:
        cls_idx = int(box.cls[0].item())
        conf = float(box.conf[0].item())
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
        cls_name = names.get(cls_idx, str(cls_idx)) if isinstance(names, dict) else str(cls_idx)
        detections.append(Detection(cls_name=cls_name, conf=conf, bbox=(x1, y1, x2, y2)))

    return detections


def select_candidate_object(detections: List[Detection]) -> Optional[Detection]:
    candidates = [d for d in detections if d.cls_name in OBJECT_CLASSES and d.conf >= OBJECT_CONF]
    if not candidates:
        return None
    return max(candidates, key=lambda d: d.conf)


def select_same_object(previous: Detection, detections: List[Detection]) -> Optional[Detection]:
    same_cls = [d for d in detections if d.cls_name == previous.cls_name and d.conf >= OBJECT_CONF]
    if not same_cls:
        return None
    return min(same_cls, key=lambda d: center_distance(previous, d))


def people_in_zone(detections: List[Detection], zone_bbox: Tuple[float, float, float, float]) -> List[Detection]:
    persons = [d for d in detections if d.cls_name == PERSON_CLASS and d.conf >= PERSON_CONF]
    in_zone: List[Detection] = []
    for p in persons:
        px, py = p.center
        zx1, zy1, zx2, zy2 = zone_bbox
        if zx1 <= px <= zx2 and zy1 <= py <= zy2:
            in_zone.append(p)
    return in_zone


def is_interacting(person: Detection, obj: Detection) -> bool:
    overlap = iou(person.bbox, obj.bbox)
    if overlap > 0.1:
        return True

    ow, oh = obj.size
    near_threshold = PERSON_NEAR_DISTANCE_RATIO * math.sqrt(ow * ow + oh * oh)
    return center_distance(person, obj) < near_threshold


def notify_security_alert(reason: str, tracked_obj: Detection, bridge: ESP32Bridge) -> None:
    LOGGER.error(
        "SECURITY ALERT | reason=%s object=%s conf=%.2f bbox=%s",
        reason,
        tracked_obj.cls_name,
        tracked_obj.conf,
        tracked_obj.bbox,
    )
    bridge.send_stop(reason=f"alert:{reason}", confidence=float(tracked_obj.conf))


def draw_overlay(frame, mode: Mode, tracking: Optional[TrackingState]) -> None:
    color = (0, 200, 255) if mode == Mode.PATROL else (0, 80, 255)
    cv2.putText(frame, f"MODE: {mode.value}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    if tracking is None:
        return

    x1, y1, x2, y2 = [int(v) for v in tracking.obj.bbox]
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 220, 0), 2)
    cv2.putText(
        frame,
        f"O={tracking.obj.cls_name} life={tracking.t_life:.1f}s alone={tracking.t_alone:.1f}s human={tracking.t_human:.1f}s",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
    )


def run(args: argparse.Namespace) -> int:
    model = YOLO(args.model)
    bridge = ESP32Bridge(port=args.port, baudrate=args.baudrate, timeout=args.timeout)
    cap = cv2.VideoCapture(args.source)

    if not cap.isOpened():
        LOGGER.error("Cannot open camera source: %s", args.source)
        return 1

    mode = Mode.PATROL
    tracking: Optional[TrackingState] = None
    last_time = time.monotonic()
    last_timer_log = 0.0
    last_stop_keepalive = 0.0

    try:
        while True:
            now = time.monotonic()
            dt = max(0.001, now - last_time)
            last_time = now

            ok, frame = cap.read()
            status = bridge.read_status()
            if status is not None:
                LOGGER.debug("ESP32 STATUS: %s", status)

            if not bridge.ensure_connection():
                LOGGER.warning("UART disconnected; holding STOP until reconnect")
                time.sleep(0.2)
                continue

            if not ok:
                LOGGER.warning("Camera frame read failed; continuing with dt=%.3f", dt)
                if mode == Mode.TRACKING and tracking is not None:
                    tracking.t_life += dt
                time.sleep(0.01)
                continue

            detections: List[Detection] = []
            inference_ok = True
            try:
                results = model(frame, verbose=False)
                if results:
                    detections = parse_detections(results[0])
            except Exception as exc:  # pylint: disable=broad-except
                inference_ok = False
                LOGGER.exception("YOLO inference error: %s", exc)

            if mode == Mode.PATROL:
                candidate = select_candidate_object(detections) if inference_ok else None
                if candidate is not None:
                    zone = expand_bbox(candidate.bbox, ZONE_SCALE, frame.shape)
                    persons = people_in_zone(detections, zone)
                    if persons:
                        LOGGER.info(
                            "PATROL SAFE | object=%s ignored due to person in zone",
                            candidate.cls_name,
                        )
                    else:
                        if bridge.send_stop(reason=f"track:{candidate.cls_name}", confidence=candidate.conf):
                            LOGGER.info(
                                "STATE TRANSITION: PATROL -> TRACKING | object=%s conf=%.2f",
                                candidate.cls_name,
                                candidate.conf,
                            )
                            mode = Mode.TRACKING
                            tracking = TrackingState(obj=candidate)
                            last_stop_keepalive = now
                        else:
                            LOGGER.warning("Failed to send STOP while entering TRACKING")

            elif mode == Mode.TRACKING and tracking is not None:
                tracking.t_life += dt

                if (now - last_stop_keepalive) >= STOP_KEEPALIVE_INTERVAL:
                    bridge.send_stop(reason="tracking_keepalive", confidence=tracking.obj.conf)
                    last_stop_keepalive = now

                if tracking.t_life >= T_LIFE_MAX:
                    if not tracking.alert_sent:
                        notify_security_alert("life_timeout", tracking.obj, bridge)
                        tracking.alert_sent = True
                        LOGGER.info("STATE TRANSITION: TRACKING -> ALERT | reason=T_life>=5min")
                else:
                    tracked_obj = select_same_object(tracking.obj, detections) if inference_ok else tracking.obj
                    if inference_ok and tracked_obj is None:
                        LOGGER.info("STATE TRANSITION: TRACKING -> SAFE | reason=object_lost")
                        if bridge.send_autonomous():
                            mode = Mode.PATROL
                            tracking = None
                        else:
                            LOGGER.warning("Failed to send AUTONOMOUS after object_lost")
                    else:
                        if tracked_obj is not None:
                            tracking.obj = tracked_obj

                        zone = expand_bbox(tracking.obj.bbox, ZONE_SCALE, frame.shape)
                        persons = people_in_zone(detections, zone) if inference_ok else []

                        if not persons:
                            tracking.t_human = 0.0
                            if inference_ok:
                                tracking.t_alone += dt

                            if tracking.t_alone >= T_ALONE_ALERT:
                                if not tracking.alert_sent:
                                    notify_security_alert("alone_timeout", tracking.obj, bridge)
                                    tracking.alert_sent = True
                                    LOGGER.info("STATE TRANSITION: TRACKING -> ALERT | reason=T_alone>=180s")
                        else:
                            tracking.t_human += dt
                            # Pause T_alone while person exists in zone.

                            interacting = any(is_interacting(person, tracking.obj) for person in persons)
                            if interacting and tracking.t_human >= T_HUMAN_INTERACT_SAFE:
                                LOGGER.info(
                                    "STATE TRANSITION: TRACKING -> SAFE | reason=interacting>=10s"
                                )
                                if bridge.send_autonomous():
                                    mode = Mode.PATROL
                                    tracking = None
                                else:
                                    LOGGER.warning("Failed to send AUTONOMOUS for interacting safe")
                            elif (not interacting) and tracking.t_human >= T_HUMAN_NEAR_SAFE:
                                LOGGER.info(
                                    "STATE TRANSITION: TRACKING -> SAFE | reason=person_near>=30s"
                                )
                                if bridge.send_autonomous():
                                    mode = Mode.PATROL
                                    tracking = None
                                else:
                                    LOGGER.warning("Failed to send AUTONOMOUS for near safe")

                if tracking is not None and (now - last_timer_log) >= 1.0:
                    LOGGER.info(
                        "TIMERS | T_life=%.1f T_alone=%.1f T_human=%.1f object=%s",
                        tracking.t_life,
                        tracking.t_alone,
                        tracking.t_human,
                        tracking.obj.cls_name,
                    )
                    last_timer_log = now

            draw_overlay(frame, mode, tracking)
            if args.show:
                cv2.imshow("Jetson YOLO Control", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        LOGGER.info("Ctrl+C received, shutting down")
        if tracking is None or (tracking is not None and not tracking.alert_sent):
            bridge.send_autonomous()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        bridge.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="YOLO unattended object monitor -> ESP32 commands")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model path")
    parser.add_argument("--source", default=0, help="Camera source index or URI")
    parser.add_argument("--port", default="/dev/ttyTHS1", help="UART port to ESP32")
    parser.add_argument("--baudrate", type=int, default=115200, help="UART baudrate")
    parser.add_argument("--timeout", type=float, default=0.1, help="UART timeout seconds")
    parser.add_argument("--show", action="store_true", help="Show camera preview window")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser


def normalize_source(raw_source):
    if isinstance(raw_source, int):
        return raw_source
    if isinstance(raw_source, str) and raw_source.isdigit():
        return int(raw_source)
    return raw_source


if __name__ == "__main__":
    cli = build_parser().parse_args()
    logging.basicConfig(
        level=getattr(logging, cli.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    cli.source = normalize_source(cli.source)
    raise SystemExit(run(cli))