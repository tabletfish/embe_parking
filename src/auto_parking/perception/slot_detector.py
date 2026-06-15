from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class SlotCandidate:
    center_px: tuple
    size_px: tuple
    yaw_rad: float
    contour: np.ndarray
    occupied: bool = False


@dataclass
class ParkingSlot:
    center_px: tuple
    entry_px: tuple
    left_boundary: SlotCandidate
    right_boundary: SlotCandidate


def detect_slot_candidates(mask, config):
    vision = config["vision"]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    slots = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < vision["min_slot_area_px"] or area > vision["max_slot_area_px"]:
            continue

        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), angle_deg = rect
        if rw <= 1 or rh <= 1:
            continue

        long_side = max(rw, rh)
        short_side = min(rw, rh)
        aspect = long_side / short_side
        if aspect < 1.2 or aspect > 8.0:
            continue

        yaw = np.deg2rad(angle_deg)
        slots.append(
            SlotCandidate(
                center_px=(int(cx), int(cy)),
                size_px=(int(rw), int(rh)),
                yaw_rad=float(yaw),
                contour=cnt,
            )
        )
    slots.sort(key=lambda s: s.center_px[0])
    return slots


def infer_parking_slots(boundaries, config):
    if len(boundaries) < 2:
        return []

    vision = config["vision"]
    min_width = int(vision.get("min_parking_width_px", 120))
    max_width = int(vision.get("max_parking_width_px", 420))
    entry_offset = int(vision.get("entry_offset_px", 120))

    parking_slots = []
    ordered = sorted(boundaries, key=lambda s: s.center_px[0])
    for left, right in zip(ordered, ordered[1:]):
        lx, ly = left.center_px
        rx, ry = right.center_px
        width = rx - lx
        if width < min_width or width > max_width:
            continue

        cx = int((lx + rx) / 2)
        cy = int((ly + ry) / 2)
        entry_y = min(config["bev"]["height"] - 1, cy + entry_offset)
        parking_slots.append(
            ParkingSlot(
                center_px=(cx, cy),
                entry_px=(cx, entry_y),
                left_boundary=left,
                right_boundary=right,
            )
        )
    return parking_slots


def draw_slots(image, slots):
    out = image.copy()
    for idx, slot in enumerate(slots):
        rect = (slot.center_px, slot.size_px, np.rad2deg(slot.yaw_rad))
        box = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(out, [box], 0, (0, 255, 255), 2)
        cv2.circle(out, slot.center_px, 5, (0, 0, 255), -1)
        cv2.putText(
            out,
            f"slot {idx}",
            (slot.center_px[0] + 8, slot.center_px[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return out


def draw_parking_slots(image, parking_slots):
    out = image.copy()
    for idx, slot in enumerate(parking_slots):
        cv2.circle(out, slot.center_px, 7, (255, 0, 255), -1)
        cv2.circle(out, slot.entry_px, 7, (0, 128, 255), -1)
        cv2.line(out, slot.center_px, slot.entry_px, (0, 128, 255), 2)
        cv2.putText(
            out,
            f"park {idx}",
            (slot.center_px[0] + 8, slot.center_px[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            out,
            "entry",
            (slot.entry_px[0] + 8, slot.entry_px[1] + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 128, 255),
            1,
            cv2.LINE_AA,
        )
    return out
