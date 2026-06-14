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

