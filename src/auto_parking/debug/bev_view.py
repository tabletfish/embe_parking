import time


def parse_source(value):
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


class BevDebugView:
    def __init__(self, config, source=None, window_name="front | BEV slots | tape mask"):
        import cv2

        from auto_parking.camera.csi import open_video_source
        from auto_parking.perception.bev import BirdEyeView

        self.cv2 = cv2
        self.cap = open_video_source(parse_source(source), config)
        self.bev = BirdEyeView(config)
        self.config = config
        self.window_name = window_name
        self.locked_slot = None
        self.locked_at = 0.0
        self.lock_timeout_s = 3.0
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    def update(self, status_text=""):
        import numpy as np

        from auto_parking.perception.slot_detector import (
            detect_slot_candidates,
            detect_vertical_tape_boundaries,
            draw_locked_parking_slot,
            draw_parking_slots,
            draw_slots,
            draw_tape_boundaries,
            infer_parking_slots_from_mask,
        )
        from auto_parking.perception.tape import tape_mask

        ok, frame = self.cap.read()
        if not ok:
            return True

        top = self.bev.warp(frame)
        mask = tape_mask(top, self.config)
        slots = detect_slot_candidates(mask, self.config)
        boundaries = detect_vertical_tape_boundaries(mask, self.config)
        parking_slots = infer_parking_slots_from_mask(mask, self.config)
        now = time.monotonic()
        if parking_slots:
            self.locked_slot = parking_slots[0]
            self.locked_at = now
        elif self.locked_slot is not None and now - self.locked_at > self.lock_timeout_s:
            self.locked_slot = None

        debug = self.bev.draw_grid(top)
        debug = draw_slots(debug, slots)
        debug = draw_tape_boundaries(debug, boundaries)
        debug = draw_parking_slots(debug, parking_slots)
        if not parking_slots and self.locked_slot is not None:
            debug = draw_locked_parking_slot(debug, self.locked_slot.center_px, self.locked_slot.entry_px)

        mask_bgr = self.cv2.cvtColor(mask, self.cv2.COLOR_GRAY2BGR)
        if frame.shape[:2] != top.shape[:2]:
            frame_show = self.cv2.resize(frame, (top.shape[1], top.shape[0]))
        else:
            frame_show = frame

        combined = np.hstack([frame_show, debug, mask_bgr])
        if status_text:
            self.cv2.putText(
                combined,
                status_text,
                (16, 28),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                self.cv2.LINE_AA,
            )
        self.cv2.imshow(self.window_name, combined)
        key = self.cv2.waitKey(1) & 0xFF
        return key not in (ord("q"), 27)

    def close(self):
        self.cap.release()
        self.cv2.destroyWindow(self.window_name)
