import cv2
import numpy as np


class BirdEyeView:
    def __init__(self, config):
        bev_cfg = config["bev"]
        self.width = int(bev_cfg["width"])
        self.height = int(bev_cfg["height"])
        src = np.array(bev_cfg["src_points"], dtype=np.float32)
        dst = np.array(bev_cfg["dst_points"], dtype=np.float32)
        self.matrix = cv2.getPerspectiveTransform(src, dst)
        self.inverse_matrix = cv2.getPerspectiveTransform(dst, src)
        self.meters_per_pixel = float(bev_cfg["meters_per_pixel"])

    def warp(self, frame):
        return cv2.warpPerspective(frame, self.matrix, (self.width, self.height))

    def pixel_to_vehicle(self, point):
        x_px, y_px = point
        x = (x_px - self.width / 2.0) * self.meters_per_pixel
        y = (self.height - y_px) * self.meters_per_pixel
        return x, y

    def draw_grid(self, image, step_px=50):
        out = image.copy()
        for x in range(0, self.width, step_px):
            cv2.line(out, (x, 0), (x, self.height), (45, 45, 45), 1)
        for y in range(0, self.height, step_px):
            cv2.line(out, (0, y), (self.width, y), (45, 45, 45), 1)
        cv2.line(out, (self.width // 2, 0), (self.width // 2, self.height), (0, 180, 255), 2)
        return out

