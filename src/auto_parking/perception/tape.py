import cv2
import numpy as np


def tape_mask(bgr, config):
    lower = np.array(config["vision"]["tape_hsv_lower"], dtype=np.uint8)
    upper = np.array(config["vision"]["tape_hsv_upper"], dtype=np.uint8)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def hough_segments(mask, config):
    vision = config["vision"]
    return cv2.HoughLinesP(
        mask,
        rho=1,
        theta=np.pi / 180.0,
        threshold=int(vision["hough_threshold"]),
        minLineLength=int(vision["min_line_length"]),
        maxLineGap=int(vision["max_line_gap"]),
    )

