import cv2
import numpy as np

class ColorDetector:
    def __init__(self):
        self.color_ranges = {
            'red': [((0, 80, 40), (10, 255, 255)), ((160, 80, 40), (179, 255, 255))],
            'green': [((40, 50, 40), (85, 255, 255))],
            'blue': [((90, 50, 40), (140, 255, 255))]
        }

    def create_mask(self, hsv, color_name):
        mask = None
        for lower, upper in self.color_ranges[color_name]:
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask