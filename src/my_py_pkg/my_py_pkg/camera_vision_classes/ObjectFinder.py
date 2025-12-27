import cv2

class ObjectFinder:
    def __init__(self, cube_threshold=15000, noise_threshold=500):
        self.cube_threshold = cube_threshold
        self.noise_threshold = noise_threshold

    def find_objects(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.noise_threshold:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            m = cv2.moments(cnt)
            cx = int(m['m10'] / m['m00']) if m['m00'] != 0 else x + w // 2
            cy = int(m['m01'] / m['m00']) if m['m00'] != 0 else y + h // 2

            if area > self.cube_threshold:
                label = 'Region'
            else:
                label = 'Cube'

            objects.append({
                'centroid': (cx, cy),
                'bbox': (x, y, w, h),
                'area': area,
                'type': label
            })

        return objects
