import json
import cv2
import time

def capture_image():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open camera.")
        return None

    print("Press SPACE to capture the image (ESC to quit).")
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        cv2.imshow("Camera Preview - Press SPACE to capture", frame)
        key = cv2.waitKey(1)

        # Allow capture only after 5 seconds have passed
        elapsed = (time.time() - start_time) * 1000  # milliseconds
        if elapsed >= 5000 or key == 32:  # SPACE
            print("Image captured.")
            cv2.imwrite("captured_frame.jpg", frame)
            break
        elif key == 27:  # ESC
            frame = None
            break

    cap.release()
    cv2.destroyAllWindows()
    return frame



class CameraProcessor:
    def __init__(self, detector, finder):
        self.detector = detector
        self.finder = finder
        self.draw_color = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        for color in self.detector.color_ranges.keys():
            mask = self.detector.create_mask(hsv, color)
            objs = self.finder.find_objects(mask)

            for o in objs:
                o['color'] = color
                detections.append(o)
                self.draw_object(frame, o, color)

        return frame, detections

    def draw_object(self, frame, obj, color):
        x, y, w, h = obj['bbox']
        cx, cy = obj['centroid']

        h_img, w_img = frame.shape[:2]
        board_width_cm = 20 #the real board width in cm
        board_height_cm = 10

        cm_per_pixel_x = board_width_cm / w_img
        cm_per_pixel_y = board_height_cm / h_img

        real_x = cx * cm_per_pixel_x
        real_y = cy * cm_per_pixel_y

        arm_x = 10
        arm_y = 9
        obj_arm_x = real_x - arm_x
        obj_arm_y = real_y - arm_y

        cv2.rectangle(frame, (x, y), (x + w, y + h), self.draw_color[color], 2)

        cv2.circle(frame, (cx, cy), 4, self.draw_color[color], -1)

        text = f"arm=({obj_arm_x:.1f},{obj_arm_y:.1f})cm"

        cv2.putText(frame, text, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.draw_color[color], 1)


    def run(self):
        frame = capture_image()
        if frame is None:
            print("No frame captured. Exiting.")
            return

        print("Processing captured image...")
        processed_frame, detections = self.process_frame(frame)


        h, w = frame.shape[:2]

        board_width_cm = 20     #use real board dimensions :) 
        board_height_cm = 9.99    

        cm_per_pixel_x = board_width_cm / w
        cm_per_pixel_y = board_height_cm / h

        detections_list = []
        for d in detections:
            cx, cy = d['centroid']
            real_x = cx * cm_per_pixel_x
            real_y = cy * cm_per_pixel_y

            arm_x = 10
            arm_y = 9

            obj_arm_x = real_x - arm_x
            obj_arm_y = real_y - arm_y

            print(
                f"{d['type']} {d['color']} @ pixel={d['centroid']} "
                f"real_cam=({real_x:.2f}cm , {real_y:.2f}cm) "
                f"real_arm=({obj_arm_x:.2f}cm , {obj_arm_y:.2f}cm) "
            )
            detections_list.append({
                'type': d['type'],
                'color': d['color'],
                'pixel': d['centroid'],
                'real_cam_cm': (real_x,real_y),
                'real_arm_cm': (obj_arm_x,obj_arm_y)
            })
        type_order = {"Cube": 0 , "Region": 1}
        # json_output = json.dumps(detections_list, indent=4)
        detections_list.sort(key=lambda d:(d['color'] , type_order[d['type']]))
        for idx, det in enumerate(detections_list): print(f"Index {idx}: type={det['type']}, color={det['color']}, real_arm_cm={det['real_arm_cm']}") 
        cv2.imshow("Detected Objects", processed_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return detections_list
