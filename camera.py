# class to handle camera operations
# detect_cubes detects all cubes in frame and returns poses
# detect_hand detects hand and returns number of fingers held up

import cv2
import numpy as np
import mediapipe as mp

class Camera:
    def __init__(self, usb_port):
        self.cap = cv2.VideoCapture(usb_port)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.75, min_tracking_confidence=0.75)
        self.mp_draw = mp.solutions.drawing_utils
        
        self.last_hand_pos = [0,0]

    def quit(self):
        self.cap.release()
        cv2.destroyAllWindows()
    
    # called before detection on every loop run
    def get_frame(self):
        ret, self.frame = self.cap.read()

    # should be called on every loop run
    def detect_cubes(self, output=False):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
    
        # range for color in hsv (currently green)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # create a mask for the color
        mask = cv2.inRange(hsv, lower_green, upper_green)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cube_poses = []  
        
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.drawContours(self.frame, [box], 0, (0, 255, 0), 2)

                # calculate centriod
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0

                cv2.circle(self.frame, (cx, cy), 5, (0, 0, 255), -1)

                angle = rect[2]
                if rect[1][0] < rect[1][1]:
                    angle += 90

                angle_rad = np.deg2rad(angle)

                cube_poses.append({
                    "centroid": (640 - cx, cy),
                    "angle": angle_rad
                })

        # print all cube poses
        if output:
            for i, pose in enumerate(cube_poses):
                centroid = pose["centroid"]
                angle = pose["angle"]
                print(f"Cube {i + 1}: Centroid={centroid}, Angle={angle}")
        return cube_poses
    
    # should be called on every loop run
    # gathered from https://github.com/Sousannah/hand-tracking-using-mediapipe
    def detect_hand(self, output=False):
        tip_ids = [4, 8, 12, 16, 20]
        palm_ids = [0, 1, 5, 9, 13, 17]  # Landmark points around the palm

        self.frame = cv2.flip(self.frame, 1)
        frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                lm_list = []
                h, w, c = self.frame.shape
                for id, lm in enumerate(hand_lms.landmark):
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lm_list.append((id, cx, cy))

                if lm_list:
                    fingers = []
                    for id in range(1, 5):
                        if lm_list[tip_ids[id]][2] < lm_list[tip_ids[id] - 2][2]:
                            fingers.append(1)
                        else:
                            fingers.append(0)

                    total_fingers = fingers.count(1)

                    # Calculate center of palm
                    palm_coords = [(lm_list[id][1], lm_list[id][2]) for id in palm_ids]
                    palm_center_x = int(sum(x for x, y in palm_coords) / len(palm_coords))
                    palm_center_y = int(sum(y for x, y in palm_coords) / len(palm_coords))
                    palm_center = (palm_center_x, palm_center_y)

                    if output:
                        print(f"Fingers up: {total_fingers}, Palm center: {palm_center}")

                    self.mp_draw.draw_landmarks(self.frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                    self.last_hand_pos = palm_center

                    return total_fingers, palm_center

        return 0, self.last_hand_pos

        
    def show_frame(self):
        cv2.imshow("Frame", self.frame)

    def detect_close(self):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return True


if __name__ == "__main__":

    camera = Camera(0)

    while True:
        camera.get_frame()
        # poses = camera.detect_cubes(output=False)
        poses = camera.detect_cubes()
        fingers = camera.detect_hand(output=True)
        
        print(poses)

        cv2.imshow("cube detect", camera.frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera.quit()
