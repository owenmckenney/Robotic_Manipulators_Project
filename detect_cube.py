# detects any cubes of a certain color in the frame
# gets the centroid of each cube as well as its quaternion

import cv2
import numpy as np

def detect_cubes(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

            # calculate centriod
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            angle = rect[2]
            if rect[1][0] < rect[1][1]:
                angle += 90

            angle_rad = np.deg2rad(angle)

            # quaternion representation (rotation only about the Z-axis)
            qw = np.cos(angle_rad / 2)
            qx = 0
            qy = 0
            qz = np.sin(angle_rad / 2)

            cube_poses.append({
                "centroid": (cx, cy),
                "quaternion": (qw, qx, qy, qz)
            })

    # print all cube poses
    for i, pose in enumerate(cube_poses):
        centroid = pose["centroid"]
        quaternion = pose["quaternion"]
        print(f"Cube {i + 1}: Centroid={centroid}, Quaternion={quaternion}")

    return cube_poses


cap = cv2.VideoCapture(0) 

if __name__ == "__main__":
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        poses = detect_cubes(frame)

        cv2.imshow("", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
