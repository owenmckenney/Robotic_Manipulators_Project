import cv2
import numpy as np
import time

# exponential moving average filter
def ema_filter(alpha, p, prev_p):
    return alpha * p + (1 - alpha) * prev_p

# load ArUco library, set dictionary type (using 7x7)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
parameters = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(dictionary, parameters)

marker_size = 40 # in mm

# camera constants gathered from single_camera_calibration.py
# camera_matrix = np.array([
#     [1.9477e3, 0, 9.5480e2],
#     [0, 1.9509e3, 5.1435e2],
#     [0, 0, 1]
# ])

# camera_matrix = camera_matrix / 1000

# distortion_coefficients = np.array([[0.1094, -0.0054, -0.0063, 0.0035, -1.0672]])



camera_matrix = np.array([
    [482.3427, 0, 319.2339],
    [0, 483.1669, 251.9371],
    [0, 0, 1]
])

distortion_coefficients = np.array([[0.0714, -0.2915, 0, 0, 0]])


marker_x, marker_y, marker_z = 0, 0, 0
prev_marker_x, prev_marker_y, prev_marker_z = 0, 0, 0

cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # detect marker
    corners, ids, rejected = aruco_detector.detectMarkers(gray)
     
    # runs only if aruco is detected
    if ids is not None:

        # estimate pose of the marker. rvecs is the rotation vector, tvecs is the translation vector
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coefficients)

        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            
            cv2.drawFrameAxes(image, camera_matrix, distortion_coefficients, rvec, tvec, marker_size / 2)
            
            rvec = np.array(rvec).reshape((3, 1))
            tvec = np.array(tvec).reshape((3, 1))

            # Extract pixel location of marker center
            marker_center_pixel = corners[i][0].mean(axis=0)  # Calculate the mean of the corners
            pixel_x, pixel_y = marker_center_pixel
            print(f"Pixel Location: ({pixel_x:.2f}, {pixel_y:.2f})")

            # get camera position from dot of rvec and tvec
            # rotation_matrix, _ = cv2.Rodrigues(rvec)
            # camera_pos = -np.dot(rotation_matrix.T, tvec)

            # extract x, y, z, calculate distance
            # raw_x, raw_y, raw_z = camera_pos.flatten()
            raw_x, raw_y, raw_z = tvec.flatten()

            # filter camera coordinates to reduce variation
            marker_x = ema_filter(0.3, raw_x, prev_marker_x)
            marker_y = ema_filter(0.3, raw_y, prev_marker_y)
            marker_z = ema_filter(0.3, raw_z, prev_marker_z)
      
            prev_marker_x = marker_x
            prev_marker_y = marker_y
            prev_marker_z = marker_z

            distance = np.sqrt(marker_x**2 + marker_y**2 + marker_z**2)

            print(f"Camera Position: ({marker_x:.2f}, {marker_y:.2f}, {marker_z:.2f}) mm, Distance: {distance:.2f} mm")

    cv2.imshow("feed", image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# center: (10, -40, 792), (325, 253)
# left: (-348 -9, 788), (105, 246)
# top: (5, -358, 780), (322, 27)
# right: (355, -18, 795), (533, 240)
# bottom: (20, 343, 818), (331, 454)

# compute homography matrix
pixel_points = np.array([(325, 253), (105, 246), (322, 27), (533, 240), (331, 454)])
robot_points = np.array([(10, -40), (-348, -9), (5, -358), (355, -18), (20, 343)])

homography_matrix, _ = cv2.findHomography(pixel_points, robot_points)
print(homography_matrix)

cap.release()
cv2.destroyAllWindows()