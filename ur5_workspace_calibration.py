import cv2
import numpy as np

# Exponential moving average filter
def ema_filter(alpha, p, prev_p):
    return alpha * p + (1 - alpha) * prev_p

def rotation_matrix_from_euler(roll, pitch, yaw):
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return R_z @ R_y @ R_x

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
parameters = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(dictionary, parameters)

marker_size = 50  # in mm

camera_matrix = np.array([
    [631.6449, 0, 313.3236],
    [0, 631.4947, 272.6468],
    [0, 0, 1]
])
distortion_coefficients = np.array([[0.0268, 0.0964, 0.00450, -0.0081, -0.7435]])

# Initialize video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Store calibration data
R_gripper2base = []  # Rotation from robot gripper to base
t_gripper2base = []  # Translation from robot gripper to base
R_target2cam = []    # Rotation from target to camera
t_target2cam = []    # Translation from target to camera

prev_marker_x, prev_marker_y, prev_marker_z = 0, 0, 0

print("Press 'c' to capture calibration data at the current marker position.")

while True:
    ret, image = cap.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco marker
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, distortion_coefficients
        )

        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            cv2.drawFrameAxes(image, camera_matrix, distortion_coefficients, rvec, tvec, marker_size / 2)

            raw_x, raw_y, raw_z = tvec.flatten()
            marker_x = ema_filter(0.3, raw_x, prev_marker_x)
            marker_y = ema_filter(0.3, raw_y, prev_marker_y)
            marker_z = ema_filter(0.3, raw_z, prev_marker_z)
            prev_marker_x, prev_marker_y, prev_marker_z = marker_x, marker_y, marker_z

            R_target = cv2.Rodrigues(rvec)[0]
            t_target = tvec.flatten()

            print(f"camera: R={R_target}, t={t_target}")

            if cv2.waitKey(1) & 0xFF == ord('c'):
                print("calibration point. ")
                R_target2cam.append(R_target)
                t_target2cam.append(t_target)

                ur5_rotation = input("rotation (rx, ry, rz): ")

                r, p, y = map(float, ur5_rotation.split())
                ur5_rotation = rotation_matrix_from_euler(r, p, y)
                
                ur5_translation = input("translation (x, y, z): ")
                R_gripper2base.append(np.array(ur5_rotation, dtype=float))
                t_gripper2base.append(np.array(ur5_translation.split(), dtype=float))

    cv2.imshow("feed", image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Convert lists to numpy arrays
R_gripper2base = np.array(R_gripper2base)
t_gripper2base = np.array(t_gripper2base)
R_target2cam = np.array(R_target2cam)
t_target2cam = np.array(t_target2cam)

print(R_gripper2base)
print(t_gripper2base)
print(R_target2cam)
print(t_target2cam)

#hand-eye calibration
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
)

print("rotation (c2g):")
print(R_cam2gripper)
print("translation (c2g):")
print(t_cam2gripper)

cap.release()
cv2.destroyAllWindows()

'''
Rotation (Camera to Gripper):
[[-0.23378378  0.15997411  0.95903776]
 [ 0.82398401  0.5562091   0.10808233]
 [-0.51613516  0.81549967 -0.26184878]]
Translation (Camera to Gripper):
[[-594.42043464]
 [ 299.20586797]
 [ 336.63113552]]
 
 Rotation (Camera to Gripper):
[[ 0.58820912  0.65469843 -0.47474203]
 [-0.68564569  0.71501684  0.13653173]
 [ 0.42883566  0.24519562  0.86947058]]
Translation (Camera to Gripper):
[[543.18024589]
 [ -6.03785461]
 [ 30.19612007]]
 
 
 
 rotation (c2g):
[[ 0.5505824  -0.51232231 -0.6590788 ]
 [-0.24973815  0.65227444 -0.71565977]
 [ 0.79654872  0.5586268   0.2311844 ]]
translation (c2g):
[[-1196.28205365]
 [   88.57750382]
 [ -406.63127192]]
'''