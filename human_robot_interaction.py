import threading
import time
from PyQt5.QtCore import QObject, pyqtSignal, QTimer  # Adjust for your Qt version
from UR5SafeControl import UR5SafeControl
from camera import Camera
import numpy as np


# R_cam2gripper = np.array([[ 0.5505824,  -0.51232231, -0.6590788 ],
#  [-0.24973815,  0.65227444, -0.71565977],
#  [ 0.79654872,  0.5586268,   0.2311844 ]])
# t_cam2gripper = np.array([[-1196.28205365],
#  [   88.57750382],
#  [ -406.63127192]])

# camera_matrix = np.array([
#     [631.6449, 0, 313.3236],
#     [0, 631.4947, 272.6468],
#     [0, 0, 1]
# ])


# def pixel_to_ur5_base(pixel_coord, depth, R_cam2base, t_cam2base):
#     # Convert pixel to camera coordinates
#     pixel_homogeneous = np.array([pixel_coord[0], pixel_coord[1], 1], dtype=float)
#     camera_coord = np.linalg.inv(camera_matrix) @ (pixel_homogeneous * depth)

#     # Transform camera coordinates to UR5 base
#     base_coord = R_cam2base @ camera_coord + t_cam2base

#     # Return x, y in the UR5 base
#     return base_coord[0, 0], base_coord[1, 0]


'''
increase ur5 x ---> decrease camera x
increase ur5 y ---> 

'''

if __name__ == "__main__":
    camera = Camera(0)
    ip = "10.168.18.106"
    x_min, x_max = 0.2, 0.8
    y_min, y_max = -0.3, 0.3
    z_min, z_max = -0.01, 0.2
    ur5 = UR5SafeControl(ip, x_min, x_max, y_min, y_max, z_min, z_max)
    
    # depth = 1250
    # coord = (320, 240)
    # coord = (2000, 2400)
    # ur5_y, ur5_x = pixel_to_ur5_base(coord, depth, R_cam2gripper, t_cam2gripper)
    # ur5_x = -ur5_x
    # ur5_y = -ur5_y
    
    # print(ur5_x, ur5_y)
    
    try:
        # pose = [ur5_x / 1000, 0, -0.01, 0.0, -3.1415, 0]
        pose = [0.5, 0.2, -0.01, 0.0, -3.1415, 0]
        ur5.moveL(pose, acceleration=0.1, velocity=0.1)
    except KeyboardInterrupt:
        print("Exit.")
    finally:
        ur5.close()

    # try:
    #     while not camera.detect_close():
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     print("Exiting...")

    # Stop threads
    # stop_event.set()
    # camera_worker.stop()
    # camera_thread.join()
    # ur5_thread.join()

    # camera.quit()
