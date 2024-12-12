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

    # depth = 1250
    # coord = (320, 240)
    # coord = (2000, 2400)
    # ur5_y, ur5_x = pixel_to_ur5_base(coord, depth, R_cam2gripper, t_cam2gripper)
    # ur5_x = -ur5_x
    # ur5_y = -ur5_y
    
    # print(ur5_x, ur5_y)

'''
increase ur5 x ---> decrease camera x
increase ur5 y ---> decrease camera y
'''

class PID:
    def __init__(self, dt=0.1, kp=1.0, ki=0.1, kd=0.0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.previous_error = 0

    def control(self, error):
        self.proportional = self.kp * error
        self.integral += error * self.dt
        self.derivative = (error - self.previous_error) / self.dt
        self.previous_error = error

        output = self.proportional + self.ki * self.integral + self.kd * self.derivative
        
        return output

def error_magnitude(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def error(p1, p2):
    x_error = p1[0] - p2[0]
    y_error = p1[1] - p2[1]
    return x_error, y_error

if __name__ == "__main__":
    camera = Camera(0)
    ip = "10.168.18.56"
    x_min, x_max = 0.2, 0.8
    y_min, y_max = -0.3, 0.3
    z_min, z_max = -0.01, 0.2
    ur5 = UR5SafeControl(ip, x_min, x_max, y_min, y_max, z_min, z_max)
    
    dt = 0.1
    kp = 0.5
    ki = 0.0 
    kd = 0.0
    x_pid = PID(dt, kp=kp, ki=ki, kd=kd)
    y_pid = PID(dt, kp=kp, ki=ki, kd=kd)
    
    goal_reached = False
    # cube_position = (100, 100)
    
    initial_ur5_position = [0.2, 0, 0.05, 0.0, 3.1415, 0]
    initial_ur5_x = initial_ur5_position[0]
    initial_ur5_y = initial_ur5_position[1]
    
    ur5.moveL(initial_ur5_position)
    
    while not goal_reached:
        camera.get_frame()
        
        cube_position = camera.detect_cubes(output=False)[0]["centroid"]
        finger_count, hand_position = camera.detect_hand(output=False)
        
        st = time.time()
        
        # hand_position = (300, 300)
        # if hand_position is not None:
        x_e, y_e = error(cube_position, hand_position)
        error_mag = error_magnitude(cube_position, hand_position)
        
        
        x_control = x_pid.control(-x_e) / 1000
        y_control = y_pid.control(-y_e) / 1000
        
        # cube_position = (cube_position[0] + x_control, cube_position[1] + y_control)
        
        # ur5_x = initial_ur5_x + x_control
        # ur5_y = iniial_ur5_y + y_control
        
        if error_mag < 20:
            goal_reached = True
            ur5.close()
        try:
            # pose = [ur5_x, ur5_y, 0.05, 0.0, -3.1415, 0]
            
            # ur5.moveL(pose, acceleration=1.0, velocity=0.1)
            xd = [x_control, y_control, 0, 0, 0, 0]
            ur5.speedL(xd, time=0.1, acceleration=0.1)
            # ur5.servoL(pose, 0.1, 0.1)
        except KeyboardInterrupt:
            print("Exit.")

            
        et = time.time()
        rt = et - st
        
        if abs(rt) < dt:
            time.sleep(dt - rt)
        
        et = time.time()
        rt = et - st
        
        ur5_x = 0
        ur5_y = 0
        print(f"rt: {rt:.8f}, e: ({x_e:.2f}, {y_e:.2f}), e mag: {error_mag:.2f}, control: ({x_control:.2f}, {y_control:.2f}), ur5 pos: ({ur5_x:.2f}, {ur5_y:.2f}), cube pos: ({cube_position[0]}, {cube_position[1]}), hand pos: ({hand_position[0]}, {hand_position[1]})")

        
        # time.sleep(1)
        
        
        