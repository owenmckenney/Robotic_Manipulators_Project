from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import threading
import time
import numpy as np

ip = "172.25.192.42"  

tcp_pose = None
running = True

def receive_data():
    global tcp_pose, running
    rtde_receive = RTDEReceiveInterface(ip)
    while running:
        try:
            tcp_pose = rtde_receive.getActualTCPPose()
            print("Current TCP Pose:", tcp_pose)
            time.sleep(0.1)  
        except Exception as e:
            print("Error receiving data:", e)
            break

def control_robot():
    global running
    rtde_control = RTDEControlInterface(ip)
    try:
        # target pose [x, y, z, rx, ry, rz]
        homeJ = [0, 0, 0, 3 * np.pi / 3, 0, 0]

        target_pose = [0, 0.5, 0, np.pi, 0, 0]
        joint_angles = rtde_control.getInverseKinematics(target_pose)

        # rtde_control.moveL(target_pose, speed=0.5, acceleration=0.1)
        rtde_control.moveJ(joint_angles, speed=0.5, acceleration=0.1)

        print("Robot moved to target pose.")

    except Exception as e:
        print("Error controlling robot:", e)
    finally:
        rtde_control.stopScript()
        running = False  # Stop the receive thread

# Create and start threads
receive_thread = threading.Thread(target=receive_data)
control_thread = threading.Thread(target=control_robot)

receive_thread.start()
control_thread.start()

# Wait for both threads to finish
receive_thread.join()
control_thread.join()


