from UR5SafeControl import UR5SafeControl
from camera import Camera
import time

if __name__ == "__main__":
    camera = Camera(0)

    ip = "10.168.18.104"
    x_min = 0.2 
    x_max = 0.8  
    y_min = -0.3 
    y_max = 0.3  
    z_min = -0.01  
    z_max = 0.2  

    ur5 = UR5SafeControl(ip, x_min, x_max, y_min, y_max, z_min, z_max)

    close = False
    last_joint_positions = []

    while not close:
        camera.get_frame()
        poses = camera.detect_cubes(output=False)
        fingers = camera.detect_hand(output=False)

        print(poses)
        print(poses[0]['angle'])
        camera.show_frame()

        joint_positions = ur5.get_joint_positions()

        if joint_positions != last_joint_positions:
            try:
                # time.sleep(1)
                joint_positions = ur5.get_joint_positions()
                joint_positions[5] = poses[0]["angle"]
                ur5.moveJ(joint_positions)

                last_joint_positions = joint_positions

            except KeyboardInterrupt:
                print("Exit.")
            finally:
                # ur5.close()
                pass

        close = camera.detect_close()
        
        # time.sleep(0.01)

    
    camera.quit()

