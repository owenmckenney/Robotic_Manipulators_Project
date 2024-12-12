from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time

class UR5SafeControl:
    def __init__(self, ip, x_min, x_max, y_min, y_max, z_min, z_max):
        self.ip = ip
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

        self.rtde_control = RTDEControlInterface(self.ip)
        self.rtde_receive = RTDEReceiveInterface(self.ip)
        

    def within_limits(self, x, y, z):
        return (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max and self.z_min <= z <= self.z_max)
    
    def clamp(self, x, y, z):
        clamped_x = max(self.x_min, min(self.x_max, x))
        clamped_y = max(self.y_min, min(self.y_max, y))
        clamped_z = max(self.z_min, min(self.z_max, z))

        return clamped_x, clamped_y, clamped_z

    def moveL(self, pose, velocity=0.1, acceleration=0.1):
        x, y, z = pose[0], pose[1], pose[2]
        clamped_x, clamped_y, clamped_z = self.clamp(x, y, z)

        if (x, y, z) != (clamped_x, clamped_y, clamped_z):
            print(f"Pose out of bounds:" f"(Original: x={x}, y={y}, z={z}, Adjusted: x={clamped_x}, y={clamped_y}, z={clamped_z})")

        pose = [clamped_x, clamped_y, clamped_z, pose[3], pose[4], pose[5]]
        self.rtde_control.moveL(pose, velocity, acceleration)
        
    def servoL(self, pose, lookahead_t=0.1, gain=0.1):
        x, y, z = pose[0], pose[1], pose[2]
        clamped_x, clamped_y, clamped_z = self.clamp(x, y, z)

        if (x, y, z) != (clamped_x, clamped_y, clamped_z):
            print(f"Pose out of bounds:" f"(Original: x={x}, y={y}, z={z}, Adjusted: x={clamped_x}, y={clamped_y}, z={clamped_z})")

        pose = [clamped_x, clamped_y, clamped_z, pose[3], pose[4], pose[5]]
        self.rtde_control.moveL(pose, lookahead_t, gain)

    def moveJ(self, joint_positions, velocity=0.1, acceleration=0.1):
        self.rtde_control.moveJ(joint_positions, velocity, acceleration)

        tcp_pose = self.rtde_receive.getActualTCPPose()
        x, y, z = tcp_pose[0], tcp_pose[1], tcp_pose[2]
        if not self.within_limits(x, y, z):
            print(f"MoveJ out of bounds: (x: {x}, y: {y}, z: {z})")
            self.rtde_control.stopL()
            
    def speedL(self, xd, time=0.05, acceleration=0.1):
        self.rtde_control.speedL(xd, acceleration, time)
        
        # tcp_pose = self.rtde_receive.getActualTCPPose()
        # x, y, z = tcp_pose[0], tcp_pose[1], tcp_pose[2]
        # if not self.within_limits(x, y, z):
        #     print(f"SpeedL out of bounds: (x: {x}, y: {y}, z: {z})")
        #     self.rtde_control.stopL()

    def get_joint_positions(self):
        return self.rtde_receive.getActualQ()
    
    def close(self):
        self.rtde_control.stopScript()
        self.rtde_control.disconnect()
        self.rtde_receive.disconnect()

if __name__ == "__main__":
    ip = "10.168.18.105"

    x_min = 0.2 
    x_max = 0.8  
    y_min = -0.3 
    y_max = 0.3  
    z_min = -0.01  
    z_max = 0.2  

    ur5 = UR5SafeControl(ip, x_min, x_max, y_min, y_max, z_min, z_max)

    try:
        pose = [0.3, 0, -0.01, 0.0, -3.1415, 0]
        ur5.moveL(pose)

    except KeyboardInterrupt:
        print("Exit.")
    finally:
        ur5.close()
