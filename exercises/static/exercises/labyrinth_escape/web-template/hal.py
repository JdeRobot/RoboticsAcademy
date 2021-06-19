import numpy as np
import rospy
import cv2

from drone_wrapper import DroneWrapper


# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240
    
    def __init__(self):
        rospy.init_node("HAL")
    
        self.image = None
        self.drone = DroneWrapper(name="rqt")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance
    
    # Get Image from ROS Driver Camera
    def get_frontal_image(self):
        image = self.drone.get_frontal_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image_rgb

    def get_ventral_image(self):
        image = self.drone.get_ventral_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image_rgb

    def get_position(self):
        pos = self.drone.get_position()
        return pos

    def get_velocity(self):
        vel = self.drone.get_velocity()
        return vel

    def get_yaw_rate(self):
        yaw_rate = self.drone.get_yaw_rate()
        return yaw_rate

    def get_orientation(self):
        orientation = self.drone.get_orientation()
        return orientation

    def get_roll(self):
        roll = self.drone.get_roll()
        return roll

    def get_pitch(self):
        pitch = self.drone.get_pitch()
        return pitch

    def get_yaw(self):
        yaw = self.drone.get_yaw()
        return yaw

    def get_landed_state(self):
        state = self.drone.get_landed_state()
        return state

    def set_cmd_pos(self, x, y, z, yaw):
        self.drone.set_cmd_pos(x, y, z, yaw)

    def set_cmd_vel(self, vx, vy, vz, yaw_rate):
        self.drone.set_cmd_vel(vx, vy, vz, yaw_rate)

    def set_cmd_mix(self, vx, vy, z, yaw_rate):
        self.drone.set_cmd_mix(vx, vy, z, yaw_rate)

    def takeoff(self, h=3):
        self.drone.takeoff(h)

    def land(self):
        self.drone.land()
