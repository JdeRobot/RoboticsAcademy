import rospy
import cv2

from drone_wrapper import DroneWrapper


# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        rospy.init_node("HAL_mouse")

        self.image = None
        self.mouse = DroneWrapper(name="rqt", ns="mouse/")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    # Get Image from ROS Driver Camera
    def get_frontal_image(self):
        image = self.mouse.get_frontal_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image_rgb

    def get_ventral_image(self):
        image = self.mouse.get_ventral_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image_rgb

    def get_position(self):
        pos = self.mouse.get_position()
        return pos

    def get_velocity(self):
        vel = self.mouse.get_velocity()
        return vel

    def get_yaw_rate(self):
        yaw_rate = self.mouse.get_yaw_rate()
        return yaw_rate

    def get_orientation(self):
        orientation = self.mouse.get_orientation()
        return orientation

    def get_roll(self):
        roll = self.mouse.get_roll()
        return roll

    def get_pitch(self):
        pitch = self.mouse.get_pitch()
        return pitch

    def get_yaw(self):
        yaw = self.mouse.get_yaw()
        return yaw

    def get_landed_state(self):
        state = self.mouse.get_landed_state()
        return state

    def set_cmd_pos(self, x, y, z, az):
        self.mouse.set_cmd_pos(x, y, z, az)

    def set_cmd_vel(self, vx, vy, vz, az):
        self.mouse.set_cmd_vel(vx, vy, vz, az)

    def set_cmd_mix(self, vx, vy, z, az):
        self.mouse.set_cmd_mix(vx, vy, z, az)

    def takeoff(self, h=3):
        self.mouse.takeoff(h)

    def land(self):
        self.mouse.land()
