from shared.image import SharedImage
from shared.value import SharedValue
import numpy as np
import cv2

# Define HAL functions
class HALFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_frontal_image = SharedImage("halfrontalimage")
        self.shared_ventral_image = SharedImage("halventralimage")
        self.shared_x = SharedValue("x")
        self.shared_y = SharedValue("y")
        self.shared_z = SharedValue("z")
        self.shared_takeoff_z = SharedValue("sharedtakeoffz")
        self.shared_az = SharedValue("az")
        self.shared_azt = SharedValue("azt")
        self.shared_vx = SharedValue("vx")
        self.shared_vy = SharedValue("vy")
        self.shared_vz = SharedValue("vz")
        self.shared_landed_state = SharedValue("landedstate")
        self.shared_position = SharedValue("position")
        self.shared_velocity = SharedValue("velocity")
        self.shared_orientation = SharedValue("orientation")
        self.shared_roll = SharedValue("roll")
        self.shared_pitch = SharedValue("pitch")
        self.shared_yaw = SharedValue("yaw")
        self.shared_yaw_rate = SharedValue("yawrate")


    # Get image function
    def get_frontal_image(self):
        image = self.shared_frontal_image.get()
        return image
    
    # Get left image function
    def get_ventral_image(self):
        image = self.shared_ventral_image.get()
        return image

    def takeoff(self, height):
        self.shared_takeoff_z.add(height)

    def land(self):
        pass
    
    def set_cmd_pos(self, x, y , z, az):
        self.shared_x.add(x)
        self.shared_y.add(y)
        self.shared_z.add(z)
        self.shared_az.add(az)
    
    def set_cmd_vel(self, vx, vy, vz, az):
        self.shared_vx.add(vx)
        self.shared_vy.add(vy)
        self.shared_vz.add(vz)
        self.shared_azt.add(az)
    
    def set_cmd_mix(self, vx, vy, z, az):
        self.shared_vx.add(vx)
        self.shared_vy.add(vy)
        self.shared_vz.add(z)
        self.shared_azt.add(az)
    
    
    def get_position(self):
        position = self.shared_position.get(type_name = "list")
        return position

    def get_velocity(self):
        velocity = self.shared_velocity.get(type_name = "list")
        return velocity

    def get_yaw_rate(self):
        yaw_rate = self.shared_yaw_rate.get(type_name = "value")
        return yaw_rate

    def get_orientation(self):
        orientation = self.shared_orientation.get(type_name = "list")
        return orientation

    def get_roll(self):
        roll = self.shared_roll.get(type_name = "value")
        return roll

    def get_pitch(self):
        pitch = self.shared_pitch.get(type_name = "value")
        return pitch
    
    def get_yaw(self):
        yaw = self.shared_yaw.get(type_name = "value")
        return yaw 

    def get_landed_state(self):
        landed_state = self.shared_landed_state.get(type_name = "value")
        return landed_state

# Define GUI functions
class GUIFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_image = SharedImage("guifrontalimage")
        self.shared_left_image = SharedImage("guiventralimage")

    # Show image function
    def showImage(self, image):
        # Reshape to 3 channel if it has only 1 in order to display it
        if (len(image.shape) < 3):
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        self.shared_image.add(image)

    # Show left image function
    def showLeftImage(self, image):
        # Reshape to 3 channel if it has only 1 in order to display it
        if (len(image.shape) < 3):
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        self.shared_left_image.add(image)