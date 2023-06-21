from shared.image import SharedImage
from shared.value import SharedValue
from interfaces.motors import PublisherMotors

import cv2

# Define HAL functions
class HALFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_image = SharedImage("guiimage")
        self.shared_vx = SharedValue("velocityX")
        self.shared_vy = SharedValue("velocityY")
        self.shared_vz = SharedValue("velocityZ")
        self.shared_w = SharedValue("angular")
        self.state = SharedValue("state")
        self.shared_turn_left = SharedValue("turn_left")
        self.shared_turn_right = SharedValue("turn_right")
        self.shared_up = SharedValue("up")
        self.shared_left = SharedValue("left")
        self.shared_right = SharedValue("right")
        self.shared_forward = SharedValue("forward")
        self.shared_back = SharedValue("back")
        self.velocities = SharedValue("velocities")
        self.shared_response = SharedValue("response")
        self.motors = PublisherMotors("/cmd_vel", 4, 0.3)
        self.tello_response = 0
        
    # Get image function
    def getImage(self):
        image = self.shared_image.get()
        return image
    
    def sendVX(self, velocity):
        self.shared_vx.add(velocity)
        
    def sendVY(self, velocity):
        self.shared_vy.add(velocity)

    # Send velocity function
    def sendVZ(self, velocity):
        self.shared_vz.add(velocity)

    # Send angular velocity function
    def sendW(self, angular):
        self.shared_w.add(angular)
        
    # Send velocity function
    def takeoff(self):
        #self.motors.takeoff()
        self.state.add(1)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0

    # Send angular velocity function
    def pause(self):
        #self.motors.pause()
        self.state.add(2)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
    
    def land(self):
        #self.motors.land()
        self.state.add(3)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def turn_left(self, degrees):
        #self.motors.land()
        self.shared_turn_left.add(degrees)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0

    def turn_right(self, degrees):
        #self.motors.land()
        self.shared_turn_right.add(degrees)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def up(self, distance):
        #self.motors.land()
        self.shared_up.add(distance)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def left(self, distance):
        #self.motors.land()
        self.shared_left.add(distance)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def right(self, distance):
        #self.motors.land()
        self.shared_right.add(distance)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def forward(self, distance):
        #self.motors.land()
        self.shared_forward.add(distance)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def back(self, distance):
        #self.motors.land()
        self.shared_back.add(distance)
        while self.tello_response < 1:
            self.tello_response = self.shared_response.get()
        self.tello_response = 0
        
    def sendVelocities(self, vx, vy, vz, az):
        self.velocities.add(az)
        self.velocities.add(vz)
        self.velocities.add(vy)
        self.velocities.add(vx)


# Define GUI functions
class GUIFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_image = SharedImage("guiimage")

    # Show image function
    def showImage(self, image):
        # Reshape to 3 channel if it has only 1 in order to display it
        if (len(image.shape) < 3):
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        self.shared_image.add(image)