import rospy
import numpy as np
import cv2
import threading
import time
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors
from shared.image import SharedImage
from shared.value import SharedValue

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        print("HAL initializing", flush=True)
        rospy.init_node("HAL")

        # Shared memory variables
        self.shared_image = SharedImage("halimage")
        self.shared_v = SharedValue("velocity")
        self.shared_w = SharedValue("angular")

        # ROS Topics
        self.camera = ListenerCamera("/F1ROS/cameraL/image_raw")
        self.motors = PublisherMotors("/F1ROS/cmd_vel", 4, 0.3)

        self.start_time = 0

        # Update thread
        self.thread = ThreadHAL(self.update_hal)
        print("HAL initialized", flush=True)

    # Function to start the update thread
    def start_thread(self):
        print("HAL thread starting", flush=True)
        self.start_time = time.time()
        self.thread.start()

    # Get Image from ROS Driver Camera
    def getImage(self):
        try:
            image = self.camera.getImage().data
            # image = self._get_test_image()
            # print(f"HAL image set, shape: {image.shape}, bytes: {image.nbytes}", flush=True)
            self.shared_image.add(image)
        except Exception as e:
            print(f"Exception in hal getImage {repr(e)}")

    def _get_test_image(self):
        image = np.zeros((640, 480, 3), np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10, 500)
        fontScale = 0.5
        fontColor = (255, 255, 255)
        thickness = 1
        lineType = 2

        cv2.putText(image, f"Image generated in hal.py, running time: {time.time()-self.start_time}",
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)

        return image

    # Set the velocity
    def setV(self):
        velocity = self.shared_v.get()
        self.motors.sendV(velocity)

    # Get the velocity
    def getV(self):
        velocity = self.shared_v.get()
        return velocity

    # Get the angular velocity
    def getW(self):
        angular = self.shared_w.get()
        return angular

    # Set the angular velocity
    def setW(self):
        angular = self.shared_w.get()
        self.motors.sendW(angular)

    def update_hal(self):
        self.getImage()
        self.setV()
        self.setW()

    # Destructor function to close all fds
    def __del__(self):
        self.shared_image.close()
        self.shared_v.close()
        self.shared_w.close()


class ThreadHAL(threading.Thread):
    def __init__(self, update_function):
        super(ThreadHAL, self).__init__()
        self.time_cycle = 80
        self.update_function = update_function

    def run(self):
        print("Starting HAL thread", flush=True)
        while(True):
            start_time = datetime.now()

            # print(f"Calling update function inside hal thread")
            self.update_function()

            finish_time = datetime.now()

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

            if(ms < self.time_cycle):
                time.sleep((self.time_cycle - ms) / 1000.0)
