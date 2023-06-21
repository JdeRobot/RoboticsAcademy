import rclpy
import sys
import cv2

import numpy as np
import threading
import time
from datetime import datetime

from interfaces.camera import ListenerCamera
from interfaces.motors import PublisherMotors

from shared.image import SharedImage
from shared.value import SharedValue

from tello_msgs.msg import TelloResponse

# Hardware Abstraction Layer

class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        print("HAL initializing", flush=True)
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('HAL')
        self.subscription = self.node.create_subscription(
            TelloResponse,
            '/tello_response',
            self.listener_callback,
            10)

        # Shared memory variables
        self.shared_image = SharedImage("guiimage")
        self.shared_vx = SharedValue("velocityX")
        self.shared_vy = SharedValue("velocityY")
        self.shared_vz = SharedValue("velocityZ")
        self.shared_w = SharedValue("angular")
        self.shared_state = SharedValue("state")
        self.velocities = SharedValue("velocities")
        self.shared_response = SharedValue("response")
        
        self.shared_turn_left = SharedValue("turn_left")
        self.shared_turn_right = SharedValue("turn_right")
        self.shared_up = SharedValue("up")
        self.shared_left = SharedValue("left")
        self.shared_right = SharedValue("right")
        self.shared_forward = SharedValue("forward")
        self.shared_back = SharedValue("back")

        # ROS Topics
        self.camera = ListenerCamera("/image_raw")
        self.motors = PublisherMotors("/cmd_vel", 4, 1)
        #self.laser = ListenerLaser("/scan")
        #self.odometry = ListenerPose3d("/odom")
        self.tello_response = 0
        #hz getImage
        self.last_image = np.zeros([500, 500, 3], dtype=np.uint8)
        self.image_counter = 0  
        self.start_time = time.time() 

        self.start_time = 0

        # Update thread
        self.thread = ThreadHAL(self.update_hal)
        print("HAL initialized", flush=True)

    # Function to start the update thread
    def start_thread(self):
        print("HAL thread starting", flush=True)
        self.start_time = time.time()
        self.thread.start()
        
    def listener_callback(self, msg):
        self.tello_response = int(msg.rc)


    # Get Image from ROS Driver Camera
    def getImage(self):
        try:
            rclpy.spin_once(self.camera)
            image = self.camera.getImage()
            self.shared_image.add(image.data)
            if not np.array_equal(image.data, self.last_image):
                self.image_counter += 1  # Incrementa el contador de imágenes
                self.last_image = image.data  # Actualiza la última imagen
            if time.time() - self.start_time >= 60:
                print(f"Number of new images in the last minute: {self.image_counter}")
                self.image_counter = 0  # Resetea el contador de imágenes
                self.start_time = time.time()  # Resetea la hora de inicio
                
            
        except Exception as e:
            print(f"Exception in hal getImage {repr(e)}")

    # Set the velocity
    def setVX(self):
        velocity = self.shared_vx.get()
        self.motors.sendVX(velocity)
        
    # Set the velocity
    def setVY(self):
        velocity = self.shared_vy.get()
        self.motors.sendVX(velocity)
        
    # Set the velocity
    def setVZ(self):
        velocity = self.shared_vz.get()
        self.motors.sendVY(velocity)

    # Set the angular velocity
    def setW(self):
        angular = self.shared_w.get()
        self.motors.sendW(angular)
        
    def state(self):
        roman = 0
        roman = self.shared_state.get()
        if roman == 1:
            self.motors.takeoff()
        elif roman == 2:
            self.motors.pause()
        elif roman == 3:
            self.motors.land()
        if roman != 0:
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if roman == 1:
                    self.motors.takeoff()
                elif roman == 2:
                    self.motors.pause()
                elif roman == 3:
                    self.motors.land()
            self.tello_response = 0
            self.shared_response.add(1)
            self.shared_state.add(0)
            
    def turn_left(self):
        degrees = 0
        degrees = self.shared_turn_left.get()
        print("CURRENT RIGHT:", degrees)
        if degrees > 0:
            self.motors.turn_left(degrees)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.turn_left(degrees)
            self.tello_response = 0
            self.shared_response.add(1)
            self.shared_turn_left.add(0)
        
    def turn_right(self):
        degrees = 0
        degrees = self.shared_turn_right.get()
        if degrees > 0:
            self.motors.turn_right(degrees)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.turn_right(degrees)
            self.tello_response = 0
            self.shared_response.add(1)
            self.shared_turn_right.add(0)
        
    def up(self):
        distance = 0
        distance = self.shared_up.get()
        if distance > 0:
            self.motors.up(distance)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.up(distance)
            self.tello_response = 0
            self.shared_response.add(1)
                
    def left(self):
        distance = 0
        distance = self.shared_left.get()
        if distance > 0:
            self.motors.left(distance)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.left(distance)
            self.tello_response = 0
            self.shared_response.add(1)
                
    def right(self):
        distance = 0
        distance = self.shared_right.get()
        if distance > 0:
            self.motors.right(distance)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.right(distance)
            self.tello_response = 0
            self.shared_response.add(1)
                
    def forward(self):
        distance = 0
        distance = self.shared_forward.get()
        print("CURRENT DISTANCE:", distance)
        if distance > 0:
            self.motors.forward(distance)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.forward(distance)
            self.tello_response = 0
            self.shared_response.add(1)
            self.shared_forward.add(0)
                
    def back(self):
        distance = 0
        distance = self.shared_back.get()
        if distance > 0:
            self.motors.back(distance)
            while self.tello_response != 1:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                self.motors.back(distance)
            self.tello_response = 0
            self.shared_response.add(1)
            
    def sendVelocities(self):
        vx = self.velocities.get()
        vy = self.velocities.get()
        vz = self.velocities.get()
        az = self.velocities.get()
        self.motors.sendVelocities(vx, vy, vz, az)
            

    def update_hal(self):
        self.getImage()
        self.state()
        self.turn_left()
        self.turn_right()
        self.up()
        self.left()
        self.right()
        self.forward()
        self.back()
        self.setW()
        self.setVX()
        self.setVY()
        self.setVZ()

    # Destructor function to close all fds
    def __del__(self):
        pass


class ThreadHAL(threading.Thread):
    def __init__(self, update_function):
        super(ThreadHAL, self).__init__()
        self.time_cycle = 500
        self.update_function = update_function

    def run(self):
        print("Starting HAL thread", flush=True)
        while (True):
            start_time = datetime.now()

            # print(f"Calling update function inside hal thread")
            self.update_function()

            finish_time = datetime.now()

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * \
                1000 + dt.microseconds / 1000.0

            if (ms < self.time_cycle):
                time.sleep((self.time_cycle - ms) / 1000.0)
