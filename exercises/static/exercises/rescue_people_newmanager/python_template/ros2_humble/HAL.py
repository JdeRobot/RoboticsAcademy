import numpy as np
import rclpy
import cv2

from jderobot_drones.drone_wrapper import DroneWrapper
from jderobot_drones.image_sub import ImageSubscriberNode


### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()

IMG_WIDTH = 320
IMG_HEIGHT = 240

drone = DroneWrapper()
cam = ImageSubscriberNode()

### GETTERS ###


def get_frontal_image():
    image = cam.get_frontal_image()
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image_rgb


def get_ventral_image():
    image = cam.get_ventral_image()
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image_rgb


def get_position():
    pos = drone.get_position()
    return pos


def get_velocity():
    vel = drone.get_velocity()
    return vel


def get_yaw_rate():
    yaw_rate = drone.get_yaw_rate()
    return yaw_rate


def get_orientation():
    orientation = drone.get_orientation()
    return orientation


def get_roll():
    roll = drone.get_roll()
    return roll


def get_pitch():
    pitch = drone.get_pitch()
    return pitch


def get_yaw():
    yaw = drone.get_yaw()
    return yaw


def get_landed_state():
    state = drone.get_landed_state()
    return state


### SETTERS ###


def set_cmd_pos(x, y, z, az):
    drone.set_cmd_pos(x, y, z, az)


def set_cmd_vel(vx, vy, vz, az):
    drone.set_cmd_vel(vx, vy, vz, az)


def set_cmd_mix(vx, vy, z, az):
    drone.set_cmd_mix(vx, vy, z, az)


def takeoff(h=3):
    drone.takeoff(h)


def land():
    drone.land()
