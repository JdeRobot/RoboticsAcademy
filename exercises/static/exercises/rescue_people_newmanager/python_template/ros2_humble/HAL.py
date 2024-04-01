import numpy as np
import rclpy
import cv2

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper


### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()

IMG_WIDTH = 320
IMG_HEIGHT = 240

CAM_FRONTAL_TOPIC = "/" + "drone0" + "/sensor_measurements/frontal_camera/image_raw"
CAM_VENTRAL_TOPIC = "/" + "drone0" + "/sensor_measurements/ventral_camera/image_raw"

drone = DroneWrapper()
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

### GETTERS ###


def get_frontal_image():
    try:
        rclpy.spin_once(frontal_camera_node)
        image = frontal_camera_node.getImage().data
        return image
    except Exception as e:
        print(f"Exception in hal get_frontal_image {repr(e)}")


def get_ventral_image():
    try:
        rclpy.spin_once(ventral_camera_node)
        image = ventral_camera_node.getImage().data
        return image
    except Exception as e:
        print(f"Exception in hal get_ventral_image {repr(e)}")


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
