import numpy as np
import rclpy
import threading
import time
import sys

from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper

IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0
# todo need to add levels logic and refinement of chasing algorithms need to be done.

# The cat is "drone", the mouse is "drone_1".
DRONE_NAMESPACE = "drone"
MOUSE_NAMESPACE = "drone_1"


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

### HAL INIT ###
print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()


CAM_FRONTAL_TOPIC = "/" + DRONE_NAMESPACE + "/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = "/" + DRONE_NAMESPACE + "/ventral_cam/image_raw"

drone = DroneWrapper(DRONE_NAMESPACE)
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(frontal_camera_node)
executor.add_node(ventral_camera_node)

# Track where the mouse is, so the cat has something to chase. The pose is
# published BEST_EFFORT, so subscribe with the sensor data profile to match it.
_mouse_position = [0.0, 0.0, 0.0]


def _mouse_pose_callback(msg):
    _mouse_position[0] = msg.pose.position.x
    _mouse_position[1] = msg.pose.position.y
    _mouse_position[2] = msg.pose.position.z


mouse_pose_node = rclpy.create_node("mouse_pose_tracker")
mouse_pose_node.create_subscription(
    PoseStamped,
    "/" + MOUSE_NAMESPACE + "/self_localization/pose",
    _mouse_pose_callback,
    qos_profile_sensor_data,
)
executor.add_node(mouse_pose_node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()

### GETTERS ###


def get_frontal_image():
    image = frontal_camera_node.getImage()
    while image is None:
        image = frontal_camera_node.getImage()
    return image.data


def get_ventral_image():
    image = ventral_camera_node.getImage()
    while image is None:
        image = ventral_camera_node.getImage()
    return image.data


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


def get_mouse_position():
    return list(_mouse_position)


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
