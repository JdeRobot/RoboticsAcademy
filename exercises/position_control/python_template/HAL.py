import numpy as np
import rclpy
import threading
import time
import sys

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper

IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0


class Beacon:
    def __init__(self, id, pose, active=False, reached=False):
        self.id = id
        self.pose = pose
        self.active = active
        self.reached = reached

    def get_pose(self):
        return self.pose

    def get_id(self):
        return self.id

    def is_reached(self):
        return self.reached

    def set_reached(self, value):
        self.reached = value

    def is_active(self):
        return self.active

    def set_active(self, value):
        self.active = value


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


CAM_FRONTAL_TOPIC = "/" + "drone" + "/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = "/" + "drone" + "/ventral_cam/image_raw"

drone = DroneWrapper("drone")
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(frontal_camera_node)
executor.add_node(ventral_camera_node)


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


### BEACONS ###

# Ground truth poses (world/earth frame), matching the visual markers placed
# in the world file: a flat introductory beacon, then 4 more sitting on top
# of progressively taller posts, ending in a distant tall one deep in the
# forest.
_beacons = []


def init_beacons():
    global _beacons
    _beacons = [
        Beacon("initial", np.array([0.0, 0.0, 0.3]), False, False),
        Beacon("beacon1", np.array([0.0, 18.0, 0.5]), False, False),
        Beacon("beacon2", np.array([18.0, 0.0, 2.0]), False, False),
        Beacon("beacon3", np.array([0.0, -18.0, 3.5]), False, False),
        Beacon("beacon4", np.array([-18.0, 0.0, 5.0]), False, False),
        Beacon("beacon5", np.array([23.0, 23.0, 6.5]), False, False),
    ]


def get_next_beacon():
    for beacon in _beacons:
        if not beacon.is_reached():
            return beacon
    return None


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
