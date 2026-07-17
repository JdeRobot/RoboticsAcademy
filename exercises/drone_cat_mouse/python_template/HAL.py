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

DRONE_NAMESPACE = "drone"

# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

### HAL INIT ###

print("HAL initializing drone (cat)", flush=True)
if not rclpy.ok():
    rclpy.init()

CAM_FRONTAL_TOPIC = f"/{DRONE_NAMESPACE}/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = f"/{DRONE_NAMESPACE}/ventral_cam/image_raw"

drone = DroneWrapper(drone_id=DRONE_NAMESPACE)
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

### mouse position tracking ###
# the cat needs to know where the mouse is to chase it.
# one thing to watch out for: don't give this its own
# MultiThreadedExecutor.spin(). i did that first and the cat just sat there.
# the reason is spin() never pauses, so it takes all the cpu time and the
# drone's own background thread barely gets to run. that thread is what reads
# the controller status, and set_cmd_vel waits for the controller to report
# it's in SPEED mode before it sends anything. if that thread can't run, the
# status never updates, set_cmd_vel waits 5s and then gives up - so the cat
# never moves. so instead we just add this to the camera loop below, which
# does a little work then sleeps, leaving room for everything else.

_mouse_position = [20.0, 5.0, 0.0]


def _mouse_pose_callback(msg):
    _mouse_position[0] = msg.pose.position.x
    _mouse_position[1] = msg.pose.position.y
    _mouse_position[2] = msg.pose.position.z


_mouse_pose_node = rclpy.create_node("cat_mouse_pose_tracker")
# this topic is published as BEST_EFFORT. if you subscribe with the default
# settings (which are RELIABLE) the two don't match and you receive nothing,
# so get_mouse_position would just keep returning the default below. using
# the sensor-data profile makes both sides BEST_EFFORT, so the messages flow.
_mouse_pose_sub = _mouse_pose_node.create_subscription(
    PoseStamped,
    "/drone1/self_localization/pose",
    _mouse_pose_callback,
    qos_profile_sensor_data,
)

executor.add_node(_mouse_pose_node)

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
<<<<<<< HEAD

=======
>>>>>>> 5f1bdec3a (Compile cpp changes)

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
