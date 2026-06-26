import numpy as np
import rclpy
import threading
import time
import sys

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper
from as2_msgs.srv import SetPlatformStateMachineEvent
from as2_msgs.msg import PlatformStateMachineEvent

IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0

DRONE_NAMESPACE = "drone1"


def custom_thread_excepthook(args):
    # ignore the errors the spin threads print while things are shutting down
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

### hal init ###

print("HAL initializing drone1 (mouse)", flush=True)
if not rclpy.ok():
    rclpy.init()

CAM_FRONTAL_TOPIC = f"/{DRONE_NAMESPACE}/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = f"/{DRONE_NAMESPACE}/ventral_cam/image_raw"

drone = DroneWrapper(drone_id=DRONE_NAMESPACE)
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

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


### helpers ###


def _call_state_event(event_value):
    """send a state machine event without using asyncio.
    the normal DroneWrapper does this through asyncio.run(), which doesn't work
    with rclpy's futures and crashes. so instead we send the request and just
    check if it's done in a small loop - the drone's own background thread
    finishes the request for us."""
    request = SetPlatformStateMachineEvent.Request()
    request.event.event = event_value
    while not drone.state_event_service_client.wait_for_service(timeout_sec=1.0):
        pass
    future = drone.state_event_service_client.call_async(request)
    while not future.done():
        time.sleep(0.01)


### getters ###


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
    return drone.get_position()


def get_velocity():
    return drone.get_velocity()


def get_yaw_rate():
    return drone.get_yaw_rate()


def get_orientation():
    return drone.get_orientation()


def get_roll():
    return drone.get_roll()


def get_pitch():
    return drone.get_pitch()


def get_yaw():
    return drone.get_yaw()


def get_landed_state():
    return drone.get_landed_state()


### setters ###


def set_cmd_pos(x, y, z, az):
    drone.set_cmd_pos(x, y, z, az)


def set_cmd_vel(vx, vy, vz, az):
    drone.set_cmd_vel(vx, vy, vz, az)


def set_cmd_mix(vx, vy, z, az):
    drone.set_cmd_mix(vx, vy, z, az)


def takeoff(h=3):
    # arm, go offboard, tell the state machine we're taking off, then hold the
    # target height until we're basically there and mark it done
    drone.arm()
    drone.offboard()
    _call_state_event(PlatformStateMachineEvent.TAKE_OFF)
    while abs(drone.position[2] - h) > 0.2:
        drone.set_cmd_pos(drone.position[0], drone.position[1], h, drone.get_yaw())
        time.sleep(0.05)
    _call_state_event(PlatformStateMachineEvent.TOOK_OFF)


def land():
    # come straight down to the ground then disarm
    _call_state_event(PlatformStateMachineEvent.LAND)
    while abs(drone.position[2]) > 0.3:
        drone.set_cmd_pos(drone.position[0], drone.position[1], 0.0, drone.get_yaw())
        time.sleep(0.05)
    _call_state_event(PlatformStateMachineEvent.LANDED)
    drone.disarm()
