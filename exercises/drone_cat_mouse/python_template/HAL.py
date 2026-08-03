import rclpy
import threading
import time
import math

from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper

IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0

DRONE_NAMESPACE = "drone"
MOUSE_NAMESPACE = "drone_mouse"

# The mouse drops out of the sky when it is caught. The cat is flown by student
# code that knows nothing about that, so the same check runs here and the
# commands stop going through.
CATCH_RADIUS = 1.8

### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()


CAM_FRONTAL_TOPIC = "/" + DRONE_NAMESPACE + "/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = "/" + DRONE_NAMESPACE + "/ventral_cam/image_raw"
MOUSE_POSE_TOPIC = "/" + MOUSE_NAMESPACE + "/self_localization/pose"

drone = DroneWrapper(DRONE_NAMESPACE)
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(frontal_camera_node)
executor.add_node(ventral_camera_node)

mouse_position = [0.0, 0.0, 0.0]


def mouse_pose_callback(msg):
    mouse_position[0] = msg.pose.position.x
    mouse_position[1] = msg.pose.position.y
    mouse_position[2] = msg.pose.position.z


mouse_pose_node = rclpy.create_node("mouse_pose_node")
mouse_pose_node.create_subscription(
    PoseStamped, MOUSE_POSE_TOPIC, mouse_pose_callback, qos_profile_sensor_data
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
    return list(mouse_position)


_stopped = False
_mouse_flew = False


def is_caught():
    if not any(abs(p) > 1e-6 for p in mouse_position):
        return False
    here = drone.get_position()
    if here is None or here[2] < 1.0:
        return False
    gap = math.sqrt(sum((here[i] - mouse_position[i]) ** 2 for i in range(3)))
    return gap < CATCH_RADIUS


### SETTERS ###


def _run_over():
   
    global _mouse_flew
    if not any(abs(p) > 1e-6 for p in mouse_position):
        return False
    if mouse_position[2] > 1.0:
        _mouse_flew = True
    return is_caught() or (_mouse_flew and mouse_position[2] < 0.5)


def _stop():
    global _stopped
    if not _run_over():
        return False
    if not _stopped:
        _stopped = True
        drone.set_cmd_vel(0.0, 0.0, 0.0, 0.0)
        drone.land()
    return True


def set_cmd_pos(x, y, z, az):
    if _stop():
        return
    drone.set_cmd_pos(x, y, z, az)


def set_cmd_vel(vx, vy, vz, az):
    if _stop():
        return
    drone.set_cmd_vel(vx, vy, vz, az)


def set_cmd_mix(vx, vy, z, az):
    if _stop():
        return
    drone.set_cmd_mix(vx, vy, z, az)


def takeoff(h=3):
    drone.takeoff(h)


def land():
    drone.land()
