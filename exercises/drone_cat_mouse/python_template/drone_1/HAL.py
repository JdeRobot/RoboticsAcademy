import rclpy
import threading
import time
import sys

sys.path.insert(0, "/workspace/code")

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper

IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0

DRONE_NAMESPACE = "drone_mouse"
CAT_NAMESPACE = "drone"

DEFAULT_COURSE = "medium"
COURSE_TIMEOUT = 10.0


### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()


CAM_FRONTAL_TOPIC = "/" + DRONE_NAMESPACE + "/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = "/" + DRONE_NAMESPACE + "/ventral_cam/image_raw"
CAT_POSE_TOPIC = "/" + CAT_NAMESPACE + "/self_localization/pose"
COURSE_TOPIC = "/drone_cat_mouse/course"

drone = DroneWrapper(DRONE_NAMESPACE)
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(frontal_camera_node)
executor.add_node(ventral_camera_node)

cat_position = [0.0, 0.0, 0.0]
course = None


def cat_pose_callback(msg):
    cat_position[0] = msg.pose.position.x
    cat_position[1] = msg.pose.position.y
    cat_position[2] = msg.pose.position.z


def course_callback(msg):
    global course
    course = msg.data


cat_pose_node = rclpy.create_node("cat_pose_node")
cat_pose_node.create_subscription(
    PoseStamped, CAT_POSE_TOPIC, cat_pose_callback, qos_profile_sensor_data
)
# Latched by the world launcher, so it arrives however late this starts
cat_pose_node.create_subscription(
    String,
    COURSE_TOPIC,
    course_callback,
    QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
)
executor.add_node(cat_pose_node)


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


def get_cat_position():
    return list(cat_position)


def get_course():
    give_up_at = time.time() + COURSE_TIMEOUT
    while course is None and time.time() < give_up_at:
        time.sleep(0.05)
    return course or DEFAULT_COURSE


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
