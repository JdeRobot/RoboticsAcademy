import rclpy
import threading
import time
import sys
from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.lidar import LidarNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

# ROS2 init
print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/prius_autoparking/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/prius_autoparking/odom")
laser_front_node = LaserNode("/prius_autoparking/scan_front")
laser_right_node = LaserNode("/prius_autoparking/scan_side")
laser_back_node = LaserNode("/prius_autoparking/scan_back")
lidar_node = LidarNode("/prius_autoparking/pc2")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(laser_front_node)
executor.add_node(laser_right_node)
executor.add_node(laser_back_node)
executor.add_node(lidar_node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()


def getPose3d():
    """
    Returns the current pose of the car in 3D space.

    Returns:
        Pose3d: Object with the following attributes:
            - x (float): X position in meters
            - y (float): Y position in meters
            - z (float): Z position in meters
            - yaw (float): Rotation around Z axis in radians
    """
    return odometry_node.getPose3d()


def getFrontLaserData():
    """
    Returns laser scan data from the front laser sensor.

    Returns:
        LaserData: Object with attributes:
            - values (list[float]): Distance measurements in meters
            - minAngle (float): Start angle of scan in radians
            - maxAngle (float): End angle of scan in radians
            - timeStamp (float): Timestamp of the scan
    """
    laser = laser_front_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_front_node.getLaserData()
        timestamp = laser.timeStamp
    return laser


def getRightLaserData():
    """
    Returns laser scan data from the right-side laser sensor.

    Returns:
        LaserData: Object with attributes:
            - values (list[float]): Distance measurements in meters
            - minAngle (float): Start angle of scan in radians
            - maxAngle (float): End angle of scan in radians
            - timeStamp (float): Timestamp of the scan
    """
    laser = laser_right_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_right_node.getLaserData()
        timestamp = laser.timeStamp
    return laser


def getBackLaserData():
    """
    Returns laser scan data from the rear laser sensor.

    Returns:
        LaserData: Object with attributes:
            - values (list[float]): Distance measurements in meters
            - minAngle (float): Start angle of scan in radians
            - maxAngle (float): End angle of scan in radians
            - timeStamp (float): Timestamp of the scan
    """
    laser = laser_back_node.getLaserData()
    timestamp = laser.timeStamp
    while timestamp == 0.0:
        laser = laser_back_node.getLaserData()
        timestamp = laser.timeStamp
    return laser


def getLidarData():
    """
    Returns 3D LiDAR point cloud data from the Prius autoparking vehicle.

    Returns:
        LidarData: Object with the following attributes:
            - points (list[tuple]): List of (x, y, z) tuples in meters
            - intensities (list[float]): Intensity values per point
                                        (empty if not available)
            - timeStamp (float): Timestamp in seconds
            - min_range (float): Minimum sensor range in meters (default: 0.1)
            - max_range (float): Maximum sensor range in meters (default: 15.0)
            - field_of_view (tuple): (horizontal, vertical) FOV in radians
            - is_dense (bool): True if all points are finite (no NaN/Inf)

    Example:
        lidar = HAL.getLidarData()
        for point in lidar.points:
            x, y, z = point
    """
    lidar = lidar_node.getLidarData()
    timestamp = lidar.timeStamp
    while timestamp == 0.0:
        lidar = lidar_node.getLidarData()
        timestamp = lidar.timeStamp
    return lidar


def setV(velocity):
    """
    Sets the linear velocity of the car.

    Args:
        velocity (float): Linear velocity in m/s.
                         Positive = forward, Negative = backward
    """
    motor_node.sendV(float(velocity))


def setW(velocity):
    """
    Sets the angular velocity of the car.

    Args:
        velocity (float): Angular velocity in rad/s.
                         Positive = left turn, Negative = right turn
    """
    motor_node.sendW(float(velocity))
