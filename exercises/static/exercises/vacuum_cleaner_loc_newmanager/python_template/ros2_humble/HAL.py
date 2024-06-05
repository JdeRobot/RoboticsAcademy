import rclpy
import sys
import threading
from console import start_console

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.bumper import BumperNode

### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=sys.argv)
    rclpy.create_node('HAL')

motor_node = MotorsNode("/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/odom")
laser_node = LaserNode("/roombaROS/laser/scan")
bumper_node = BumperNode(
    [
        "/roombaROS/events/right_bumper",
        "/roombaROS/events/center_bumper",
        "/roombaROS/events/left_bumper",
    ]
)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

### GETTERS ###


# Laser
def getLaserData():
    try:
        rclpy.spin_once(laser_node)
        values = laser_node.getLaserData().values
        return values
    except Exception as e:
        print(f"Exception in hal getLaserData {repr(e)}")


# Pose
def getPose3d():
    return odometry_node.getPose3d()


# Bumper
def getBumperData():
    try:
        rclpy.spin_once(bumper_node)
        return bumper_node.getBumperData()
    except Exception as e:
        print(f"Exception in hal getBumper {repr(e)}")


### SETTERS ###


# Linear speed
def setV(v):
    motor_node.sendV(float(v))


# Angular speed
def setW(w):
    motor_node.sendW(float(w))
