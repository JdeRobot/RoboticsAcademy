import rclpy
import sys
from console import start_console

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.bumper import BumperNode

### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=sys.argv)

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
    try:
        rclpy.spin_once(odometry_node)
        pose = odometry_node.getPose3d()
        return pose
    except Exception as e:
        print(f"Exception in hal getPose3d {repr(e)}")


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
