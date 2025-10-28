import rclpy
import threading
import time
import sys
from geometry_msgs.msg import Twist

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.sim_time import SimTimeNode

# Hardware Abstraction Layer
freq = 30.0  # 30 Hz loop frequency

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/odom")
laser_node = LaserNode("/scan")
sim_time_node = SimTimeNode()

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(laser_node)
executor.add_node(sim_time_node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()

# ===========================
# CMD_VEL continuous publisher
# ===========================
cmd_vel_node = rclpy.create_node("hal_cmdvel_publisher")
cmd_vel_pub = cmd_vel_node.create_publisher(Twist, "/cmd_vel", 4)

linear_vel = 0.0
angular_vel = 0.0
publish_rate = 100.0  # Hz

def __publish_cmdvel():
    global linear_vel, angular_vel
    rate = 1.0 / publish_rate
    twist = Twist()
    while rclpy.ok():
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        cmd_vel_pub.publish(twist)
        time.sleep(rate)

cmdvel_thread = threading.Thread(target=__publish_cmdvel, daemon=True)
cmdvel_thread.start()

def getPose3d():
    return odometry_node.getPose3d()

def getSimTime():
    return sim_time_node.getSimTime()

def getLaserData():
    laser_data = laser_node.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser_node.getLaserData()
    return laser_data

def setV(velocity):
    """Update target linear velocity (m/s)."""
    global linear_vel
    linear_vel = float(velocity)

def setW(angular_velocity):
    """Update target angular velocity (rad/s)."""
    global angular_vel
    angular_vel = float(angular_velocity)

def stop():
    global linear_vel, angular_vel
    linear_vel = 0.0
    angular_vel = 0.0

print("HAL ready — publishing /cmd_vel continuously", flush=True)

def lift():
    global liftState
    liftState = True
    platform_pub.load()


def putdown():
    global liftState
    liftState = False
    platform_pub.unload()


def getLiftState():
    global liftState
    return liftState
