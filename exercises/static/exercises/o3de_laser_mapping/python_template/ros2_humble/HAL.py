import rclpy
import threading
import time
import sys
import subprocess
import re
from geometry_msgs.msg import Twist

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from real_noise_odometry import NoisyOdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.sim_time import SimTimeNode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from math import atan2, asin, pi

# Hardware Abstraction Layer
freq = 120.0

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init(args=None)

### HAL INIT ###
motor_node = MotorsNode("/cmd_vel", 4, 0.3)
odometry_node = OdometryNode("/odom")
noisy_odometry_node = NoisyOdometryNode("/odom")
laser_node = LaserNode("/scan")
sim_time_node = SimTimeNode()

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(odometry_node)
executor.add_node(noisy_odometry_node)
executor.add_node(laser_node)
executor.add_node(sim_time_node)

class LaserData:
    def __init__(self, angle_min, angle_max, angle_increment, ranges, angles):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.angles = angles
        self.values = ranges
        self.maxRange = max(ranges) if len(ranges) > 0 else 0

def getPose3d():
    """Lee la posición actual del robot usando ros2 topic echo."""
    try:
        cmd = ["ros2", "topic", "echo", "/odom", "--field", "pose.pose", "--once"]
        output = subprocess.check_output(cmd, text=True)

        num = r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"

        # Extraer posición y orientación
        pattern = rf"""
            position:\s*\n
            \s*x:\s*{num}\s*\n
            \s*y:\s*{num}\s*\n
            \s*z:\s*{num}.*?\n
            orientation:\s*\n
            \s*x:\s*{num}\s*\n
            \s*y:\s*{num}\s*\n
            \s*z:\s*{num}\s*\n
            \s*w:\s*{num}
        """

        match = re.search(pattern, output, re.S | re.X)
        if not match:
            return None

        px, py, pz, qx, qy, qz, qw = map(float, match.groups())
        
        yaw = atan2(2*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz)
        pitch_val = -2*(qx*qz - qw*qy)
        pitch_val = max(-1.0, min(1.0, pitch_val))
        pitch = asin(pitch_val)
        roll = atan2(2*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        
        yaw -= pi / 2

        # Normalizar yaw a [-pi, pi]
        if yaw > pi:
            yaw -= 2 * pi
        elif yaw < -pi:
            yaw += 2 * pi
        
        class Pose3D:
            pass

        pose = Pose3D()
        pose.x = px
        pose.y = py
        pose.z = pz
        pose.yaw = yaw
        pose.pitch = pitch
        pose.roll = roll
        pose.q = [qw, qx, qy, qz]
        pose.timeStamp = 0.0
        pose.h = 1.0

        return pose

    except subprocess.CalledProcessError:
        return None

def getOdom():
    "Pos is gotten from odometry"
    return getPose3d()

# Extraer valores principales
def extract_float(field, output):
    match = re.search(rf"{field}:\s*([-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?\d+)?)", output)
    return float(match.group(1)) if match else None

def getLaserData():
    cmd = ["ros2", "topic", "echo", "/scan", "--once", "--full-length"]
    output = subprocess.check_output(cmd, text=True)

    angle_min = extract_float("angle_min", output)
    angle_max = extract_float("angle_max", output)
    angle_increment = extract_float("angle_increment", output)

    # Extraer todos los valores numéricos válidos de la lista "ranges:"
    ranges_match = re.search(
    r"ranges:\s*((?:[^\n]*\n)+?)\s*intensities:",
    output,
    re.S)
    if not ranges_match:
        print("[WARN] No se encontraron rangos en el mensaje /scan.")
        return None

    ranges_text = ranges_match.group(1)
    ranges = [float(x) for x in re.findall(r"[-+]?\d*\.\d+", ranges_text)]

    # Calcular ángulos correspondientes
    if angle_min is not None and angle_increment is not None:
        angles = [angle_min + i * angle_increment for i in range(len(ranges))]
    else:
        angles = list(range(len(ranges)))
    
    laser = LaserData(angle_min, angle_max, angle_increment, ranges, angles)

    return laser
    
def getSimTime():
    return sim_time_node.getSimTime()

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


# ===========================
# CMD_VEL continuous publisher
# ===========================
cmd_vel_node = rclpy.create_node("hal_cmdvel_publisher")
cmd_vel_pub = cmd_vel_node.create_publisher(Twist, "/cmd_vel", 4)

pose3d = None
laser_data = None

def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)

linear_vel = 0.0
angular_vel = 0.0
publish_rate = 60.0  # Hz

def __publish_cmdvel():
    global linear_vel, angular_vel
    rate = 1.0 / publish_rate
    twist = Twist()
    while rclpy.ok():
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        cmd_vel_pub.publish(twist)
        time.sleep(rate)

def __update_pose():
    global pose3d
    rate = 1.0 / freq
    while rclpy.ok():
        pose3d = getPose3d()
        time.sleep(rate)

def __update_laser():
    global laser_data
    rate = 1.0 / freq
    while rclpy.ok():
        data = laser_node.getLaserData()
        if len(data.values) > 0:
            laser_data = data
        time.sleep(rate)

def __update_noisy_pose():
    global noisy_pose3d
    rate = 1.0 / freq
    while rclpy.ok():
        noisy_pose3d = noisy_odometry_node.getPose3d()
        time.sleep(rate)



# Lanzar los hilos de actualización
executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()
cmdvel_thread = threading.Thread(target=__publish_cmdvel, daemon=True)
cmdvel_thread.start()
pose_thread = threading.Thread(target=__update_pose, daemon=True)
pose_thread.start()
scan_thread = threading.Thread(target=__update_laser, daemon=True)
scan_thread.start()
noisy_pose_thread = threading.Thread(target=__update_noisy_pose, daemon=True)
noisy_pose_thread.start()

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
    
    
