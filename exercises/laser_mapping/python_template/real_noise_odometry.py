import numpy as np
import math
import rclpy
from rclpy.node import Node
import nav_msgs.msg

class Pose3d:
    def __init__(self):
        self.x = 0  
        self.y = 0  
        self.z = 0  
        self.h = 1  
        self.yaw = 0  
        self.pitch = 0  
        self.roll = 0  
        self.q = [0, 0, 0, 0]  
        self.timeStamp = 0  

def quat2Yaw(qw, qx, qy, qz):
    rotateZa0 = 2.0 * (qx * qy + qw * qz)
    rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz
    rotateZ = 0.0
    if rotateZa0 != 0.0 and rotateZa1 != 0.0:
        rotateZ = math.atan2(rotateZa0, rotateZa1)
    return rotateZ

def quat2Pitch(qw, qx, qy, qz):
    rotateYa0 = -2.0 * (qx * qz - qw * qy)
    rotateY = 0.0
    if rotateYa0 >= 1.0:
        rotateY = math.pi / 2.0
    elif rotateYa0 <= -1.0:
        rotateY = -math.pi / 2.0
    else:
        rotateY = math.asin(rotateYa0)
    return rotateY

def quat2Roll(qw, qx, qy, qz):
    rotateXa0 = 2.0 * (qy * qz + qw * qx)
    rotateXa1 = qw * qw - qx * qx - qy * qy + qz * qz
    rotateX = 0.0
    if rotateXa0 != 0.0 and rotateXa1 != 0.0:
        rotateX = math.atan2(rotateXa0, rotateXa1)
    return rotateX

def euler2quat(yaw, pitch, roll):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

def gaussian_noise(x, mu=0.0, std=0.1, noise_level=0.01):
    noise = np.random.normal(mu, std) * noise_level
    x_noisy = x + noise
    return x_noisy

def add_noise(last_pose, new_pose, base_odom, noise_level):
    if last_pose is None:
        return new_pose

    mov_x = new_pose.pose.pose.position.x - last_pose.pose.pose.position.x
    mov_y = new_pose.pose.pose.position.y - last_pose.pose.pose.position.y
    mov_z = new_pose.pose.pose.position.z - last_pose.pose.pose.position.z
    dist = math.sqrt(math.pow(mov_x, 2) + math.pow(mov_y, 2))

    new_ori = new_pose.pose.pose.orientation
    old_ori = last_pose.pose.pose.orientation

    mov_yaw = quat2Yaw(new_ori.w, new_ori.x, new_ori.y, new_ori.z) - quat2Yaw(old_ori.w, old_ori.x, old_ori.y, old_ori.z)
    mov_pitch = quat2Pitch(new_ori.w, new_ori.x, new_ori.y, new_ori.z) - quat2Pitch(old_ori.w, old_ori.x, old_ori.y, old_ori.z)
    mov_roll = quat2Roll(new_ori.w, new_ori.x, new_ori.y, new_ori.z) - quat2Roll(old_ori.w, old_ori.x, old_ori.y, old_ori.z)

    dist = gaussian_noise(dist, noise_level=noise_level)
    mov_yaw = gaussian_noise(mov_yaw, noise_level=noise_level)

    ori = base_odom.pose.pose.orientation
    new_yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z) + mov_yaw
    new_pitch = quat2Pitch(ori.w, ori.x, ori.y, ori.z) + mov_pitch
    new_roll = quat2Roll(ori.w, ori.x, ori.y, ori.z) + mov_roll
    new_ori = euler2quat(new_yaw, new_pitch, new_roll)

    new_odom = nav_msgs.msg.Odometry()
    new_odom.pose.pose.position.x = base_odom.pose.pose.position.x + (dist * math.cos(new_yaw))
    new_odom.pose.pose.position.y = base_odom.pose.pose.position.y + (dist * math.sin(new_yaw))
    new_odom.pose.pose.position.z = base_odom.pose.pose.position.z + mov_z
    new_odom.pose.pose.orientation.x = new_ori[0]
    new_odom.pose.pose.orientation.y = new_ori[1]
    new_odom.pose.pose.orientation.z = new_ori[2]
    new_odom.pose.pose.orientation.w = new_ori[3]
    new_odom.header.stamp.sec = new_pose.header.stamp.sec
    new_odom.header.stamp.nanosec = new_pose.header.stamp.nanosec

    return new_odom

class OdomNoiseInjectorNode(Node):
    def __init__(self):
        super().__init__("odom_noise_injector")
        self.sub = self.create_subscription(
            nav_msgs.msg.Odometry, "/turtlebot3/odom", self.listener_callback, 10
        )
        self.pub_low = self.create_publisher(nav_msgs.msg.Odometry, "/turtlebot3/odom_noisy_low", 10)
        self.pub_med = self.create_publisher(nav_msgs.msg.Odometry, "/turtlebot3/odom_noisy_med", 10)
        self.pub_high = self.create_publisher(nav_msgs.msg.Odometry, "/turtlebot3/odom_noisy_high", 10)
        
        self.last_pose = None
        self.noisy_pose_low = nav_msgs.msg.Odometry()
        self.noisy_pose_med = nav_msgs.msg.Odometry()
        self.noisy_pose_high = nav_msgs.msg.Odometry()

    def listener_callback(self, msg):
        self.noisy_pose_low = add_noise(self.last_pose, msg, self.noisy_pose_low, 0.001)
        self.noisy_pose_med = add_noise(self.last_pose, msg, self.noisy_pose_med, 0.03)
        self.noisy_pose_high = add_noise(self.last_pose, msg, self.noisy_pose_high, 0.06)
        
        self.last_pose = msg
        
        self.pub_low.publish(self.noisy_pose_low)
        self.pub_med.publish(self.noisy_pose_med)
        self.pub_high.publish(self.noisy_pose_high)