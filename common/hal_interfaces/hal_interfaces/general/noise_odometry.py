import numpy as np
from rclpy.node import Node
from math import asin, atan2, pi
import nav_msgs.msg

### AUXILIARY FUNCTIONS ###
class Pose3d:

    def __init__(self):

        self.x = 0  # X coord [meters]
        self.y = 0  # Y coord [meters]
        self.z = 0  # Z coord [meters]
        self.h = 1  # H param
        self.yaw = 0  # Yaw angle[rads]
        self.pitch = 0  # Pitch angle[rads]
        self.roll = 0  # Roll angle[rads]
        self.q = [0, 0, 0, 0]  # Quaternion
        self.timeStamp = 0  # Time stamp [s]

    def __str__(self):
        s = "Pose3D: {\n   x: " + str(self.x) + "\n   y: " + str(self.y)
        s = s + "\n   z: " + str(self.z) + "\n   H: " + str(self.h)
        s = (
            s
            + "\n   Yaw: "
            + str(self.yaw)
            + "\n   Pitch: "
            + str(self.pitch)
            + "\n   Roll: "
            + str(self.roll)
        )
        s = (
            s
            + "\n   quaternion: "
            + str(self.q)
            + "\n   timeStamp: "
            + str(self.timeStamp)
            + "\n}"
        )
        return s


def quat2Yaw(qw, qx, qy, qz):
    """
    Translates from Quaternion to Yaw.
    @param qw,qx,qy,qz: Quaternion values
    @type qw,qx,qy,qz: float
    @return Yaw value translated from Quaternion
    """

    rotateZa0 = 2.0 * (qx * qy + qw * qz)
    rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz
    rotateZ = 0.0
    if rotateZa0 != 0.0 and rotateZa1 != 0.0:
        rotateZ = atan2(rotateZa0, rotateZa1)

    return rotateZ


def quat2Pitch(qw, qx, qy, qz):
    """
    Translates from Quaternion to Pitch.
    @param qw,qx,qy,qz: Quaternion values
    @type qw,qx,qy,qz: float
    @return Pitch value translated from Quaternion
    """

    rotateYa0 = -2.0 * (qx * qz - qw * qy)
    rotateY = 0.0
    if rotateYa0 >= 1.0:
        rotateY = pi / 2.0
    elif rotateYa0 <= -1.0:
        rotateY = -pi / 2.0
    else:
        rotateY = asin(rotateYa0)

    return rotateY


def quat2Roll(qw, qx, qy, qz):
    """
    Translates from Quaternion to Roll.
    @param qw,qx,qy,qz: Quaternion values
    @type qw,qx,qy,qz: float
    @return Roll value translated from Quaternion
    """
    rotateXa0 = 2.0 * (qy * qz + qw * qx)
    rotateXa1 = qw * qw - qx * qx - qy * qy + qz * qz
    rotateX = 0.0

    if rotateXa0 != 0.0 and rotateXa1 != 0.0:
        rotateX = atan2(rotateXa0, rotateXa1)
    return rotateX

def euler2quat(yaw, pitch, roll):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def odometry2Pose3D(odom):
    """
    Translates from ROS Odometry to JderobotTypes Pose3d.
    @param odom: ROS Odometry to translate
    @type odom: Odometry
    @return a Pose3d translated from odom

    """
    pose = Pose3d()
    ori = odom.pose.pose.orientation

    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.z = odom.pose.pose.position.z
    # pose.h = odom.pose.pose.position.h
    pose.yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z)
    pose.pitch = quat2Pitch(ori.w, ori.x, ori.y, ori.z)
    pose.roll = quat2Roll(ori.w, ori.x, ori.y, ori.z)
    pose.q = [ori.w, ori.x, ori.y, ori.z]
    pose.timeStamp = odom.header.stamp.sec + (odom.header.stamp.nanosec * 1e-9)

    return pose

def gaussian_noise(x,mu=0.0,std=0.1, noise_level = 0.01):
    noise = np.random.normal(mu, std) * noise_level
    x_noisy = x + noise
    return x_noisy 

def add_noise(last_pose, new_pose, base_odom, noise_level):
    
    # First odom is real
    if (last_pose == None):
        return new_pose
    
    # Next odom is movement from last pose to new pose + noise + base odom
    mov_x = new_pose.pose.pose.position.x - last_pose.pose.pose.position.x
    mov_y = new_pose.pose.pose.position.y - last_pose.pose.pose.position.y
    mov_z = new_pose.pose.pose.position.z - last_pose.pose.pose.position.z

    new_ori = new_pose.pose.pose.orientation
    old_ori = last_pose.pose.pose.orientation

    mov_yaw = quat2Yaw(new_ori.w, new_ori.x, new_ori.y, new_ori.z) - quat2Yaw(old_ori.w, old_ori.x, old_ori.y, old_ori.z)
    mov_pitch = quat2Pitch(new_ori.w, new_ori.x, new_ori.y, new_ori.z) - quat2Pitch(old_ori.w, old_ori.x, old_ori.y, old_ori.z)
    mov_roll = quat2Roll(new_ori.w, new_ori.x, new_ori.y, new_ori.z) - quat2Roll(old_ori.w, old_ori.x, old_ori.y, old_ori.z)

    # Add noise
    mov_x = gaussian_noise(mov_x, noise_level = noise_level)
    mov_y = gaussian_noise(mov_y, noise_level = noise_level)
    mov_yaw = gaussian_noise(mov_yaw, noise_level = noise_level)

    # Get new odom angle
    ori = base_odom.pose.pose.orientation

    new_yaw = quat2Yaw(ori.w, ori.x, ori.y, ori.z) + mov_yaw
    new_pitch = quat2Pitch(ori.w, ori.x, ori.y, ori.z) + mov_pitch
    new_roll = quat2Roll(ori.w, ori.x, ori.y, ori.z) + mov_roll
    new_ori = euler2quat(new_yaw, new_pitch, new_roll)

    # Generate new odom
    new_odom = nav_msgs.msg.Odometry()
    
    new_odom.pose.pose.position.x = base_odom.pose.pose.position.x + mov_x
    new_odom.pose.pose.position.y = base_odom.pose.pose.position.y + mov_y
    new_odom.pose.pose.position.z = base_odom.pose.pose.position.z + mov_z
    new_odom.pose.pose.orientation.x = new_ori[0]
    new_odom.pose.pose.orientation.y = new_ori[1]
    new_odom.pose.pose.orientation.z = new_ori[2]
    new_odom.pose.pose.orientation.w = new_ori[3]
    new_odom.header.stamp.sec = new_pose.header.stamp.sec
    new_odom.header.stamp.nanosec = new_pose.header.stamp.nanosec

    return new_odom

### HAL INTERFACE ###
class NoisyOdometryNode(Node):

    def __init__(self, topic):
        super().__init__("noisy_odometry_node")
        self.sub = self.create_subscription(
            nav_msgs.msg.Odometry, topic, self.listener_callback, 10
        )
        self.last_pose_ = None
        self.noisy_pose = nav_msgs.msg.Odometry()

        ### Control the amount of noise ###
        self.noise_level = 0.01 # 0.1 = a lot

    def listener_callback(self, msg):
        # First odom is real
        # Second odom is movement from first real to second real + noise + first odom
        # Third odom is movement from second real to third real + noise + second odom
        self.noisy_pose = add_noise(self.last_pose_, msg, self.noisy_pose, self.noise_level)
        self.last_pose_ = msg

    def getPose3d(self):
        return odometry2Pose3D(self.noisy_pose)
