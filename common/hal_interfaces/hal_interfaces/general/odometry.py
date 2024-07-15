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


### HAL INTERFACE ###
class OdometryNode(Node):

    def __init__(self, topic):
        super().__init__("odometry_node")
        self.sub = self.create_subscription(
            nav_msgs.msg.Odometry, topic, self.listener_callback, 10
        )
        self.last_pose_ = nav_msgs.msg.Odometry()

    def listener_callback(self, msg):
        self.last_pose_ = msg

    def getPose3d(self):
        return odometry2Pose3D(self.last_pose_)
