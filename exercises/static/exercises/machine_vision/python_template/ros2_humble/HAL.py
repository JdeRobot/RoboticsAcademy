#!/usr/bin/env python3

print("HAL initializing", flush=True)
##############################################################################
# JdeROBOT ROBOTICS ACADEMY (http://jderobot.github.io/RoboticsAcademy/)
#  API PICK and PLACE exercise, including:
#   Robot Info: get_TCP_pose, get_Joint_states
#   Kinematics: MoveAbsJ, MoveJoint, MoveLinear, MoveSingleJ 
#               MoveRelLinear, MoveRelReor
#   Gripper: GripperSet, attach, dettach
#   Perception: Color and Shape filtering capabilities
#   
#   VERSION: 1.1
# 	DATE: 	 April 21, 2025
#   AUTHOR:  Diego Martin (diego.martin.martin@gmail.com)
# 
# ======= Acknowledgments =======
#  IFRA-Cranfield nice "ROS2 Sim-to-Real Robot Control" package 
#  URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl
##############################################################################


import sys, os, time, math
import rclpy
import numpy as np
import yaml
import copy
from rclpy.time import Time

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf2_geometry_msgs.tf2_geometry_msgs

from sensor_msgs.msg import JointState
from ros2srrc_data.msg import Robpose
from linkattacher_msgs.srv import AttachLink, DetachLink
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from pcl_filter_msgs.msg import ColorFilter, ShapeFilter
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Build PATH and import Python classes from IFRA package:
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')

PATH_ROB = PATH + "/robot"
sys.path.append(PATH_ROB)
from robot import RBT

PATH_EE = PATH + "/endeffector"
sys.path.append(PATH_EE)
from robotiq_ur import RobotiqGRIPPER

# Import ROS2 Custom Messages from IFRA package:
from ros2srrc_data.msg import Action, Joint, Joints, Xyz, Ypr, Robpose

# Global variables
rclpy.init(args=None)
UR5 = RBT()
_home_joints = [0.0, -90.0, 0.0, 0.0, -90.0, 0.0]
_perception_manager = None

#################################### UTILITY CLASSES ###################################################

class Object:
    def __init__(self, relative_pose, abs_pose, height, width, length, shape, color):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        self.shape = shape
        self.color = color

class WorkSpace:
    def __init__(self, x, y, z, min_r, max_r, min_z):
        self.x = x
        self.y = y
        self.z = z
        self.min_r = min_r
        self.max_r = max_r
        self.min_z = min_z

#################################### PERCEPTION MANAGER ###################################################

class PerceptionManager(Node):
    """Unified perception management for robot workspace"""
    
    def __init__(self):
        super().__init__('perception_manager')
        
        self.object_list = {}
        self.goal_list = {}
        self.color_shape_converter = {
            "red": 1, "green": 2, "blue": 3, "purple": 4,
            "sphere": 1, "cylinder": 2
        }
        
        # Publishers
        self.message_pub = self.create_publisher(String, '/gui_message', 10)
        self.updatepose_pub = self.create_publisher(Bool, '/updatepose', 10)
        self.color_filter_pub = self.create_publisher(ColorFilter, '/start_color_filter', 10)
        self.shape_filter_pub = self.create_publisher(ShapeFilter, '/start_shape_filter', 10)
        self.env_scan_pub = self.create_publisher(Bool, '/trigger_env_scan', 10)
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        
        # Load configuration
        self.load_models_info()
        self.set_target_info()
        self.get_workspace()
        
        self.get_logger().info('Perception Manager initialized')

    def load_models_info(self):
        """Load object information from YAML configuration file"""
        try:
            package_share_dir = get_package_share_directory('machine_vision_exercise')
            filename = os.path.join(package_share_dir, 'config', 'models_info.yaml')
            
            with open(filename, 'r') as file:
                objects_info = yaml.safe_load(file)
                robot_x = objects_info["robot"]["pose"]["x"]
                robot_y = objects_info["robot"]["pose"]["y"]
                robot_z = objects_info["robot"]["pose"]["z"]

                objects = objects_info["objects"]
                
                for name, spec in objects.items():
                    shape = spec["shape"]
                    color = spec["color"]
                    x, y, z = spec["pose"]["x"], spec["pose"]["y"], spec["pose"]["z"]
                    roll, pitch, yaw = spec["pose"]["roll"], spec["pose"]["pitch"], spec["pose"]["yaw"]
                    object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)

                    p = PoseStamped()
                    p.header.frame_id = "base_link"
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.pose.position.x = float(x - robot_x)
                    p.pose.position.y = float(y - robot_y)
                    p.pose.position.z = float(z - robot_z)

                    q = quaternion_from_euler(roll, pitch, yaw)
                    p.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

                    if shape == "box":
                        sx, sy, sz = spec["size"]["x"], spec["size"]["y"], spec["size"]["z"]
                        p.pose.position.z += float(sz/2)
                        self.object_list[name] = Object(p.pose, object_pose, sz, sy, sx, shape, color)
                    elif shape == "cylinder":
                        height, radius = spec["size"]["height"], spec["size"]["radius"]
                        p.pose.position.z += float(height/2)
                        self.object_list[name] = Object(p.pose, object_pose, height, radius*2, radius*2, shape, color)
                    elif shape == "sphere":
                        radius = spec["size"]
                        p.pose.position.z += float(radius)
                        self.object_list[name] = Object(p.pose, object_pose, radius*2, radius*2, radius*2, shape, color)
                        
        except Exception as e:
            self.get_logger().error(f'Failed to load models info: {e}')

    def set_target_info(self):
        """Load target information from YAML configuration file"""
        try:
            package_share_dir = get_package_share_directory('machine_vision_exercise')
            filename = os.path.join(package_share_dir, 'config', 'models_info.yaml')
            
            with open(filename, 'r') as file:
                objects_info = yaml.safe_load(file)
                robot_x = objects_info["robot"]["pose"]["x"]
                robot_y = objects_info["robot"]["pose"]["y"]
                robot_z = objects_info["robot"]["pose"]["z"]

                targets = objects_info["targets"]
                for name, target in targets.items():
                    position = Point()
                    position.x = float(target["x"] - robot_x)
                    position.y = float(target["y"] - robot_y)
                    position.z = float(target["z"])
                    self.goal_list[name] = position
                    
        except Exception as e:
            self.get_logger().error(f'Failed to load target info: {e}')

    def get_workspace(self):
        """Load workspace configuration"""
        try:
            package_share_dir = get_package_share_directory('machine_vision_exercise')
            filename = os.path.join(package_share_dir, 'config', 'joints_setup.yaml')
            
            with open(filename, 'r') as file:
                joints_setup = yaml.safe_load(file)
                workspace = joints_setup["workspace"]
                x, y, z = workspace["center"]["x"], workspace["center"]["y"], workspace["center"]["z"]
                min_r, max_r = workspace["r"]["min"], workspace["r"]["max"]
                min_z = workspace["min_z"]
                self.workspace = WorkSpace(x, y, z, min_r, max_r, min_z)
                
        except Exception as e:
            self.get_logger().error(f'Failed to load workspace info: {e}')

    def send_message(self, message):
        """Send message to GUI"""
        msg = String()
        msg.data = message
        self.message_pub.publish(msg)

    def updatepose_trigger(self, value):
        """Trigger pose update"""
        msg = Bool()
        msg.data = value
        self.updatepose_pub.publish(msg)

    def start_color_filter(self, color, rmax, rmin, gmax, gmin, bmax, bmin):
        """Start color filtering"""
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.rmax = rmax
        color_filter.rmin = rmin
        color_filter.gmax = gmax
        color_filter.gmin = gmin
        color_filter.bmax = bmax
        color_filter.bmin = bmin
        color_filter.status = True
        self.color_filter_pub.publish(color_filter)

    def stop_color_filter(self, color):
        """Stop color filtering"""
        color_filter = ColorFilter()
        color_filter.color = self.color_shape_converter[color]
        color_filter.status = False
        self.color_filter_pub.publish(color_filter)

    def start_shape_filter(self, color, shape, radius):
        """Start shape filtering"""
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.radius = radius
        shape_filter.status = True
        self.shape_filter_pub.publish(shape_filter)

    def stop_shape_filter(self, color, shape):
        """Stop shape filtering"""
        shape_filter = ShapeFilter()
        shape_filter.color = self.color_shape_converter[color]
        shape_filter.shape = self.color_shape_converter[shape]
        shape_filter.status = False
        self.shape_filter_pub.publish(shape_filter)

    def get_object_list(self):
        """Get list of object names"""
        return list(self.object_list.keys())

    def get_target_list(self):
        """Get list of target names"""
        return list(self.goal_list.keys())

    def get_object_pose(self, object_name):
        """Get object pose"""
        if object_name in self.object_list:
            return copy.deepcopy(self.object_list[object_name].relative_pose)
        return None

    def get_object_position(self, object_name):
        """Get object position using TF2"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', object_name, rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            position = [
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z + 0.03)
            ]
            
            self.get_logger().info("*************************************")
            self.get_logger().info(f"Detected position of {object_name}:")
            self.get_logger().info(f"{position}")
            self.get_logger().info("*************************************")
            return position
            
        except Exception as e:
            self.get_logger().error(f"Cannot find desired object {object_name}: {e}")
            return None

    def get_object_info(self, object_name):
        """Get object information"""
        if object_name in self.object_list:
            obj = copy.deepcopy(self.object_list[object_name])
            return obj.height, obj.width, obj.length, obj.shape, obj.color
        return None, None, None, None, None

    def get_target_position(self, target_name):
        """Get target position"""
        if target_name in self.goal_list:
            return self.goal_list[target_name]
        return None

    def gripper_setting_percentage(self, diameter, max_open_m=0.085):
        """Convert object diameter to gripper closure percentage"""
        percentage = (1 - (diameter / max_open_m)) * 100.0
        return percentage

    def is_inside_workspace(self, x, y, z):
        """Check if position is inside workspace"""
        if z > self.workspace.min_z:
            dx = x - self.workspace.x
            dy = y - self.workspace.y
            dz = z - self.workspace.z
            r = math.sqrt(dx**2 + dy**2 + dz**2)
            if self.workspace.min_r < r < self.workspace.max_r:
                return True
        return False

    def pose2msg(self, x, y, z, roll, pitch, yaw):
        """Convert pose parameters to geometry_msgs/Pose"""
        pose = Pose()
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        return pose

    def msg2pose(self, pose):
        """Convert geometry_msgs/Pose to pose parameters"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw, x, y, z

    def pose2msg_deg(self, x, y, z, roll, pitch, yaw):
        """Convert pose parameters (degrees) to geometry_msgs/Pose"""
        pose = Pose()
        quat = quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        return pose

    def msg2pose_deg(self, pose):
        """Convert geometry_msgs/Pose to pose parameters (degrees)"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = np.rad2deg(euler[0])
        pitch = np.rad2deg(euler[1])
        yaw = np.rad2deg(euler[2])
        return roll, pitch, yaw, x, y, z

    def start_environment_scan(self):
        """Start environment scanning"""
        msg = Bool()
        msg.data = True
        self.env_scan_pub.publish(msg)
        self.get_logger().info("Environment scanning started")
        self.send_message("Environment scanning started")

    def stop_environment_scan(self):
        """Stop environment scanning"""
        msg = Bool()
        msg.data = False
        self.env_scan_pub.publish(msg)
        self.get_logger().info("Environment scanning stopped")
        self.send_message("Environment scanning stopped")

    def buildmap(self):
        """Build a map of the workspace by moving robot to scanning positions"""
        print("Starting workspace mapping procedure...")
        
        # Step 1: Go to home position
        back_to_home()

        # Start environment scanning
        self.start_environment_scan()
        time.sleep(1.0)

        # Step 2: Move to scanning position
        print("Moving to scanning position...")
        MoveAbsJ([180.00, -90.0, 0.0, 0.0, -60.0, 0.0], 0.1, 1)

        # Step 3: Wait for sensors to stabilize and capture data
        time.sleep(2)
        self.stop_environment_scan()
        
        # Step 5: Return to home position
        back_to_home()
        
        print("Workspace mapping completed")

def _get_perception_manager():
    """Get or create the global perception manager instance"""
    global _perception_manager
    if _perception_manager is None:
        _perception_manager = PerceptionManager()
        # Start the perception manager in a separate thread
        executor = MultiThreadedExecutor()
        executor.add_node(_perception_manager)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
    return _perception_manager

#################################### ROBOT KINEMATICS ###################################################

def MoveAbsJ(absolute_joints, speed, wait_time):
    """Move robot to absolute joint positions"""
    ACTION = Action()
    ACTION.action = "MoveJ"
    ACTION.speed = float(speed)

    INPUT = Joints()
    INPUT.joint1 = float(absolute_joints[0])
    INPUT.joint2 = float(absolute_joints[1])   
    INPUT.joint3 = float(absolute_joints[2])
    INPUT.joint4 = float(absolute_joints[3])
    INPUT.joint5 = float(absolute_joints[4])
    INPUT.joint6 = float(absolute_joints[5])
    ACTION.movej = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    if EXECUTION['Success'] == True:
        print(f"Robot moved to Joint Angular Goal: {absolute_joints}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

def MoveLinear(abs_xyz, abs_ypr, speed, wait_time):
    """Linear movement to absolute pose"""
    roll = math.radians(abs_ypr[0]) # Converts XYR to rad
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])
    
    # Quaternion from YPT in rad
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)    
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) 
    
    InputPose = Robpose()
    InputPose.x = float(abs_xyz[0])
    InputPose.y = float(abs_xyz[1])
    InputPose.z = float(abs_xyz[2])
    InputPose.qx = qx 
    InputPose.qy = qy
    InputPose.qz = qz
    InputPose.qw = qw

    EXECUTION = UR5.RobMove_EXECUTE("LIN", float(speed), InputPose)
    
    if EXECUTION['Success'] == True:
        print(f"Robot moved linearly to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")
    
def MoveJoint(abs_xyz, abs_ypr, speed, wait_time):
    """Point-to-point movement to absolute pose"""
    roll = math.radians(abs_ypr[0]) # Converts XYR to rad
    pitch = math.radians(abs_ypr[1])
    yaw = math.radians(abs_ypr[2])
    
    # Quaternion from YPT in rad
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)    
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) 
    
    InputPose = Robpose()
    InputPose.x = float(abs_xyz[0])
    InputPose.y = float(abs_xyz[1])
    InputPose.z = float(abs_xyz[2])
    InputPose.qx = qx 
    InputPose.qy = qy
    InputPose.qz = qz
    InputPose.qw = qw

    EXECUTION = UR5.RobMove_EXECUTE("PTP", float(speed), InputPose)
    
    if EXECUTION['Success'] == True:
        print(f"Robot moved Point-to-Point to Abs XYZ: {abs_xyz} and Abs YPR: {abs_ypr}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

def MoveRelLinear(relative_xyz, speed, wait_time):
    """Linear movement, relative cartesian coordinates"""
    ACTION = Action()
    ACTION.action = "MoveL"
    ACTION.speed = float(speed)

    INPUT = Xyz()
    INPUT.x = float(relative_xyz[0])
    INPUT.y = float(relative_xyz[1])
    INPUT.z = float(relative_xyz[2])
    ACTION.movel = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    if EXECUTION['Success'] == True:
        print(f"Robot moved LINEARLY by a relative increment of : {relative_xyz}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")

def MoveSingleJ(joint_number, relative_angle, speed, wait_time):
    """Relative angle in degrees, speed max 1.0, wait time after movement in seconds"""
    ACTION = Action()
    ACTION.action = "MoveR"
    ACTION.speed = float(speed)

    INPUT = Joint()
    INPUT.joint = str(joint_number)
    INPUT.value = float(relative_angle)
    ACTION.mover = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    if EXECUTION['Success'] == True:
        print(f"Robot moved {joint_number} in {relative_angle} degrees")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")
    
    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")
    
def MoveRelReor(relative_ypr, speed, wait_time):
    """Relative Reorient given relative Euler Angles"""
    ACTION = Action()
    ACTION.action = "MoveROT"
    ACTION.speed = float(speed)

    INPUT = Ypr()
    INPUT.pitch = float(relative_ypr[0])
    INPUT.yaw = float(relative_ypr[1])
    INPUT.roll = float(relative_ypr[2])
    ACTION.moverot = INPUT

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    if EXECUTION['Success'] == True:
        print(f"TCP reoriented by a relative increment of : {relative_ypr}")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s at Robot Speed: {speed*100} %")
    else: 
        print("Robot movement FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

###################################### ROBOT INFO ###################################################

def get_TCP_pose():
    """Get current TCP pose in XYZ coordinates and YPR orientation (degrees)"""
    current_pose = UR5.RobGet_POSE()
    if current_pose:
        xyz = [current_pose.x, current_pose.y, current_pose.z]
        qx, qy, qz, qw = current_pose.qx, current_pose.qy, current_pose.qz, current_pose.qw
        
        # Convert quaternion to Euler angles (YPR)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        ypr = [math.degrees(yaw), math.degrees(pitch), math.degrees(roll)]
        
        print(f"Current TCP Pose - XYZ: {xyz}, YPR: {ypr}")
        return xyz, ypr
    else:
        print("Failed to get TCP pose")
        return None, None

def get_Joint_states():
    """Get current joint states in degrees"""
    joint_states = UR5.RobGet_JOINTS()
    if joint_states:
        joints = [
            math.degrees(joint_states.joint1),
            math.degrees(joint_states.joint2),
            math.degrees(joint_states.joint3),
            math.degrees(joint_states.joint4),
            math.degrees(joint_states.joint5),
            math.degrees(joint_states.joint6)
        ]
        print(f"Current Joint States (degrees): {joints}")
        return joints
    else:
        print("Failed to get joint states")
        return None

###################################### GRIPPER ###################################################

class LinkAttacherClient(Node):
    def __init__(self):
        super().__init__('link_attacher_client')
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attach service not available, waiting again...')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detach service not available, waiting again...')
    
    def send_attach_request(self, model1_name, link1_name, model2_name, link2_name):
        request = AttachLink.Request()
        request.model1_name = model1_name
        request.link1_name = link1_name
        request.model2_name = model2_name
        request.link2_name = link2_name
        
        future = self.attach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_detach_request(self, model1_name, link1_name, model2_name, link2_name):
        request = DetachLink.Request()
        request.model1_name = model1_name
        request.link1_name = link1_name
        request.model2_name = model2_name
        request.link2_name = link2_name
        
        future = self.detach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def attach(item):
    """Attach object to gripper"""
    link_attacher_client = LinkAttacherClient()
    attach_response = link_attacher_client.send_attach_request('ur5', 'EE_robotiq_2f85', item, item)
    link_attacher_client.get_logger().info('Attach Response: %s' % attach_response.success)

def detach(): 
    """Detach all objects from gripper"""
    link_attacher_client = LinkAttacherClient()
    objects = [
        'blue_sphere', 'red_sphere', 'green_sphere', 'purple_sphere',
        'green_cylinder', 'purple_cylinder', 'red_cylinder', 'blue_cylinder'
    ]
    for obj in objects:
        link_attacher_client.send_detach_request('ur5', 'EE_robotiq_2f85', obj, obj)

def GripperSet(relative_closure, wait_time):
    """Set gripper closure percentage (100% full open, 0% full closed)"""
    ACTION = Action()
    ACTION.action = "MoveG"
    ACTION.speed = float(1) # Gripper speed not working for Robotiq 85, set to 100%

    ACTION.moveg = float(relative_closure)

    EXECUTION = UR5.Move_EXECUTE(ACTION)
    
    if EXECUTION['Success'] == True:
        print(f"Gripper set to a percentage of: {relative_closure} %")
        print(f"Movement Execution Time: {EXECUTION['ExecTime']} s")
        if relative_closure == 0:
            detach() # Automatic object dettach from gripper when full open (0%)
            
    else: 
        print("Gripper closing FAILED, check REASON in MoveIt output")

    time.sleep(wait_time)
    print(f"Waiting {wait_time} s")
    print ("")

#################################### WORKSPACE MANAGEMENT ###################################################

def set_home_position(joint_angles):
    """Set the home position for the robot"""
    global _home_joints
    if len(joint_angles) != 6:
        print("Error: Home position must have exactly 6 joint angles")
        return False
    _home_joints = joint_angles.copy()
    print(f"Home position set to: {_home_joints}")
    return True

def get_home_position():
    """Get the current home position"""
    return _home_joints.copy()

def back_to_home():
    """Move robot back to home position"""
    print("Moving robot back to home position...")
    MoveAbsJ(_home_joints, 0.9, 1.0)
    GripperSet(0, 0.5)  # 100% open (full release)
    print("Robot returned to home position")

def move_joint_arm(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    """Move robot using Forward Kinematics by specifying joint angles"""
    joint_angles = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5]
    MoveAbsJ(joint_angles, 0.5, 1.0)

def custom_scan_sequence(scan_positions):
    """Perform workspace scanning using custom joint positions"""
    print(f"Starting custom scan sequence with {len(scan_positions)} positions...")
    perception_mgr = _get_perception_manager()
    perception_mgr.send_message("Starting custom scanning sequence")
    
    all_detected_objects = {}
    
    # Start from home
    back_to_home()
    
    for i, position in enumerate(scan_positions):
        print(f"Moving to scan position {i+1}/{len(scan_positions)}")
        
        # Move to scanning position
        if len(position) == 6:
            move_joint_arm(*position)
        else:
            print(f"Warning: Invalid position {i+1}, skipping...")
            continue
        
        # Wait for stabilization
        time.sleep(1.0)
        
        # Scan from this position
        position_results = scan_workspace()
        
        # Merge results (in real implementation, you'd handle coordinate transformations)
        all_detected_objects.update(position_results)
    
    # Return to home
    back_to_home()
    
    print("Custom scan sequence completed")
    perception_mgr.send_message("Custom scanning sequence completed")
    
    return all_detected_objects

def load_objects():
    """Load objects from YAML configuration file"""
    pkg_path = get_package_share_directory("machine_vision_exercise")
    filename = os.path.join(pkg_path, "config", "models_info.yaml")

    object_list = {}
    goal_list = {}

    with open(filename, "r") as f:
        objects_info = yaml.safe_load(f)

    robot_x = objects_info["robot"]["pose"]["x"]
    robot_y = objects_info["robot"]["pose"]["y"]
    robot_z = objects_info["robot"]["pose"]["z"]

    objects = objects_info["objects"]

    for name, spec in objects.items():
        shape = spec["shape"]
        color = spec["color"]

        x = spec["pose"]["x"]
        y = spec["pose"]["y"]
        z = spec["pose"]["z"]
        roll = spec["pose"]["roll"]
        pitch = spec["pose"]["pitch"]
        yaw = spec["pose"]["yaw"]

        p = PoseStamped()
        p.header.frame_id = "world"
        p.header.stamp = Time().to_msg()

        p.pose.position.x = x - robot_x
        p.pose.position.y = y - robot_y
        p.pose.position.z = z - robot_z

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        if shape == "box":
            sx = spec["size"]["x"]
            sy = spec["size"]["y"]
            sz = spec["size"]["z"]
            p.pose.position.z += sz / 2.0
            object_list[name] = Object(p.pose, None, sz, sy, sx, shape, color)

        elif shape == "cylinder":
            height = spec["size"]["height"]
            radius = spec["size"]["radius"]
            p.pose.position.z += height / 2.0
            object_list[name] = Object(p.pose, None, height, radius * 2.0, radius * 2.0, shape, color)

        elif shape == "sphere":
            radius = spec["size"]
            p.pose.position.z += radius
            object_list[name] = Object(p.pose, None, radius * 2.0, radius * 2.0, radius * 2.0, shape, color)

        else:
            print(f"Unknown shape '{shape}' for object '{name}'")

    return object_list, goal_list

#################################### PERCEPTION INTERFACE ###################################################

def get_object_position(object_name):
    """Get object position using perception system"""
    perception_mgr = _get_perception_manager()
    return perception_mgr.get_object_position(object_name)

def get_object_info(object_name):
    """Get object information - returns height, width, length, shape, color"""
    perception_mgr = _get_perception_manager()
    return perception_mgr.get_object_info(object_name)

def scan_workspace():
    """Perform comprehensive workspace scanning"""
    perception_mgr = _get_perception_manager()
    perception_mgr.send_message("Starting workspace scan...")
    
    # Move to scanning position
    print("Moving to scanning position...")
    MoveAbsJ([180.0, -90.0, 0.0, 0.0, -60.0, 0.0], 0.1, 1)
    
    # Start environment scanning
    perception_mgr.start_environment_scan()
    time.sleep(2)
    perception_mgr.stop_environment_scan()
    
    # Return to home
    back_to_home()
    
    print("Workspace scan completed")
    return {}  # Return detected objects dictionary

def buildmap():
    """Alias for workspace mapping - calls the PerceptionManager's buildmap method"""
    perception_mgr = _get_perception_manager()
    perception_mgr.buildmap()

#################################### MAIN FUNCTION ###################################################

def main(args=None):
    """Main function for running as standalone module"""
    print("HAL system ready!")
    
    # Initialize perception manager
    perception_mgr = _get_perception_manager()
    
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Shutting down HAL system...")
    finally:
        if _perception_manager:
            _perception_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
