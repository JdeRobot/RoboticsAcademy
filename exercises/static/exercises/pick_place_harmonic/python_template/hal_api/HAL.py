#!/usr/bin/env python3
"""
HAL (Hardware Abstraction Layer) API for Pick and Place Harmonic Exercise
Provides a simple interface for students to control the UR5 + Robotiq gripper
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
import math


class HAL:
    """Hardware Abstraction Layer for UR5 + Robotiq Gripper"""
    
    def __init__(self):
        """Initialize HAL interface"""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('hal_node')
        
        # Action Clients
        self.move_group_client = ActionClient(self.node, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Wait for servers
        self.node.get_logger().info('HAL: Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()
        self.node.get_logger().info('HAL: Waiting for Gripper action server...')
        self.gripper_client.wait_for_server()
        self.node.get_logger().info('HAL: Ready!')
        
        # Standard orientations
        self.DOWN_ORIENTATION = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)  # Gripper pointing down
        
    def move_to_pose(self, x, y, z, orientation=None):
        """
        Move the robot arm to a target pose
        
        Args:
            x (float): X coordinate in meters (relative to base_link)
            y (float): Y coordinate in meters
            z (float): Z coordinate in meters
            orientation (Quaternion, optional): Target orientation. Defaults to pointing down.
        
        Returns:
            bool: True if movement succeeded, False otherwise
        """
        if orientation is None:
            orientation = self.DOWN_ORIENTATION
            
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.group_name = 'ur5_manipulator'
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]  # 1cm tolerance
        bv.primitives.append(sp)
        bv.primitive_poses.append(Pose(position=Point(x=x, y=y, z=z)))
        
        pc.constraint_region = bv
        pc.weight = 1.0
        
        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'tool0'
        oc.orientation = orientation
        oc.absolute_x_axis_tolerance = 0.3
        oc.absolute_y_axis_tolerance = 0.3
        oc.absolute_z_axis_tolerance = 0.3
        oc.weight = 1.0
        
        goal_msg.request.goal_constraints.append(Constraints(
            position_constraints=[pc],
            orientation_constraints=[oc]
        ))
        
        goal_msg.planning_options.plan_only = False
        
        self.node.get_logger().info(f'HAL: Moving to [{x:.2f}, {y:.2f}, {z:.2f}]')
        
        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        
        while not send_goal_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().error('HAL: Goal rejected')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        result = result_future.result()
        
        if result and result.result.error_code.val == 1:
            self.node.get_logger().info('HAL: Movement successful')
            return True
        else:
            self.node.get_logger().error(f'HAL: Movement failed')
            return False
    
    def set_gripper(self, position):
        """
        Set gripper position
        
        Args:
            position (float): Gripper opening in meters (0.0 to 0.085)
                             0.0 = fully closed
                             0.085 = fully open
        
        Returns:
            bool: True if gripper movement succeeded
        """
        # Convert meters to joint position
        # 0.0m (closed) = 0.8 rad
        # 0.085m (open) = 0.0 rad
        if position < 0.0:
            position = 0.0
        if position > 0.085:
            position = 0.085
            
        joint_position = 0.8 * (1.0 - (position / 0.085))
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [joint_position]
        point.time_from_start.sec = 2
        
        goal.trajectory.points.append(point)
        
        self.node.get_logger().info(f'HAL: Setting gripper to {position*1000:.1f}mm ({joint_position:.2f} rad)')
        
        send_goal_future = self.gripper_client.send_goal_async(goal)
        
        while not send_goal_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().error('HAL: Gripper goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return True
    
    def open_gripper(self):
        """Open gripper fully"""
        return self.set_gripper(0.085)
    
    def close_gripper(self):
        """Close gripper fully"""
        return self.set_gripper(0.0)
    
    def grasp(self, width):
        """
        Grasp an object of given width
        
        Args:
            width (float): Object width in meters
        
        Returns:
            bool: True if gripper closed to grasp position
        """
        # Add squeeze factor for firm grip
        grasp_width = max(0.0, width - 0.01)  # Close 1cm more than object width
        return self.set_gripper(grasp_width)
    
    def sleep(self, seconds):
        """Sleep for given seconds while keeping ROS alive"""
        time.sleep(seconds)


# Global HAL instance (created by Academy)
_hal_instance = None

def get_hal():
    """Get or create HAL instance"""
    global _hal_instance
    if _hal_instance is None:
        _hal_instance = HAL()
    return _hal_instance
