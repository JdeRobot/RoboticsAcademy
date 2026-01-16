#!/usr/bin/env python3
"""
HAL Module - Hardware Abstraction Layer API
Direct function definitions for Academy compatibility
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


class _HALSingleton:
    """Internal HAL singleton"""
    _instance = None
    
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('hal_node')
        self.move_group_client = ActionClient(self.node, MoveGroup, 'move_action')
        self.gripper_client = ActionClient(self.node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        self.node.get_logger().info('HAL: Waiting for servers...')
        self.move_group_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.node.get_logger().info('HAL: Ready!')
        
        self.DOWN_ORIENTATION = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


def _get_hal():
    """Get or create HAL singleton"""
    if _HALSingleton._instance is None:
        _HALSingleton._instance = _HALSingleton()
    return _HALSingleton._instance


def move_to_pose(x, y, z, orientation=None):
    """Move robot end-effector to target position"""
    hal = _get_hal()
    
    if orientation is None:
        orientation = hal.DOWN_ORIENTATION
    
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
    sp.dimensions = [0.01]
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
    
    hal.node.get_logger().info(f'Moving to [{x:.2f}, {y:.2f}, {z:.2f}]')
    
    send_goal_future = hal.move_group_client.send_goal_async(goal_msg)
    
    while not send_goal_future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    goal_handle = send_goal_future.result()
    
    if not goal_handle or not goal_handle.accepted:
        hal.node.get_logger().error('Goal rejected')
        return False
    
    result_future = goal_handle.get_result_async()
    
    while not result_future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    result = result_future.result()
    
    if result and result.result.error_code.val == 1:
        hal.node.get_logger().info('Movement successful')
        return True
    else:
        hal.node.get_logger().error('Movement failed')
        return False


def set_gripper(position):
    """Set gripper opening (0.0 to 0.085 meters)"""
    hal = _get_hal()
    
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
    
    hal.node.get_logger().info(f'Setting gripper to {position*1000:.1f}mm')
    
    send_goal_future = hal.gripper_client.send_goal_async(goal)
    
    while not send_goal_future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    goal_handle = send_goal_future.result()
    
    if not goal_handle or not goal_handle.accepted:
        hal.node.get_logger().error('Gripper goal rejected')
        return False
    
    result_future = goal_handle.get_result_async()
    
    while not result_future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    return True


def open_gripper():
    """Open gripper fully"""
    return set_gripper(0.085)


def close_gripper():
    """Close gripper fully"""
    return set_gripper(0.0)


def grasp(width):
    """Grasp object of given width with squeeze factor"""
    grasp_width = max(0.0, width - 0.01)
    return set_gripper(grasp_width)


def sleep(seconds):
    """Sleep while keeping ROS alive"""
    time.sleep(seconds)
