#!/usr/bin/env python3
"""
HAL Module - Hardware Abstraction Layer API
Direct function definitions for Academy compatibility
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, CollisionObject
from moveit_msgs.srv import GetCartesianPath
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
        self.execute_trajectory_client = ActionClient(self.node, ExecuteTrajectory, 'execute_trajectory')
        self.gripper_client = ActionClient(self.node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Service client for Cartesian path planning
        self.cartesian_path_client = self.node.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # Publisher for collision objects
        self.collision_pub = self.node.create_publisher(CollisionObject, '/collision_object', 10)
        
        self.node.get_logger().info('HAL: Waiting for servers...')
        self.move_group_client.wait_for_server()
        self.execute_trajectory_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.cartesian_path_client.wait_for_service()
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
    """Grasp object of given width with proper squeeze factor
    
    Uses same calculation as original demo for reliable grasping
    """
    # Clamp width to valid range
    if width > 0.085:
        width = 0.085
    if width < 0.0:
        width = 0.0
    
    # Calculate base gripper position
    # 0.0 rad = fully open (0.085m)
    # 0.8 rad = fully closed (0.0m)
    val = 0.8 * (1.0 - (width / 0.085))
    
    # Add squeeze factor for firm grip (matches original demo)
    squeeze_factor = 0.2
    val += squeeze_factor
    
    # Clamp to valid range
    if val > 0.8:
        val = 0.8
    if val < 0.0:
        val = 0.0
    
    return set_gripper(val)


def sleep(seconds):
    """Sleep while keeping ROS alive"""
    time.sleep(seconds)

def add_collision_box(name, size_x, size_y, size_z, pos_x, pos_y, pos_z):
    """Add a box-shaped collision object to the planning scene"""
    hal = _get_hal()
    
    co = CollisionObject()
    co.header.frame_id = 'base_link'
    co.id = name
    co.operation = CollisionObject.ADD
    
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [size_x, size_y, size_z]
    co.primitives.append(primitive)
    
    pose = Pose()
    pose.position = Point(x=pos_x, y=pos_y, z=pos_z)
    pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    co.primitive_poses.append(pose)
    
    hal.collision_pub.publish(co)
    hal.node.get_logger().info(f'Added collision: {name}')
    time.sleep(0.5)


def remove_collision_object(name):
    """Remove a collision object from the planning scene"""
    hal = _get_hal()
    
    co = CollisionObject()
    co.header.frame_id = 'base_link'
    co.id = name
    co.operation = CollisionObject.REMOVE
    
    hal.collision_pub.publish(co)
    hal.node.get_logger().info(f'Removed collision: {name}')
    time.sleep(0.5)


def setup_workspace():
    """Setup collision objects for workspace - prevents table/bin collisions"""
    hal = _get_hal()
    hal.node.get_logger().info('Setting up workspace...')
    
    # Table (behind robot) - WITH 90-DEGREE ROTATION
    # Quaternion(w=0.707, z=0.707) rotates box 90 degrees around Z-axis
    co = CollisionObject()
    co.header.frame_id = 'base_link'
    co.id = 'table'
    co.operation = CollisionObject.ADD
    
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.8, 1.6, 0.06]
    co.primitives.append(primitive)
    
    pose = Pose()
    pose.position = Point(x=-0.6, y=0.0, z=-0.17)
    pose.orientation = Quaternion(w=0.707, x=0.0, y=0.0, z=0.707)  # 90-deg rotation!
    co.primitive_poses.append(pose)
    
    hal.collision_pub.publish(co)
    hal.node.get_logger().info('Added collision: table')
    time.sleep(0.5)
    
    # Conveyor (in front) - WITH 90-DEGREE ROTATION
    co = CollisionObject()
    co.header.frame_id = 'base_link'
    co.id = 'conveyor'
    co.operation = CollisionObject.ADD
    
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [2.4, 0.8, 0.1]
    co.primitives.append(primitive)
    
    pose = Pose()
    pose.position = Point(x=0.6, y=0.0, z=0.05)
    pose.orientation = Quaternion(w=0.707, x=0.0, y=0.0, z=0.707)  # 90-deg rotation!
    co.primitive_poses.append(pose)
    
    hal.collision_pub.publish(co)
    hal.node.get_logger().info('Added collision: conveyor')
    time.sleep(0.5)
    
    # Bins - NO ROTATION (identity quaternion)
    # Bins have walls 0.24-0.39m high in world file
    # Using 0.35m height (average) and 0.3x0.3 footprint
    bin_z = -0.145  # Bin center in base_link frame
    bin_height = 0.35  # Increased from 0.1m to match wall heights!
    bins = [
        ('yellow_bin', -0.4, -0.45),
        ('red_bin', -0.4, 0.15),
        ('blue_bin', -0.4, 0.45),
        ('green_bin', -0.4, -0.15)
    ]
    
    for name, x, y in bins:
        co = CollisionObject()
        co.header.frame_id = 'base_link'
        co.id = name
        co.operation = CollisionObject.ADD
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.3, 0.3, bin_height]  # Updated height!
        co.primitives.append(primitive)
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=bin_z)
        pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)  # No rotation
        co.primitive_poses.append(pose)
        
        hal.collision_pub.publish(co)
        hal.node.get_logger().info(f'Added collision: {name}')
        time.sleep(0.5)
    
    hal.node.get_logger().info('Workspace ready!')
# Cartesian path planning function - add before sleep() function

def move_cartesian(x, y, z, orientation=None):
    """
    Move in a straight line to target position using Cartesian path planning
    Use this for precision movements like lowering to pick or placing objects
    
    Args:
        x, y, z: Target position in meters
        orientation: Optional orientation (defaults to pointing down)
    
    Returns:
        bool: True if movement succeeded
    """
    hal = _get_hal()
    
    if orientation is None:
        orientation = hal.DOWN_ORIENTATION
    
    # Create service request
    req = GetCartesianPath.Request()
    req.header.frame_id = 'base_link'
    req.header.stamp = hal.node.get_clock().now().to_msg()
    req.group_name = 'ur5_manipulator'
    
    # Define target waypoint
    req.waypoints = [Pose(
        position=Point(x=x, y=y, z=z),
        orientation=orientation
    )]
    
    req.max_step = 0.01  # 1cm resolution for smooth path
    req.jump_threshold = 0.0  # Disable jump detection
    req.avoid_collisions = True
    
    hal.node.get_logger().info(f'Cartesian path to [{x:.2f}, {y:.2f}, {z:.2f}]')
    
    # Call service
    future = hal.cartesian_path_client.call_async(req)
    
    while not future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    response = future.result()
    
    if not response or response.error_code.val != 1:
        hal.node.get_logger().error(f'Cartesian path failed (error: {response.error_code.val if response else "None"})')
        return False
    
    if response.fraction < 0.9:
        hal.node.get_logger().error(f'Cartesian path incomplete ({response.fraction*100:.0f}% achieved)')
        return False
    
    # Execute trajectory
    hal.node.get_logger().info(f'Executing Cartesian path ({response.fraction*100:.0f}% planned)')
    
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = response.solution
    
    send_goal_future = hal.execute_trajectory_client.send_goal_async(goal)
    
    while not send_goal_future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    goal_handle = send_goal_future.result()
    
    if not goal_handle or not goal_handle.accepted:
        hal.node.get_logger().error('Cartesian execution rejected')
        return False
    
    result_future = goal_handle.get_result_async()
    
    while not result_future.done():
        rclpy.spin_once(hal.node, timeout_sec=0.1)
    
    result = result_future.result()
    
    if result and result.result.error_code.val == 1:
        hal.node.get_logger().info('Cartesian move successful')
        return True
    else:
        hal.node.get_logger().error('Cartesian execution failed')
        return False
