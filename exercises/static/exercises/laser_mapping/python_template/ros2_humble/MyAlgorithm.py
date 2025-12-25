#!/usr/bin/env python3
"""
Laser Mapping Exercise - Student Implementation Template

This file contains the template for implementing a laser-based mapping algorithm.
Students should implement their mapping solution in the execute() function.

Author: RoboticsAcademy
License: MIT
"""

import numpy as np
import cv2
import math
from HAL import setV, setW, getPose3d, getOdom, getLaserData
from WebGUI import setUserMap, poseToMap


class MyAlgorithm:
    """
    Student implementation of laser mapping algorithm.
    
    This class provides the framework for implementing SLAM (Simultaneous 
    Localization and Mapping) using laser sensor data.
    """
    
    def __init__(self):
        """Initialize the mapping algorithm."""
        # Map parameters
        self.map_width = 1500   # Map width in pixels
        self.map_height = 970   # Map height in pixels
        self.resolution = 0.05  # Map resolution (meters per pixel)
        
        # Initialize occupancy grid map
        # 0 = unknown, 127 = free space, 255 = occupied
        self.occupancy_map = np.ones((self.map_height, self.map_width), dtype=np.uint8) * 127
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Algorithm state
        self.first_iteration = True
        self.exploration_state = "forward"  # "forward", "turning", "exploring"
        self.turn_time = 0
        
        print("Laser Mapping Algorithm Initialized")
        print(f"Map size: {self.map_width}x{self.map_height}")
        print(f"Resolution: {self.resolution} m/pixel")
    
    def execute(self):
        """
        Main execution function - called repeatedly by the exercise framework.
        
        Students should implement their mapping algorithm here.
        This function is called at regular intervals and should:
        1. Get sensor data (laser, odometry)
        2. Update the map based on laser readings
        3. Control robot movement for exploration
        4. Display the updated map
        """
        try:
            # Get current robot pose and sensor data
            self.update_robot_state()
            laser_data = getLaserData()
            
            # Skip if no laser data available
            if not laser_data or len(laser_data.values) == 0:
                return
            
            # Update map with current laser scan
            self.update_map_with_laser(laser_data)
            
            # Control robot movement for exploration
            self.explore_environment(laser_data)
            
            # Display the updated map
            self.display_map()
            
        except Exception as e:
            print(f"Error in execute(): {e}")
    
    def update_robot_state(self):
        """Update robot's current position from odometry."""
        try:
            pose = getPose3d()
            if pose:
                self.robot_x = pose.x
                self.robot_y = pose.y
                self.robot_yaw = pose.yaw
        except Exception as e:
            print(f"Error getting robot pose: {e}")
    
    def update_map_with_laser(self, laser_data):
        """
        Update the occupancy grid map using laser scan data.
        
        Args:
            laser_data: Laser scan data containing ranges and angles
            
        This function should:
        1. Convert laser readings to world coordinates
        2. Mark occupied cells where obstacles are detected
        3. Mark free space between robot and obstacles
        4. Handle sensor noise and invalid readings
        """
        # TODO: Implement laser-based mapping
        # 
        # Hints:
        # - laser_data.values contains distance measurements
        # - laser_data.minAngle and maxAngle define scan range
        # - Use world_to_map() to convert coordinates
        # - Use bresenham_line() for ray tracing
        # - Filter out invalid readings (inf, nan, too close/far)
        
        ranges = laser_data.values
        min_angle = laser_data.minAngle
        max_angle = laser_data.maxAngle
        
        if len(ranges) == 0:
            return
        
        # Calculate angle increment
        angle_increment = (max_angle - min_angle) / len(ranges)
        
        # Process each laser ray
        for i, distance in enumerate(ranges):
            # Skip invalid readings
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance < 0.1 or distance > 10.0:  # Filter range
                continue
            
            # Calculate ray angle in world frame
            ray_angle = min_angle + i * angle_increment + self.robot_yaw
            
            # Calculate obstacle position in world coordinates
            obstacle_x = self.robot_x + distance * math.cos(ray_angle)
            obstacle_y = self.robot_y + distance * math.sin(ray_angle)
            
            # Convert to map coordinates
            robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
            obstacle_map_x, obstacle_map_y = self.world_to_map(obstacle_x, obstacle_y)
            
            # Mark free space along the ray (Bresenham's line algorithm)
            free_cells = self.bresenham_line(robot_map_x, robot_map_y, 
                                           obstacle_map_x, obstacle_map_y)
            
            # Mark free space
            for cell_x, cell_y in free_cells[:-1]:  # Exclude obstacle cell
                if self.is_valid_cell(cell_x, cell_y):
                    # Update probability (simple approach: set to free)
                    self.occupancy_map[cell_y, cell_x] = 200  # Free space
            
            # Mark obstacle
            if self.is_valid_cell(obstacle_map_x, obstacle_map_y):
                self.occupancy_map[obstacle_map_y, obstacle_map_x] = 50  # Occupied
    
    def explore_environment(self, laser_data):
        """
        Control robot movement for autonomous exploration.
        
        Args:
            laser_data: Current laser scan data
            
        This function implements a simple exploration strategy:
        - Move forward when path is clear
        - Turn when obstacles are detected
        - Avoid getting stuck in corners
        """
        # TODO: Implement exploration strategy
        #
        # Simple wall-following or random exploration:
        # 1. Check front sensors for obstacles
        # 2. Move forward if clear
        # 3. Turn if obstacle detected
        # 4. Add more sophisticated behavior as needed
        
        ranges = laser_data.values
        if len(ranges) == 0:
            return
        
        # Get front sensor readings (approximate)
        front_ranges = ranges[len(ranges)//3:2*len(ranges)//3]
        min_front_distance = min(front_ranges) if front_ranges else float('inf')
        
        # Simple exploration logic
        if self.exploration_state == "forward":
            if min_front_distance > 1.0:  # Path clear
                setV(0.3)  # Move forward
                setW(0.0)
            else:  # Obstacle detected
                self.exploration_state = "turning"
                self.turn_time = 0
        
        elif self.exploration_state == "turning":
            setV(0.0)  # Stop
            setW(0.5)  # Turn
            self.turn_time += 1
            
            if self.turn_time > 20:  # Turn for a while
                self.exploration_state = "forward"
    
    def world_to_map(self, world_x, world_y):
        """
        Convert world coordinates to map pixel coordinates.
        
        Args:
            world_x, world_y: Position in world frame (meters)
            
        Returns:
            map_x, map_y: Position in map frame (pixels)
        """
        # Use the coordinate transformation from WebGUI
        map_coords = poseToMap(world_x, world_y, 0)
        return int(map_coords[0]), int(map_coords[1])
    
    def is_valid_cell(self, x, y):
        """Check if map coordinates are within bounds."""
        return 0 <= x < self.map_width and 0 <= y < self.map_height
    
    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm for ray tracing.
        
        Returns list of (x, y) coordinates along the line from (x0,y0) to (x1,y1).
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def display_map(self):
        """Display the current map using the WebGUI."""
        try:
            # Convert occupancy grid to display format
            display_map = self.occupancy_map.copy()
            
            # Optional: Add robot position marker
            robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
            if self.is_valid_cell(robot_map_x, robot_map_y):
                # Draw robot as a small circle
                cv2.circle(display_map, (robot_map_x, robot_map_y), 5, 0, -1)
            
            # Send map to GUI
            setUserMap(display_map)
            
        except Exception as e:
            print(f"Error displaying map: {e}")


# Global algorithm instance
algorithm = MyAlgorithm()


def execute():
    """Main execution function called by the exercise framework."""
    algorithm.execute()


# Additional utility functions for students

def get_laser_reading_at_angle(laser_data, target_angle):
    """
    Get laser reading at a specific angle.
    
    Args:
        laser_data: Laser scan data
        target_angle: Desired angle in radians
        
    Returns:
        Distance reading at the specified angle
    """
    if not laser_data or len(laser_data.values) == 0:
        return float('inf')
    
    ranges = laser_data.values
    min_angle = laser_data.minAngle
    max_angle = laser_data.maxAngle
    
    # Normalize target angle
    while target_angle > math.pi:
        target_angle -= 2 * math.pi
    while target_angle < -math.pi:
        target_angle += 2 * math.pi
    
    # Check if angle is within scan range
    if target_angle < min_angle or target_angle > max_angle:
        return float('inf')
    
    # Find closest index
    angle_increment = (max_angle - min_angle) / len(ranges)
    index = int((target_angle - min_angle) / angle_increment)
    index = max(0, min(index, len(ranges) - 1))
    
    return ranges[index]


def filter_laser_data(laser_data, min_range=0.1, max_range=10.0):
    """
    Filter laser data to remove invalid readings.
    
    Args:
        laser_data: Raw laser scan data
        min_range, max_range: Valid range limits
        
    Returns:
        Filtered laser data
    """
    if not laser_data:
        return laser_data
    
    filtered_ranges = []
    for distance in laser_data.values:
        if math.isnan(distance) or math.isinf(distance):
            filtered_ranges.append(max_range)  # Replace with max range
        elif distance < min_range:
            filtered_ranges.append(min_range)
        elif distance > max_range:
            filtered_ranges.append(max_range)
        else:
            filtered_ranges.append(distance)
    
    laser_data.values = filtered_ranges
    return laser_data


# Example usage and testing functions

def test_coordinate_conversion():
    """Test coordinate conversion functions."""
    print("Testing coordinate conversions...")
    
    # Test some known coordinates
    test_points = [(0, 0), (1, 1), (-1, -1), (5, 3)]
    
    for world_x, world_y in test_points:
        map_x, map_y = algorithm.world_to_map(world_x, world_y)
        print(f"World ({world_x}, {world_y}) -> Map ({map_x}, {map_y})")


def print_laser_info(laser_data):
    """Print laser data information for debugging."""
    if not laser_data:
        print("No laser data available")
        return
    
    ranges = laser_data.values
    print(f"Laser data: {len(ranges)} readings")
    print(f"Range: {laser_data.minAngle:.2f} to {laser_data.maxAngle:.2f} rad")
    print(f"Min distance: {min(ranges):.2f}m")
    print(f"Max distance: {max(ranges):.2f}m")
    
    # Print front, left, right readings
    if len(ranges) > 0:
        front_idx = len(ranges) // 2
        left_idx = len(ranges) // 4
        right_idx = 3 * len(ranges) // 4
        
        print(f"Front: {ranges[front_idx]:.2f}m")
        print(f"Left: {ranges[left_idx]:.2f}m") 
        print(f"Right: {ranges[right_idx]:.2f}m")


# Uncomment for testing
# if __name__ == "__main__":
#     test_coordinate_conversion()