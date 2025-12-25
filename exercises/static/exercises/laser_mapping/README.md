# Laser Mapping Exercise

## Overview
The Laser Mapping exercise teaches students how to build maps using laser sensor data from a mobile robot. This exercise demonstrates SLAM (Simultaneous Localization and Mapping) concepts in a simulated warehouse environment.

## Learning Objectives
- Understand laser sensor data processing
- Learn mapping algorithms and techniques
- Implement SLAM concepts
- Work with ROS2 and Gazebo simulation

## Exercise Description
In this exercise, you will program a TurtleBot3 robot to navigate through a warehouse environment while building a map using laser sensor data. The robot must:

1. **Collect Laser Data**: Use the onboard laser scanner to detect obstacles and walls
2. **Process Sensor Data**: Convert raw laser readings into useful mapping information
3. **Build Map**: Create an occupancy grid map of the environment
4. **Handle Noise**: Deal with sensor noise and uncertainty in odometry

## Environment
- **Robot**: TurtleBot3 with laser scanner
- **Simulation**: Gazebo warehouse environment
- **Sensors**: 360-degree laser scanner, wheel odometry
- **Framework**: ROS2 Humble

## Getting Started

### Prerequisites
- ROS2 Humble installed
- Gazebo simulation environment
- Basic understanding of Python programming
- Familiarity with robotics concepts

### Running the Exercise
1. Launch the exercise from the RoboticsAcademy web interface
2. Wait for the simulation environment to load
3. Implement your mapping algorithm in the provided code template
4. Test your solution in the warehouse environment

## Code Structure

### Available APIs

#### HAL (Hardware Abstraction Layer)
```python
# Robot Movement
setV(velocity)          # Set linear velocity (m/s)
setW(angular_velocity)  # Set angular velocity (rad/s)

# Sensor Data
getPose3d()            # Get robot's real position (x, y, yaw)
getOdom()              # Get noisy odometry data
getLaserData()         # Get laser scanner readings
```

#### GUI (Graphical User Interface)
```python
# Map Display
setUserMap(image)      # Display your generated map
poseToMap(x, y, yaw)   # Convert world coordinates to map coordinates
```

### Code Template
The exercise provides a Python template with:
- **HAL.py**: Hardware abstraction layer for robot control and sensors
- **WebGUI.py**: Interface for displaying maps and robot position
- **map.py**: Utility functions for coordinate transformations
- **MyAlgorithm.py**: Your implementation goes here

## Implementation Guide

### Step 1: Understanding Laser Data
```python
# Get laser readings
laser_data = getLaserData()
ranges = laser_data.values  # Array of distance measurements
min_angle = laser_data.minAngle  # Minimum scan angle
max_angle = laser_data.maxAngle  # Maximum scan angle
```

### Step 2: Basic Mapping Algorithm
1. **Initialize Map**: Create an empty occupancy grid
2. **Process Laser Data**: For each laser reading:
   - Calculate obstacle position in world coordinates
   - Mark occupied cells in the map
   - Mark free space between robot and obstacle
3. **Update Map**: Continuously update as robot moves
4. **Display Result**: Use `setUserMap()` to show your map

### Step 3: Handle Robot Movement
```python
# Example movement pattern
def explore_environment():
    # Move forward
    setV(0.3)
    setW(0.0)
    
    # Check for obstacles
    laser_data = getLaserData()
    if min(laser_data.values) < 0.5:  # Obstacle detected
        # Turn to avoid obstacle
        setV(0.0)
        setW(0.5)
```

## Tips and Best Practices

### Mapping Tips
- **Grid Resolution**: Choose appropriate cell size for your map
- **Coordinate Systems**: Understand robot, world, and map coordinate frames
- **Noise Handling**: Filter sensor noise and handle uncertainty
- **Memory Management**: Efficiently store and update map data

### Algorithm Suggestions
- **Occupancy Grid**: Most common approach for laser-based mapping
- **Bresenham's Line**: Efficient algorithm for ray tracing
- **Probabilistic Updates**: Handle sensor uncertainty properly

### Debugging
- Use `getPose3d()` to track robot's actual position
- Compare with `getOdom()` to see odometry drift
- Visualize laser rays to debug sensor processing
- Check map updates in real-time using the GUI

## Evaluation Criteria
Your solution will be evaluated based on:
- **Map Quality**: Accuracy and completeness of generated map
- **Algorithm Efficiency**: Performance and computational complexity
- **Code Quality**: Clean, well-documented implementation
- **Robustness**: Handling of edge cases and sensor noise

## Common Challenges
1. **Coordinate Transformations**: Converting between different reference frames
2. **Sensor Noise**: Dealing with imperfect laser readings
3. **Odometry Drift**: Accumulating errors in robot position
4. **Dynamic Objects**: Handling moving obstacles (if present)

## Advanced Extensions
- Implement loop closure detection
- Add particle filter for better localization
- Handle dynamic environments
- Optimize mapping algorithm for real-time performance

## Resources
- [ROS2 Navigation Stack](https://navigation.ros.org/)
- [SLAM Algorithms Overview](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
- [Occupancy Grid Mapping](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)

## Troubleshooting

### Common Issues
- **Map not displaying**: Check image dimensions (970x1500 pixels required)
- **Robot not moving**: Verify velocity commands are being sent
- **Laser data empty**: Ensure simulation is fully loaded before starting

### Getting Help
- Check the console for error messages
- Use print statements to debug your algorithm
- Verify sensor data is being received correctly

---

**Exercise Documentation Website**: [https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/laser_mapping](https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/laser_mapping)