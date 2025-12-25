# Laser Mapping Exercise - API Reference

This document provides detailed information about the APIs available in the Laser Mapping exercise.

## Table of Contents
- [HAL (Hardware Abstraction Layer)](#hal-hardware-abstraction-layer)
- [WebGUI (Graphical User Interface)](#webgui-graphical-user-interface)
- [Data Structures](#data-structures)
- [Coordinate Systems](#coordinate-systems)
- [Examples](#examples)

## HAL (Hardware Abstraction Layer)

The HAL module provides access to robot sensors and actuators.

### Motor Control

#### `setV(velocity)`
Set the robot's linear velocity.

**Parameters:**
- `velocity` (float): Linear velocity in m/s
  - Positive values: move forward
  - Negative values: move backward
  - Range: typically -1.0 to 1.0 m/s

**Example:**
```python
from HAL import setV
setV(0.3)  # Move forward at 0.3 m/s
setV(-0.2) # Move backward at 0.2 m/s
setV(0.0)  # Stop
```

#### `setW(angular_velocity)`
Set the robot's angular velocity.

**Parameters:**
- `angular_velocity` (float): Angular velocity in rad/s
  - Positive values: turn left (counterclockwise)
  - Negative values: turn right (clockwise)
  - Range: typically -2.0 to 2.0 rad/s

**Example:**
```python
from HAL import setW
setW(0.5)  # Turn left
setW(-0.5) # Turn right
setW(0.0)  # Stop turning
```

### Sensor Data

#### `getPose3d()`
Get the robot's true position (ground truth from simulation).

**Returns:**
- `Pose3D` object with attributes:
  - `x` (float): X position in meters
  - `y` (float): Y position in meters
  - `z` (float): Z position in meters (usually 0 for ground robots)
  - `yaw` (float): Orientation in radians
  - `pitch` (float): Pitch angle in radians
  - `roll` (float): Roll angle in radians

**Example:**
```python
from HAL import getPose3d
pose = getPose3d()
print(f"Robot position: ({pose.x:.2f}, {pose.y:.2f})")
print(f"Robot orientation: {pose.yaw:.2f} rad")
```

#### `getOdom()`
Get noisy odometry data (simulates real-world sensor noise).

**Returns:**
- `Pose3D` object (same structure as `getPose3d()`)
- Contains accumulated drift and noise typical of wheel encoders

**Example:**
```python
from HAL import getOdom
odom_pose = getOdom()
true_pose = getPose3d()

# Compare noisy vs true position
print(f"Odometry: ({odom_pose.x:.2f}, {odom_pose.y:.2f})")
print(f"True pose: ({true_pose.x:.2f}, {true_pose.y:.2f})")
```

#### `getOdom2()` and `getOdom3()`
Get odometry data with different noise levels.

**Returns:**
- `Pose3D` object with varying amounts of noise
- `getOdom2()`: Medium noise level
- `getOdom3()`: High noise level

#### `getLaserData()`
Get laser scanner readings.

**Returns:**
- `LaserData` object with attributes:
  - `values` (list): Array of distance measurements in meters
  - `minAngle` (float): Minimum scan angle in radians
  - `maxAngle` (float): Maximum scan angle in radians
  - `maxRange` (float): Maximum detection range in meters

**Example:**
```python
from HAL import getLaserData
import math

laser = getLaserData()
ranges = laser.values

# Check for obstacles
if len(ranges) > 0:
    min_distance = min(ranges)
    print(f"Closest obstacle: {min_distance:.2f}m")
    
    # Get front reading
    front_idx = len(ranges) // 2
    front_distance = ranges[front_idx]
    print(f"Front distance: {front_distance:.2f}m")
```

## WebGUI (Graphical User Interface)

The WebGUI module provides functions for displaying maps and visualizations.

#### `setUserMap(image)`
Display a map image in the web interface.

**Parameters:**
- `image` (numpy.ndarray): Map image as a 2D numpy array
  - Shape: (970, 1500) - height x width in pixels
  - Data type: uint8
  - Values: 0-255 (0=black/occupied, 255=white/free, 127=unknown)

**Example:**
```python
import numpy as np
from WebGUI import setUserMap

# Create a simple map
map_height, map_width = 970, 1500
my_map = np.ones((map_height, map_width), dtype=np.uint8) * 127  # Unknown

# Mark some occupied cells
my_map[100:200, 100:200] = 0  # Black square (occupied)
my_map[300:400, 300:400] = 255  # White square (free)

# Display the map
setUserMap(my_map)
```

#### `poseToMap(x, y, yaw)`
Convert world coordinates to map pixel coordinates.

**Parameters:**
- `x` (float): X position in world frame (meters)
- `y` (float): Y position in world frame (meters)  
- `yaw` (float): Orientation in radians (not used in conversion)

**Returns:**
- `list`: [map_x, map_y, transformed_yaw]
  - `map_x` (int): X coordinate in map pixels
  - `map_y` (int): Y coordinate in map pixels

**Example:**
```python
from WebGUI import poseToMap

# Convert robot position to map coordinates
world_x, world_y = 2.5, -1.0
map_coords = poseToMap(world_x, world_y, 0)
map_x, map_y = int(map_coords[0]), int(map_coords[1])

print(f"World ({world_x}, {world_y}) -> Map ({map_x}, {map_y})")
```

## Data Structures

### Pose3D
Represents a 3D pose (position + orientation).

```python
class Pose3D:
    x: float      # X position (meters)
    y: float      # Y position (meters)  
    z: float      # Z position (meters)
    yaw: float    # Yaw angle (radians)
    pitch: float  # Pitch angle (radians)
    roll: float   # Roll angle (radians)
```

### LaserData
Represents laser scanner data.

```python
class LaserData:
    values: list[float]  # Distance measurements (meters)
    minAngle: float      # Minimum scan angle (radians)
    maxAngle: float      # Maximum scan angle (radians)
    maxRange: float      # Maximum detection range (meters)
```

## Coordinate Systems

### World Frame
- Origin: Simulation world origin
- X-axis: Points forward (robot's initial direction)
- Y-axis: Points left
- Units: Meters
- Angles: Radians (0 = facing +X, positive = counterclockwise)

### Map Frame  
- Origin: Top-left corner of map image
- X-axis: Points right (image columns)
- Y-axis: Points down (image rows)
- Units: Pixels
- Size: 1500 x 970 pixels (width x height)

### Robot Frame
- Origin: Robot center
- X-axis: Points forward
- Y-axis: Points left
- Laser scanner mounted at robot center

## Examples

### Basic Movement
```python
from HAL import setV, setW, getLaserData
import time

def move_forward_until_obstacle():
    while True:
        laser = getLaserData()
        if len(laser.values) > 0:
            front_distance = laser.values[len(laser.values)//2]
            
            if front_distance > 1.0:  # Safe distance
                setV(0.3)  # Move forward
                setW(0.0)
            else:  # Obstacle detected
                setV(0.0)  # Stop
                setW(0.5)  # Turn left
        
        time.sleep(0.1)
```

### Simple Mapping
```python
import numpy as np
import math
from HAL import getPose3d, getLaserData
from WebGUI import setUserMap, poseToMap

def create_simple_map():
    # Initialize map
    map_height, map_width = 970, 1500
    occupancy_map = np.ones((map_height, map_width), dtype=np.uint8) * 127
    
    # Get current data
    pose = getPose3d()
    laser = getLaserData()
    
    if not laser or len(laser.values) == 0:
        return
    
    # Process laser data
    ranges = laser.values
    angle_increment = (laser.maxAngle - laser.minAngle) / len(ranges)
    
    for i, distance in enumerate(ranges):
        if math.isnan(distance) or distance > 10.0:
            continue
            
        # Calculate obstacle position
        angle = laser.minAngle + i * angle_increment + pose.yaw
        obs_x = pose.x + distance * math.cos(angle)
        obs_y = pose.y + distance * math.sin(angle)
        
        # Convert to map coordinates
        map_coords = poseToMap(obs_x, obs_y, 0)
        map_x, map_y = int(map_coords[0]), int(map_coords[1])
        
        # Mark obstacle
        if 0 <= map_x < map_width and 0 <= map_y < map_height:
            occupancy_map[map_y, map_x] = 0  # Occupied
    
    # Display map
    setUserMap(occupancy_map)
```

### Wall Following
```python
from HAL import setV, setW, getLaserData

def wall_follow():
    laser = getLaserData()
    if not laser or len(laser.values) == 0:
        return
    
    ranges = laser.values
    n_readings = len(ranges)
    
    # Get sensor readings at different angles
    front = ranges[n_readings // 2]           # 0 degrees
    right = ranges[3 * n_readings // 4]       # -90 degrees  
    left = ranges[n_readings // 4]            # +90 degrees
    
    # Wall following logic
    target_distance = 0.8  # Desired distance from wall
    
    if front < 1.0:  # Obstacle ahead
        setV(0.0)
        setW(0.5)    # Turn left
    elif right > target_distance + 0.2:  # Too far from right wall
        setV(0.3)
        setW(-0.2)   # Turn slightly right
    elif right < target_distance - 0.2:  # Too close to right wall
        setV(0.3)
        setW(0.2)    # Turn slightly left
    else:  # Good distance from wall
        setV(0.3)
        setW(0.0)    # Go straight
```

## Error Handling

### Common Issues
1. **Empty laser data**: Always check if `laser.values` is not empty
2. **Invalid coordinates**: Verify map coordinates are within bounds
3. **NaN/Inf values**: Filter laser readings before processing
4. **Map dimensions**: Ensure map is exactly 970x1500 pixels

### Best Practices
```python
# Safe laser data access
laser = getLaserData()
if laser and len(laser.values) > 0:
    ranges = laser.values
    # Process data...

# Safe map coordinate conversion
map_coords = poseToMap(x, y, 0)
map_x, map_y = int(map_coords[0]), int(map_coords[1])
if 0 <= map_x < 1500 and 0 <= map_y < 970:
    # Update map...

# Filter invalid laser readings
def filter_laser(ranges):
    return [r if not (math.isnan(r) or math.isinf(r)) else 10.0 
            for r in ranges]
```