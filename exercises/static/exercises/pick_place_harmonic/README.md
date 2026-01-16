# Pick and Place Harmonic - Exercise Instructions

## Objective
Program the UR5 robot with Robotiq gripper to pick objects from a conveyor and place them in colored bins using MoveIt2 motion planning.

## Environment
- **Robot**: Universal Robots UR5 with Robotiq 85 2-Finger Gripper
- **Simulation**: Gazebo Harmonic
- **Planning**: MoveIt2

## Coordinate System
- **Frame**: `base_link` (robot base)
- **Units**: meters
- **Robot base height**: Z = 0.9m in world frame
- **Work area**: 
  - X: -0.8m to 0.8m
  - Y: -0.6m to 0.6m
  - Z: -0.2m to 0.6m (relative to base)

## HAL API Functions

### Movement
```python
hal.move_to_pose(x, y, z, orientation=None)
```
Move the robot end-effector to target position. Returns `True` if successful.

**Parameters:**
- `x`, `y`, `z`: Position in meters relative to robot base
- `orientation`: Optional Quaternion (defaults to pointing down)

**Example:**
```python
hal.move_to_pose(x=0.5, y=0.0, z=0.3)  # Move 50cm forward, at 30cm height
```

### Gripper Control
```python
hal.open_gripper()          # Open gripper fully (85mm)
hal.close_gripper()         # Close gripper fully
hal.set_gripper(position)   # Set gripper opening in meters (0.0 to 0.085)
hal.grasp(width)            # Grasp object of given width (meters)
```

**Example:**
```python
hal.open_gripper()          # Prepare to pick
hal.grasp(width=0.050)      # Grasp 50mm wide object
```

### Utility
```python
hal.sleep(seconds)          # Wait while keeping ROS alive
```

## Example Tasks

### Task 1: Home Position
```python
hal.move_to_pose(x=0.0, y=-0.4, z=0.4)
```

### Task 2: Pick from Conveyor
```python
# Above object
hal.move_to_pose(x=0.6, y=-0.3, z=0.3)
hal.sleep(1)

# Down to object
hal.move_to_pose(x=0.6, y=-0.3, z=0.15)
hal.sleep(1)

# Grasp
hal.grasp(width=0.055)
hal.sleep(2)

# Lift
hal.move_to_pose(x=0.6, y=-0.3, z=0.3)
```

### Task 3: Place in Bin
```python
# Move to bin  
hal.move_to_pose(x=-0.4, y=0.15, z=0.2)
hal.sleep(1)

# Release
hal.open_gripper()
hal.sleep(2)
```

## Tips
1. **Plan your path**: Move to safe positions first to avoid collisions
2. **Use intermediate waypoints**: Don't move directly from pick to place
3. **Wait for completion**: Use `hal.sleep()` after each movement
4. **Test incrementally**: Test one movement at a time
5. **Check RViz**: Visualize planned paths before execution

## Common Positions
```python
HOME = [0.0, -0.4, 0.4]          # Safe home position
CONVEYOR_ABOVE = [0.6, 0.0, 0.3] # Above conveyor
RED_BIN = [-0.4, 0.15, 0.2]      # Red bin location
BLUE_BIN = [-0.4, 0.45, 0.2]     # Blue bin location
GREEN_BIN = [-0.4, -0.15, 0.2]   # Green bin location
YELLOW_BIN = [-0.4, -0.45, 0.2]  # Yellow bin location
```

## Error Handling
If a movement fails (returns `False`), the robot cannot reach that position. Try:
- Using intermediate waypoints
- Adjusting the target position
- Checking for collision obstacles in RViz

## Challenge
Once you master basic pick and place, try:
1. Picking multiple objects in sequence
2. Sorting objects by color into different bins
3. Optimizing movement paths for speed
4. Adding perception to detect object locations

Good luck! 🤖
