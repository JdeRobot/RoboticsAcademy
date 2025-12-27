---
permalink: /exercises/MobileRobots/laser_mapping
title: "Laser Mapping"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Laser Mapping"
toc_icon: "cog"


gallery:
  - url: /assets/images/exercises/laser_mapping/laser_mapping_teaser.png
    image_path: /assets/images/exercises/laser_mapping/laser_mapping_teaser.png
    alt: "Vacuum"
lidar:
  - url: /assets/images/exercises/laser_mapping/laser_mapping_lidar.png
    image_path: /assets/images/exercises/laser_mapping/laser_mapping_lidar.png
    alt: "lidar"
    title: "lidar"

Occupancy_grid:
  - url: /assets/images/exercises/laser_mapping/occupancy_grid.png
    image_path: /assets/images/exercises/laser_mapping/occupancy_grid.png
    alt: "Occupancy Grid"
    title: "Occupancy Grid"

youtubeId1: obHhJ-_Y96c
youtubeId2: 8pDsqMVAsv0
---

## Goal

The goal of this exercise is to implement a laser-based mapping system that allows a mobile robot to autonomously build a map of its environment under realistic conditions.
{% include gallery caption="Laser Mapping." %}

This exercise introduces real-world challenges such as sensor noise, motion uncertainty, and partial observability, forming the basis for SLAM-oriented navigation systems.

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API
These APIs provide access to the robot’s sensors, actuators, and visualization tools required to implement laser-based mapping.
* `import HAL` - to import the HAL library class. This class contains the functions that receive information from the sensors or work with the actuators.
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getPose3d().x` - to get position x of the robot.
* `HAL.getPose3d().y` - to get position y of the robot.
* `HAL.getPose3d().yaw` - to get the orientation of the robot.
* `HAL.getOdom().x` - to get the approximated X coordinate of the robot (with noise).
* `HAL.getOdom().y` - to get the approximated Y coordinate of the robot (with noise).
* `HAL.getOdom().yaw` - to get the approximated orientation position of the robot (with noise).
* `HAL.getOdom2().x` - to get the approximated X coordinate of the robot (with more noise than getOdom).
* `HAL.getOdom2().y` - to get the approximated Y coordinate of the robot (with more noise than getOdom).
* `HAL.getOdom2().yaw` - to get the approximated orientation position of the robot (with more noise than getOdom).
* `HAL.getOdom3().x` - to get the approximated X coordinate of the robot (with even more noise than getOdom).
* `HAL.getOdom3().y` - to get the approximated Y coordinate of the robot (with even more noise than getOdom).
* `HAL.getOdom3().yaw` - to get the approximated orientation position of the robot (with even more noise than getOdom).
* `HAL.setW(angular_velocity)` - to set the angular velocity.
* `HAL.setV(velocity)` - to set the linear velocity.
* `HAL.getLaserData()` - to get the data of the LIDAR. Which consists of 360 values.
* `WebGUI.poseToMap(x, y, yaw)` - converts a gazebo world coordinate system position to a map pixel.
* `WebGUI.setUserMap(map)` - shows the user built map on the user interface. It represents the values of the field that have been assigned to the array passed as a parameter. Accepts as input a two-dimensional uint8 numpy array whose values can range from 0 to 255 (grayscale). The array must be 970 pixels high and 1500 pixels wide.

### Example of use
```python
def execute():
    # 1. Get the robot's pose (Start with Pose3d for perfect mapping)
    pose = HAL.getPose3d()
    
    # 2. Get Laser scans
    laser = HAL.getLaserData()
    
    # 3. Convert robot position to map pixels for Ray Tracing start point
    rx, ry = WebGUI.poseToMap(pose.x, pose.y, 0)
    
    # 4. Loop through laser.values
    for i, dist in enumerate(laser.values):
        # Calculate obstacle world (x,y)
        # Convert obstacle to map pixel (mx, my)
        # Use Bresenham from (rx, ry) to (mx, my)
        pass

    # 5. Push map to UI
    WebGUI.setUserMap(my_generated_occupancy_grid)
```
## Theory
Implementing laser-based mapping for a mobile robot is the fundamental requirement of this exercise. We begin by understanding how mapping works when the robot’s position is known.
### Introduction to Laser Mapping
Laser mapping is a perception technique that allows a mobile robot to build a representation of its environment using distance measurements obtained from a laser range sensor (LiDAR). By continuously observing the surroundings while moving, the robot incrementally constructs a map that describes the location of obstacles and free space.

## Laser Sensor Model
A 2D LiDAR sensor emits laser beams uniformly around the robot and measures the distance to the first obstacle encountered by each beam.

Each laser measurement is expressed in polar coordinates:
* Distance:`ri​`
* Angle: `θi`

A complete scan typically covers 360 degrees, producing hundreds of distance measurements per iteration.
{% include gallery id="lidar" caption="Geometric Interpretation of 2D LiDAR Measurements." %}

### Coordinate Transformation
 To place laser measurements onto a global map, each laser reading must be transformed from the robot frame into the world frame.

The robot pose in the world frame is represented as **(x_r, y_r, ψ)**, where **x_r** and **y_r** denote the position of the robot and **ψ** represents its orientation (yaw angle).

Each laser sensor measurement is given in the robot frame as **(r_i, θ_i)**, where **r_i** is the distance to the detected obstacle and **θ_i** is the angle of the laser beam relative to the robot’s forward direction.

To compute the position of the detected obstacle in the world frame, the laser measurement is first rotated by the robot’s orientation and then translated by the robot’s position. The resulting world coordinates of the obstacle are calculated as:

* `x_i = x_r + r_i cos(θ_i + ψ)`
* `y_i = y_r + r_i sin(θ_i + ψ)`

This coordinate transformation enables the robot to determine the global positions of obstacles detected by the laser sensor, which is essential for mapping and navigation tasks.

## Occupancy Grid Mapping

The environment is represented using an **occupancy grid**, which divides the workspace into a two-dimensional grid of discrete cells. Each cell stores the state of the environment at that location.

The possible cell states are:

- **Occupied** → An obstacle is detected in the cell  
- **Free** → The cell represents empty space  
- **Unknown** → The cell has not yet been observed  

Formally, an occupancy grid is defined as a mapping:

`m : Z² → {occupied, free, unknown}`

This means that each integer grid coordinate in the 2D space is assigned a state indicating whether it is occupied, free, or unknown.

Occupancy grids are widely used in mobile robotics because they are simple to update using sensor data, scale well to large environments, and integrate naturally with mapping, localization, and path-planning algorithms.

## Free Space and Ray Tracing

A laser beam provides information not only about the location of obstacles but also about the free space between the robot and the detected obstacle. This information is essential for building an accurate and navigable map.

When a laser beam is emitted, it travels through empty space until it reaches the first obstacle. As a result, every position along the beam’s path before the obstacle can be considered free space. The final point where the beam stops corresponds to an occupied cell, indicating the presence of an obstacle.

For each individual laser beam, the occupancy grid is updated according to the following rules:

- All grid cells intersected by the beam between the robot’s position and the obstacle are marked as **free**  
- The grid cell corresponding to the obstacle location is marked as **occupied**

To accurately identify which grid cells are intersected by a laser beam, a **ray tracing** algorithm is required. Ray tracing converts a continuous line segment in space into a discrete set of grid cells that approximate the path of the beam.

A widely used and computationally efficient approach for this purpose is **Bresenham’s Line Algorithm**. This algorithm incrementally determines the grid cells that best approximate a straight line between two points using only integer arithmetic, making it well suited for real-time robotic mapping applications.

By repeatedly applying ray tracing for all laser beams over multiple iterations, the robot progressively refines its understanding of the environment. Regions observed multiple times become more certain, while unexplored areas remain marked as unknown.

This process ensures that the resulting occupancy grid map accurately represents both obstacles and navigable free space, enabling safe navigation and effective path planning.

## Exploration, Uncertainty, and Relation to SLAM

Mapping an environment requires the robot to actively explore its surroundings. If the robot remains stationary or follows an inefficient exploration strategy, large portions of the environment will remain unknown, resulting in an incomplete map. In this exercise, exploration is typically achieved through simple reactive behaviors, such as moving forward when free space is detected and turning when obstacles are encountered. Even these basic behaviors clearly demonstrate that robot motion and map quality are tightly coupled.

Real robotic systems operate under uncertainty caused by sensor noise, wheel slippage, and odometry drift. As the robot moves, small pose estimation errors accumulate over time and become visible as distortions in the generated map. The estimated robot pose can be expressed as:

`x̂ₜ = xₜ + εₜ`

where xₜ represents the true robot pose and εₜ represents accumulated noise. Since laser measurements are projected into the map using this estimated pose, localization errors directly affect mapping accuracy. This leads to a fundamental insight:

**Accurate mapping cannot be achieved without accurate localization.**

This observation motivates the need for SLAM (Simultaneous Localization and Mapping), in which both the robot’s pose and the map are estimated jointly instead of assuming perfect position information.

---

## SLAM (Simultaneous Localization and Mapping)

SLAM addresses the problem of building a map while simultaneously estimating the robot’s pose using sensor measurements and control inputs. In a full SLAM formulation, the objective is to estimate the joint probability distribution:

`p(x₁:ₜ, m | z₁:ₜ, u₁:ₜ)`

where x₁:ₜ are the robot poses over time, m is the map, z₁:ₜ are sensor measurements, and u₁:ₜ are control inputs.

Conceptually, a SLAM system alternates between two key steps:

- **Prediction**, where the robot estimates its new pose using a motion model and control inputs  
- **Correction**, where sensor measurements are used to reduce uncertainty by comparing expected and actual observations  

By repeatedly alternating between the prediction and correction steps, SLAM gradually reduces uncertainty in the robot’s pose while simultaneously refining the map of the environment.
### Illustrations

{% include gallery id="Occupancy_grid" caption="An example of a map obtained with the Mapping technique with known positions." %}

## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

*This is a demostrative solution on unibotics.org*

{% include youtubePlayer.html id=page.youtubeId1 %}

*This solution is an illustration for the Web Templates*

<br/>


## Contributors

- Contributors: [Vladislav](https://github.com/vladkrav), [Jose María Cañas](https://github.com/jmplaza), [Nacho Arranz](https://github.com/igarag).
- Maintained by [Juan Manuel Carretero](https://github.com/JuanManuelCarretero).

<!--
Another possible solution is to implement the logic of a navigation algorithm for an autonomous vacuum with autolocation.
{% include youtubePlayer.html id=page.youtubeId2 %}
-->
