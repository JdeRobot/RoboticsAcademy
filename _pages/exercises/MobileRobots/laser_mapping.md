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

Occupancy_grid:
  - url: /assets/images/exercises/laser_mapping/occupancy_grid.png
    image_path: /assets/images/exercises/laser_mapping/occupancy_grid.png
    alt: "Occupancy Grid"
    title: "Occupancy Grid"

youtubeId1: obHhJ-_Y96c
youtubeId2: 8pDsqMVAsv0
---

## Goal

The goal of this exercise is to develop a navigation algorithm that allows a robot to autonomously explore a warehouse environment while generating an accurate 2D map of the surroundings using LIDAR sensor data.

The robot must be able to:
* Perceive obstacles and free space using laser measurements
* Build and update a map of the environment while moving
* Navigate safely without prior knowledge of the environment
* Continue exploration until the reachable area is sufficiently covered

{% include gallery caption="Laser Mapping." %}

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-native implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

## Robot API

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
* `HAL.setW()` - to set the angular velocity.
* `HAL.setV()` - to set the linear velocity.
* `HAL.getLaserData()` - to get the data of the LIDAR. Which consists of 360 values.
* `WebGUI.poseToMap(x, y, yaw)` - converts a gazebo world coordinate system position to a map pixel.
* `WebGUI.setUserMap(map)` - shows the user built map on the user interface. It represents the values of the field that have been assigned to the array passed as a parameter. Accepts as input a two-dimensional uint8 numpy array whose values can range from 0 to 255 (grayscale). The array must be 970 pixels high and 1500 pixels wide.

### ROS 2-direct Implementation

#### ROS 2 Topics

Use standard ROS 2 topics for direct communication with the simulation.
> ⚠️ Even when using ROS 2-direct, you must still import `WebGUI` if you want visualization.

- `/turtlebot3/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`

- `/turtlebot3/odom` - Subscribe to this topic to receive the robot ground-truth odometry. Message type: `nav_msgs/msg/Odometry`

- `/turtlebot3/odom_noisy` - Subscribe to this topic to receive the noisy odometry. Message type: `nav_msgs/msg/Odometry`

- `/turtlebot3/laser/scan` - Subscribe to this topic to receive laser data. Message type: `sensor_msgs/msg/LaserScan`

- `/webgui/user_map` - Publish to this topic to display the generated occupancy map in the WebGUI. Message type: `sensor_msgs/msg/Image`  
  QoS: `TRANSIENT_LOCAL`, depth `1` The map sent to `/webgui/user_map` must be a `mono8` image with **1500 pixels width** and **970 pixels height**.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

## Theory
Laser mapping is the process by which a robot builds a representation of an unknown environment using distance measurements obtained from a LIDAR sensor while moving through the space.
The robot continuously senses its surroundings, updates the map, and decides where to move next in order to reduce unexplored areas.

### Mapping with known possitions
The robot uses its current position and orientation to correctly place sensor measurements into a global map. Each laser beam is described by a distance and an angle relative to the robot, which are converted into Cartesian coordinates. These points are then projected onto a grid map representing the environment. By repeating this process over time, the robot consistently marks free space and obstacles in the correct locations, gradually building an accurate map of its surroundings.

## Analyzing Coverage Algorithms
Mapping with known positions assumes that the current position of the robot is known. This technique consists of converting the distance measurements of the different laser beams into Cartesian coordinates relative to the robot. The distance mesured by the beam can reflect the existence of an obstacle; therefore, these Cartesian coordinates are inserted reflecting obstacles in an occupation grid relative to the current position of the robot.
This technique is not entirely real because in most cases, the position of the robot is unknown. Therefore, other techniques such as SLAM are used in real life.

### Occupancy grid
An occupation grid is a discretization of the robot's environment in cells. This discretization will be given by the size of the world in which the robot is located. Each cell can represent free space, an obstacle, or an unexplored area. Instead of using fixed binary values, the belief of a cell is updated probabilistically using laser sensor measurements. When a laser beam passes through a cell, the confidence that the cell is free increases. When a laser beam ends at a cell, the confidence that the cell is occupied increases.

To combine observations over time in a stable way, the grid is updated using a log-odds representation. In this representation, probabilities are stored in a logarithmic form, allowing evidence from multiple measurements to be added incrementally. Positive log-odds values indicate a higher confidence of an obstacle, negative values indicate free space, and values close to zero correspond to unknown areas.

### Laser Ray Interpretation and Measurement Independence
A single laser measurement provides information about both free space and obstacles. The space between the robot and the detected point is considered free, while the point where the laser ends represents an obstacle. Only the region up to the detected obstacle is interpreted, so areas behind walls are not incorrectly marked.

For probabilistic mapping to remain consistent, laser measurements should be treated as independent observations. Since repeated scans taken without sufficient movement are highly correlated, updates are considered meaningful only after the robot has moved or changed its orientation. This helps prevent overconfidence and improves the overall quality of the map.

### Exploration and Coverage
Exploration defines the path that the robot follows to observe and map unknown areas of the environment. Different exploration algorithms generate different robot paths.
* Random Exploration-The robot moves along irregular paths, changing direction when obstacles are encountered.
* Wall-Following Exploration-The robot follows paths that remain close to walls or obstacles, allowing it to trace boundaries and gradually reveal the structure of the environment.
* Systematic Coverage (Lawn-Mower Motion)-The robot follows parallel back-and-forth paths that sweep the environment in an organized manner, ensuring uniform and predictable coverage.
* Frontier-Based Exploration-The robot’s path is directed toward the boundary between explored and unexplored regions, causing the mapped area to expand outward until no unknown space remains.

Any exploration algorithm that guides the robot safely through the environment and progressively reduces unknown areas is considered valid for this exercise.

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