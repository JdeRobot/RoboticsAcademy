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
    alt: "Laser Mapping"

warehouse:
  - url: /assets/images/exercises/laser_mapping/warehouse_birdeye.png
    image_path: /assets/images/exercises/laser_mapping/warehouse_birdeye.png
    alt: "Warehouse Bird's Eye View"
    title: "Warehouse Environment - Bird's Eye View"

Occupancy_grid:
  - url: /assets/images/exercises/laser_mapping/occupancy_grid.png
    image_path: /assets/images/exercises/laser_mapping/occupancy_grid.png
    alt: "Occupancy Grid"
    title: "Occupancy Grid"

youtubeId1: obHhJ-_Y96c
youtubeId2: 8pDsqMVAsv0
---

## Goal

Build a navigation algorithm that explores the warehouse, avoids obstacles, and produces an accurate 2D occupancy map from laser data.

{% include gallery caption="Laser Mapping." %}

### Warehouse Environment

{% include gallery id="warehouse" caption="Bird's eye view of the warehouse environment to be mapped." %}

## What you must do

- Fuse laser + pose into an occupancy map (0 = obstacle, 255 = free, 127 = unknown).
- Use an exploration policy (wall-following, random, lawn‑mower, or frontier) that reduces unknown space.
- Keep the robot collision-free while it maps.
- Publish your map to the UI with `WebGUI.setUserMap(map)` (shape must be 970×1500, type uint8, values 0–255).

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

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

## Theory

Implementation of laser mapping for a vacuum is the basic requirement for this exercise. First, let's see how mapping with known positions works.

### Mapping with known positions

Coverage Path Planning is an important area of research in Path Planning for robotics, which involves finding a path that passes through every reachable position in its environment. In this exercise, we are using a very basic coverage algorithm called Random Exploration.

### Analyzing Coverage Algorithms

Mapping with known positions assumes that the current position of the robot is known. This technique consists of converting the distance measurements of the different laser beams into Cartesian coordinates relative to the robot. The distance measured by the beam can reflect the existence of an obstacle; therefore, these Cartesian coordinates are inserted reflecting obstacles in an occupation grid relative to the current position of the robot.

This technique is not entirely real because in most cases, the position of the robot is unknown. Therefore, other techniques such as SLAM are used in real life.

### Occupancy grid

An occupation grid is a discretization of the robot's environment in cells. This discretization will be given by the size of the world in which the robot is located. With an occupation grid, a matrix is handled whose cells will contain a probability value, which indicates the certainty that in that position there is an obstacle (1), there is free space (0), or it has not been explored for the moment (gray space).

The occupation grids were initially proposed in 1985 by Moravec and Elfes. The biggest advantage of these types of maps is that they are easy to build and maintain, even in large environments. Also, it is easy for a robot to determine its position within the map just by knowing its position and orientation, since the geometry of the cells corresponds to the geometry of the terrain.

On the other hand, the basic problem with this type of map is the large amount of memory required for storing the information.

### Occupancy grid basics
- The world is discretized into cells (matrix 970×1500). Each cell is marked as free, occupied, or unknown using uint8 values (255, 0, 127).
- For each laser ray: mark cells along the beam as free, and the end of the beam as occupied (if it hits before max range).
- Repeated scans are most useful after the robot moves; avoid over-trusting multiple readings from the same pose.

### Exploration options (pick one)
- Random: irregular paths, reacts to obstacles.
- Wall-following: traces boundaries; simple but not optimal.
- Lawn‑mower (systematic coverage): parallel sweeps for uniform coverage.
- Frontier-based: drives to the boundary between known/unknown for efficient coverage.

### When you are done
- Obstacles and free space should align with the warehouse layout.
- Unknown area should shrink over time.
- Robot should avoid collisions.

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
