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
---

## Goal

The objective of this practice is to implement the logic of a navigation algorithm for a vacuum using laser mapping. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

{% include gallery caption="Laser Mapping." %}

## Robot API

* `import HAL` - to import the HAL library class. This class contains the functions that receives information from the sensors or to work with the actuators.
* `import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getPose3d().x` - to get position x of the robot
* `HAL.getPose3d().y` - to get position y of the robot
* `HAL.getPose3d().yaw` - to get the orientation of the robot
* `HAL.getOdom().x` - to get the approximated X coordinate of the robot (with noise)
* `HAL.getOdom().y` - to get the approximated Y coordinate of the robot (with noise)
* `HAL.getOdom().yaw` - to get the approximated orientation position of the robot (with noise)
* `HAL.getOdom2().x` - to get the approximated X coordinate of the robot (with more noise than getOdom)
* `HAL.getOdom2().y` - to get the approximated Y coordinate of the robot (with more noise than getOdom)
* `HAL.getOdom2().yaw` - to get the approximated orientation position of the robot (with more noise than getOdom)
* `HAL.getOdom3().x` - to get the approximated X coordinate of the robot (with even more noise than getOdom)
* `HAL.getOdom3().y` - to get the approximated Y coordinate of the robot (with even more noise than getOdom)
* `HAL.getOdom3().yaw` - to get the approximated orientation position of the robot (with even more noise than getOdom)
* `HAL.motors.sendW()` - to set the angular velocity
* `HAL.motors.sendV()` - to set the linear velocity
* `HAL.getLaserData()` - to get the data of the LIDAR. Which consists of 360 values
* `GUI.poseToMap(x, y, yaw)` - converts a gazebo world coordinate system position to a map pixel.
* `GUI.setUserMap(map)` - shows the user built map on the user interface. It represents the values of the field that have been assigned to the array passed as a parameter. Accepts as input a two-dimensional uint8 numpy array whose values can range from 0 to 255 (grayscale). The array must be 970 pixels high and 1500 pixels wide.

## Theory
Implementation of laser mapping for a vacuum is the basic requirement for this exercise. First, lets see how mapping with known possitions works.

### Mapping with known possitions
Coverage Path Planning is an important area of research in Path Planning for robotics, which involves finding a path that passes through every reachable position in its environment. In this exercise, We are using a very basic coverage algorithm called Random Exploration.

## Analyzing Coverage Algorithms
Mapping with known positions assumes that the current position of the robot is known. This technique consists of converting the distance measurements of the different laser beams into Cartesian coordinates relative to the robot. The distance of the beams reflects the existence of an obstacle; therefore, these Cartesian coordinates are inserted reflecting obstacles in an occupation grid with respect to the current position of the robot.
This technique is not entirely real because in most cases, the position of the robot is unknown. Therefore, other techniques such as SLAM are used.

### Occupancy grid
An occupation grid is a discretization of the robot's environment in cells. This discretization will be given by the size of the world in which the robot is located. With an occupation grid, a matrix is handled whose cells will contain a probability value, which indicates the certainty that in that position there is an obstacle (1), there is free space (0), or it has not been explored for the moment (gray space).
The occupation grids were initially proposed in 1985 by Moravec and Elfes. The biggest advantage of these types of maps is that they are easy to build and maintain, even in large environments. Also, it is easy for a robot to determine its position within the map just by knowing its position and orientation, since the geometry of the cells corresponds to the geometry of the terrain.
On the other hand, the basic problem with this type of map is the large amount of memory required for storing the information.

### Illustrations

{% include gallery id="Occupancy_grid" caption="An example of a map obtained with the Mapping technique with known positions." %}

## Videos

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
