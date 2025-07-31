---
permalink: /exercises/MobileRobots/vacuum_cleaner
title: " Basic Vacuum Cleaner"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Vacuum Cleaner"
toc_icon: "cog"


gallery:
    image_path: /assets/images/exercises/vacuum_cleaner/vacuum_cleaner.png
    alt: "Vacuum"

youtubeId1: c90hmfkZRNY
youtubeId2: Xcy84DhVjrY
youtubeId3: qwBQ1B-05xU
---

## Goal

The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

<img src="/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/vacuum_cleaner.png" width="100%" height="60%">
{% include gallery caption="Vacuum cleaner." %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is done, how to launch a RoboticsBackend and how to access the exercises.

## Robot API

This exercise now supports ROS 2-native implementation in addition to the original HAL-based approach. Below you'll find the details for both options.
### HAL-based Implementation

* `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
* `import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getBumperData().state` - to establish if the robot has crashed or not. Returns 1 if the robot collides and 0 if it has not crashed.
* `HAL.getBumperData().bumper` - if the robot has crashed, it returns 1 when the crash occurs on center of the robot, 0 when it occurs on its right and 2 if the collision is on its left.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate).
* `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate).
* `HAL.getPose3d().yaw` - to get the orientation of the robot regarding the map.
* `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in meters).
* `HAL.setV()` - to set the linear speed.
* `HAL.setW()` - to set the angular velocity.

```python
print ('Execute')

# TODO

print(HAL.getPose3d().x)
print(HAL.getPose3d().y)
# Vacuum's yaw
yaw = HAL.getPose3d().yaw
```

For this example, it is necessary to ensure that the vacuum cleaner covers the highest possible percentage of the house. The application of the automatic evaluator (referee) will measure the percentage traveled, and based on this percentage, will perform the qualification of the solution algorithm.

### Types conversion

- **Laser**

```python
import math
import numpy as np

def parse_laser_data(laser_data):
    """ Parses the LaserData object and returns a tuple with two lists:
        1. List of  polar coordinates, with (distance, angle) tuples,
           where the angle is zero at the front of the robot and increases to the left.
        2. List of cartesian (x, y) coordinates, following the ref. system noted below.

        Note: The list of laser values MUST NOT BE EMPTY.
    """
    laser_polar = []  # Laser data in polar coordinates (dist, angle)
    laser_xy = []  # Laser data in cartesian coordinates (x, y)
    for i in range(180):
        # i contains the index of the laser ray, which starts at the robot's right
        # The laser has a resolution of 1 ray / degree
        #
        #                (i=90)
        #                 ^
        #                 |x
        #             y   |
        # (i=180)    <----R      (i=0)

        # Extract the distance at index i
        dist = laser_data.values[i]
        # The final angle is centered (zeroed) at the front of the robot.
        angle = math.radians(i - 90)
        laser_polar += [(dist, angle)]
        # Compute x, y coordinates from distance and angle
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        laser_xy += [(x, y)]
    return laser_polar, laser_xy

# Usage
laser_data = HAL.getLaserData()
if len(laser_data.values) > 0:
    laser_polar, laser_xy = parse_laser_data(laser_data)
```

### ROS 2-native Implementation
#### ROS 2 Topics
Use standard ROS 2 topics for direct communication with the simulation.
* `/cmd_vel`  - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`
* `/odom` - Subscribe to this topic to get the robot's position and orientation. Message type: `nav_msgs/msg/Odometry`
* `/roombaROS/laser/scan` - Subscribe to this topic to get laser scan data. Message type: `sensor_msgs/msg/LaserScan`
* `/roombaROS/events/center_bumper` - Subscribe to this topic to detect collisions at the center of the robot. Message type: `gazebo_msgs/msg/ContactsState`
* `/roombaROS/events/left_bumper` - Subscribe to this topic to detect collisions at the left side of the robot. Message type: `gazebo_msgs/msg/ContactsState`
* `/roombaROS/events/right_bumper` - Subscribe to this topic to detect collisions at the right side of the robot. Message type: `gazebo_msgs/msg/ContactsState`
#### Frequency Control
Use standard ROS 2 mechanisms to manage loop timing:
* `rclpy.spin()` - Event-driven execution using callbacks.
* `rclpy.spin_once()` - Single-step processing, often with custom timers.
* `rclpy.Rate()` - Loop-based frequency control.


## Theory

Implementation of navigation algorithms for an autonomous vacuum is the basic requirement for this exercise. The main objective is to cover the largest area of a house. First, let us understand what are Coverage Algorithms.

### Coverage Algorithms

Coverage Path Planning is an important area of research in Path Planning for robotics, which involves finding a path that passes through every reachable position in its environment. In this exercise, We are using a very basic coverage algorithm called Random Exploration.

## Analyzing Coverage Algorithms

### Classification
Coverage algorithms are divided into two categories.

- **Offline coverage**
Uses fixed information and the environment is known in advance. Genetic Algorithms, Neural Networks, Cellular Decomposition, Spanning Trees are some examples to name a few.

- **Online Coverage**

Uses real-time measurements and decisions to cover the entire area. The Sensor-based approach is included in this category.

### Base Movement

The problem of coverage involves two standard basic motions, which are used as a base for other complex coverage algorithms.

- **Spiral Motion**

The robot follows an increasing circle/square pattern.

![Base Movement Spiral]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/spiral.gif)

- **Boustrophedon Motion**

The robot follows an S-shaped pattern.

![Base Movement Boustrophedon]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/boustrophedon.gif)

### Analysis of Coverage Algorithms

Any coverage algorithm is analyzed using the given criterion.

- **Environment Decomposition**
This involves dividing the area into smaller parts.

- **Sweep Direction**

This influences the optimality of generated paths for each sub-region by adjusting the duration, speed, and direction of each sweep.

- **Optimal Backtracking**

This involves the plan to move from one small subregion to another. The coverage is said to be complete when there is no point left to backtrack.

### Supplements

Usually, coverage algorithms generate a linear, piecewise path composed of straight lines and sharp turns. This path is difficult to follow for other autonomous drones like Underwater Vehicles, Aerial Vehicles and some Ground Vehicles. Path Smoothening is applied to these paths to effectively implement the algorithm.

## Hints

Simple hints provided to help you solve the vacuum_cleaner exercise. Please note that the **full solution has not been provided.**

### Random Angle Generation

The most important task is the generation of a random angle. There are 2 ways to achieve it.

- **Random Duration**: By keeping the angular_velocity fixed, the duration of the turn can be randomized, in order to point the robot towards a random direction.

- **Random Angle**: This method requires calculation. We generate a random angle and then turn towards it. Approximately an angular speed of 3 turns the robot by 90 degrees.

Among both methods, Random Duration would be preferable as the Random Angle requires precision, which requires PID to be achieved successfully.

Also, in order to achieve better precision it is preferable to use ```rospy.sleep()``` in place of ```time.sleep()```.

### Dash Movement

Once the direction has been decided, we move in that direction. This is the simplest part, we have to send a velocity changing command to the robot, and wait until a collision is detected.

A word of caution though, whenever we have a change of state, we have to give a sleep duration to the robot to give it time to reset the commands given to it. [Illustrations](#Illustrations) section describes a visual representation.

### Spiral Movement

Using the physical formula $v = r·\omega$ (See [references](#References) for more details). In order to increase $r$, we can either increase $v$ or decrease $\omega$, while keeping the other parameter constant. Experimentally, increasing $v$ has a better effect than decreasing $\omega$. Refer to [illustrations](#Illustrations).

### Analysis

Being such a simple algorithm, it is not expected to work all the time. The maximum accuracy we got was 80% and that too only once!

### Illustrations

![]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/without_duration.gif) 

*Without applying a sleep duration the previous rotation command still has effect on the go straight command*

![]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/duration.gif)

*After applying a duration, we get straight direction movement*

![]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/reduce_omega.gif)

*Effect of reducing $\omega$ to generate spiral*

![]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/increasing_v.gif)

*Effect of increasing $v$ to generate spiral*

### Demonstrative video of the solution
 
{% include youtubePlayer.html id=page.youtubeId3 %}

## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

*This solution is an illustration for the Web Templates*

<br/>

## Contributors

- Contributors: [Vanessa Fernandez](https://github.com/vmartinezf), [Jose María Cañas](https://github.com/jmplaza), [Carlos Awadallah](https://github.com/cawadall), [Nacho Arranz](https://github.com/igarag), [Javier Izquierdo](https://github.com/javizqh), [Ashish Ramesh](https://github.com/AshishRamesh).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna), [Javier Izquierdo](https://github.com/javizqh).

## References

1. [http://wiki.ros.org/Robots/Roomba](http://wiki.ros.org/Robots/Roomba)  
2. [https://docs.ros2.org/humble/api/sensor_msgs/msg/LaserScan.html](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
3. [https://docs.ros.org/en/noetic/api/gazebo_msgs/html/msg/ContactsState.html](https://docs.ros.org/en/noetic/api/gazebo_msgs/html/msg/ContactsState.html)  
4. [http://wiki.ros.org/Robots/Roomba](http://wiki.ros.org/Robots/Roomba)

<!--
Another possible solution is to implement the logic of a navigation algorithm for an autonomous vacuum with autolocation.
{% include youtubePlayer.html id=page.youtubeId2 %}
-->
