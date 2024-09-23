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
---

## Goal

The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

<img src="/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/vacuum_cleaner.png" width="100%" height="60%">
{% include gallery caption="Vacuum cleaner." %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Robot API

* `import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getBumperData().state` - To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* `HAL.getBumperData().bumper` - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its right and 2 if the collision is at its left.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate)
* `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate)
* `HAL.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity

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
laser_data = HAL.getLaserData()

def parse_laser_data(laser_data):
    laser = []
    for i in range(180):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser
```

```python
def laser_vector(laser):
    laser_vectorized = []
    for d,a in laser:
        # (4.2.1) laser into GUI reference system
        x = d * math.cos(a) * -1
        y = d * math.sin(a) * -1
        v = (x,y)
        laser_vectorized += [v]

    laser_mean = np.mean(laser_vectorized, axis=0)
    return laser_mean
```

## Theory

Implementation of navigation algorithms for an autonomous vacuum is the basic requirement for this exercise. The main objective is to cover the largest area of a house. First, let us understand what is Coverage Algorithms.

### Coverage Algorithms

Coverage Path Planning is an important area of research in Path Planning for robotics, which involves finding a path that passes through every reachable position in its environment. In this exercise, We are using a very basic coverage algorithm called Random Exploration.

## Analyzing Coverage Algorithms

### Classification
Coverage algorithms are divided into two categories.

- **Offline coverage**
use fixed information and the environment is known in advance. Genetic Algorithms, Neural Networks, Cellular Decomposition, Spanning Trees are some examples to name a few.

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

Usually, coverage algorithms generate a linear, piecewise path composed of straight lines and sharp turns. This path is difficult for other autonomous drones like Underwater Vehicles, Aerial Vehicles and some Ground Vehicles difficult to follow. Path Smoothening is applied to these paths to effectively implement the algorithm.

## Hints

Simple hints provided to help you solve the vacuum_cleaner exercise. Please note that the **full solution has not been provided.**

### Random Angle Generation

The most important task is the generation of a random angle. There are 2 ways to achieve it.

- **Random Duration**: By keeping the angular_velocity fixed, the duration of the turn can be randomized, in order to point the robot towards a random direction.

- **Random Angle**: This method requires calculation. We generate a random angle and then turn towards it. Approximately an angular speed of 3 turns the robot by 90 degrees.

Among both the methods, Random Duration would be a preferable one as the Random Angle requires precision, which requires PID to be achieved successfully.

Also, in order to achieve better precision it is preferable to use ```rospy.sleep()``` in place of ```time.sleep()```.

### Dash Movement

Once the direction has been decided, we move in that direction. This is the simplest part, we have to send velocity command to the robot, until a collision is detected.

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


## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

*This solution is an illustration for the Web Templates*

<br/>

## Contributors

- Contributors: [Vanessa Fernandez](https://github.com/vmartinezf), [Jose María Cañas](https://github.com/jmplaza), [Carlos Awadallah](https://github.com/cawadall), [Nacho Arranz](https://github.com/igarag).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna).

<!--
Another possible solution is to implement the logic of a navigation algorithm for an autonomous vacuum with autolocation.
{% include youtubePlayer.html id=page.youtubeId2 %}
-->
