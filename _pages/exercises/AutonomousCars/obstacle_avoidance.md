---
permalink: /exercises/AutonomousCars/obstacle_avoidance
title: "Local navigation with VFF"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visual Follow Line"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance.png
    image_path: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance.png
    alt: "Obstacle Avoidance"
  - url: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance_teaser_gallery.png
    image_path: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance_teaser_gallery.png
    alt: "F1 laser"
  - url: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance_interface.png
    image_path: /assets/images/exercises/obstacle_avoidance/obstacle_avoidance_interface.png
    alt: "Interface"

gifs:
  - url: /assets/images/exercises/obstacle_avoidance/with_rotation.gif
    image_path: /assets/images/exercises/obstacle_avoidance/with_rotation.gif
    alt: "examples"
    title: "On applying the rotation matrix"
  - url: /assets/images/exercises/obstacle_avoidance/without_rotation.gif
    image_path: /assets/images/exercises/obstacle_avoidance/without_rotation.gif
    alt: "examples"
    title: "Without applying the rotation matrix"

gifs2:
  - url: /assets/images/exercises/obstacle_avoidance/oscillations.gif
    image_path: /assets/images/exercises/obstacle_avoidance/oscillations.gif
    alt: "examples"
    title: "Oscillation Problem in Narrow Corridors"

youtubeId1: 5SVkvfKPi_s
youtubeId2: wVJJ9ndY2aY
---

## Objective

The objective of this practice is to implement the logic of the VFF navigation algorithm.

Navigation using VFF (Virtual Force Field), consists of:

* Each object in the environment generates a repulsive force towards the robot.

* Destiny generates an attractive force in the robot.

This makes it possible for the robot to go towards the target, distancing itself of the obstacles, so that their address is the vector sum of all the forces.

{% include gallery caption="Gallery" %}

The solution can integrate one or more of the following levels of difficulty, as well as any other one that occurs to you:

* Go as quickly as possible

* Choose the safest way

* Obstacles in movement

* Robustness in situations of indecision (zero vector sum)

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Robot API

* `import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate)
* `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate)
* `HAL.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `HAL.getLaserData()` - to obtain laser sensor data
  It is composed of 180 pairs of values: (0-180º distance in millimeters)
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity
* `GUI.getNextTarget()` - to obtain the next target object on the scenario.
* `GUI.setTargetx` - sets the x coordinate of the target on the GUI.
* `GUI.setTargety` - sets the y coordinate of the target on the GUI.
* `GUI.showForces` - shows the forces being appliend on the car in real time.

To access the target 'x' and 'y' coordinates use (target is the object obtained from GUI.getNextTarget):
* `target.getPose().x` - to obtain the x position of the target
* `target.getPose().y` - to obtain the y position of the target

**Own API**

To simplify, the implementation of control points is offered.
To use it, only two actions must be carried out:
1. Obtain the following point:

   `currentTarget = GUI.getNextTarget()`
2. Mark it as visited when necessary:

   `currentTarget.setReached(True)`
   
**Debugging**

The graphical interface (GUI) allows to visualize each of the vectors of calculated forces. There is a function for this purpose:
```python
# Car direction  (green line in the image below)
carForce = [2.0, 0.0]
# Obstacles direction (red line in the image below)
obsForce = [0.0, 2.0]
# Average direction (black line in the image below)
avgForce = [-2.0, 0.0]

GUI.showForces(carForce, obsForce, avgForce)
```
As well as the destination that we have assigned:
```python
# Current target
target = [1.0, 1.0]
GUI.showLocalTarget(target)

```

![VFF_FORCES]({{ site.url }}/RoboticsAcademy/assets/images/exercises/obstacle_avoidance/vff_forces.png)


Alternatively, the following variables can be setted with the same results:
```python
# Car direction
GUI.map.carx = 0.0
GUI.map.cary = 0.0

# Obstacles direction
GUI.map.obsx = 0.0
GUI.map.obsy = 0.0

# Average direction
GUI.map.avgx = 0.0
GUI.map.avgy = 0.0

# Current target
GUI.map.targetx = 0.0
GUI.map.targety = 0.0
```
<!---
**API**

* `pose3d.getPose3d().x` - to get the position of the robot (x coordinate)
* `pose3d.getPose3d().y` - to obtain the position of the robot (y coordinate)
* `pose3d.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `laser.getLaserData()` - to obtain laser sensor data
  It is composed of 180 pairs of values: (0-180º distance in millimeters)
* `setV()` - to set and send the linear speed
* `setW()` - to set and send the angular velocity

**Own API**

To simplify, the implementation of control points is offered.
To use it, only two actions must be carried out:
1. Obtain the following point:
   `self.currentTarget = self.getNextTarget()`
2. Mark it as visited when necessary:
   `self.currentTarget.setReached(True)` --->


### Conversion of types

**Laser**

The following function parses laser data taking in to accoun 1) laser only has 180º coverage and 2) the measure red at 90º corresponds to the 'front' of the robot.

You must apply the conversions needed to transform that laser data to a vector in the relavite coodinate system of the robot.

```python

def parse_laser_data (laser_data):
    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1
    return laser
```


**Coordinate system**

We have 2 different coordinate systems in this exercise. 

* **Absolute coordinate system**: Its origin (0,0) is located in the finish line of the circuit (exactly where the F1 starts the lap). 
* **Relative coordinate system**: It is the coordinate system solidary to the robot (F1). Positive values of X means 'forward', and positive values of Y means 'left'.

You can use the following code to convert absolute coordinates to relative ones (solidary to the F1).

```python
def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel and y_rel
```


<!---### Debugging
The graphical interface (GUI) allows to visualize each of the vectors of
calculated forces. For this purpose, the following variables should be given 
value:
```python
# Car direction
self.carx = 0.0
self.cary = 0.0

# Obstacles direction
self.obsx = 0.0
self.obsy = 0.0

# Average direction
self.avgx = 0.0
self.avgy = 0.0
```

As well as the destination that we have assigned:
```python
# Current target
self.targetx = 0.0
self.targety = 0.0
```
-->
## Theory
This exercise requires us to implement a local navigation algorithm called Virtual Force Field Algorithm. Following is the complete theory regarding this algorithm.

### Navigation
Robot Navigation involves all the related tasks and algorithms required to take a robot from point A to point B **autonomously** without making any collisions. It is a very well studied topic in Mobile Robotics, comprising volumes of books! The problem of Navigation is broken down into the following subproblems:

* **Localisation**: The robot needs to know where it is.
* **Collision Avoidance**: The robot needs to detect and avoid obstacles
* **Mapping**: The robot needs to remember its surroundings
* **Planning**: The robot needs to be able to plan a route to point B
* **Explore**: The robot needs to be able to explore new terrain

![Robot Navigation Problems]({{ site.url }}/RoboticsAcademy/assets/images/exercises/obstacle_avoidance/robot-navigation-problems.png)

Some of the ways to achieve the task of Navigation are as follows:

* **Vision Based**: Computer Vision algorithms and optical sensors, like LIDAR sensors are used for Vision Based Navigation.
* **Inertial Navigation**: Airborne robots use [inertial sensors](https://en.wikipedia.org/wiki/Inertial_measurement_unit) for Navigation
* **Acoustic Navigation**: Underwater robots use SONAR based Navigation Systems
* **Radio Navigation**: Navigation used RADAR technology.


The problem of Path Planning in Navigation is dealt in 2 ways, which are Global Navigation and Local Navigation

### Global Navigation
Global Navigation involves the use of a map of the environment to plan a path from point A to point B. The optimality of the path is decided based on the length of the path, the time taken to reach the target, using permanent roads etc. Global Positioning System(GPS) is one such example of Global Navigation. The algorithms used behind such systems may include [Dijkstra](https://www.youtube.com/watch?v=GazC3A4OQTE), Best First or [A*](https://www.youtube.com/watch?v=ySN5Wnu88nE) etc.


### Local Navigation
Once the global path is decided, it is broken down into suitable waypoints. The robot navigates through these waypoints in order to reach it's destination. Local Navigation involves a dynamically changing path plan taking into consideration the changing surroundings and the vehicle constraints. Some examples of such algorithms would be Virtual Force Field, [Follow Wall](https://link.springer.com/chapter/10.1007/978-3-319-62533-1_7), [Pledge Algorithm](https://link.springer.com/chapter/10.1007/978-3-319-62533-1_7) etc.



### Virtual Force Field Algorithm
The Virtual Force Field Algorithm works in the following steps:

* The robot assigns an attractive vector to the waypoint that points towards the waypoint.
* The robot assigns a repulsive vector to the obstacle according to its sensor readings that points away from the waypoint. This is done by summing all the vectors that are translated from the sensor readings.
* The robot follows the vector obtained by summing the target and obstacle vector.

![VFF]({{ site.url }}/RoboticsAcademy/assets/images/exercises/obstacle_avoidance/vff.png)



### Drawbacks
There are a few problems related to this algorithm:

* The robot tends to oscillate in narrow corridors, that is when the robot receives an obstacle vector simultaneously from opposite sides.
* The robot may not be able to enter narrow corridors in the first place!

![Drawbacks]({{ site.url }}/RoboticsAcademy/assets/images/exercises/obstacle_avoidance/drawbacks.png)

### Virtual Force Histogram Algorithm
This algorithm improves over the Virtual Force Field Algorithm, by using a data structure called the Polar Histogram. The robot maintains a histogram grid of the  instantaneous sensor values received. Then, based on the threshold value set by the programmer, the program detects minimas(valleys) in the polar histogram. The angle corresponding to these values are then followed by the robot.

![VFH]({{ site.url }}/RoboticsAcademy/assets/images/exercises/obstacle_avoidance/vfh.gif)

**Note**: The exercise only requires us to implement Virtual Force Field Algorithm
	
## Hints
Simple hints provided to help you solve the local_navigation exercise. Please note that the **full solution has not been provided.**

### Determining the Vectors
First of all, we need to generate the 3 required vectors, that are the **Target Vector**, **Obstacle Vector** and the **Direction Vector**.

### Target Vector
The target vector can be easily obtained by subtracting the position of the car from the position of the next waypoint.

In order to implement this on the GUI interface of the exercise, in addition to the vector obtained by subtracting, we need to apply a rotation to the vector as well. The purpose of rotation is to keep the target vector always in the direction of the waypoint and not in front of the car. You may try seeing this in your own implementation, or refer to the [illustrations](#Illustrations) 

Refer to this [webpage](https://en.wikipedia.org/wiki/Rotation_matrix#In_two_dimensions) to know about the exact mathematical details of the implementation.

### Obstacle Vector
The obstacle vector is to be calculated from the sensor readings we obtain from the surroundings of the robot. Each obstacle in front of the car, is going to provide a repulsive vector, which we will add to obtain the resultant repulsive vector. Assign a repulsive vector, for each of the 180 sensor readings. The magnitude of the repulsive vector is inversely proportional to the magnitude of the sensor reading. Once, all the repulsive vectors are obtained they are all added, to get the resultant.

**Note**: There is a catch in this case, you may notice that most of the time in our implementation of the exercise we get an obstacle vector which is almost always pointing opposite to the direction in which we are supposed to head. This is a problem, as adding this vector directly to the target vector, would give a resultant which is more or less, quite not we expect. Hence, there is some trick we need to apply to solve this problem.

![Obstacle Vector]({{ site.url }}/RoboticsAcademy/assets/images/exercises/obstacle_avoidance/obstacle_vector.png)
*Obstacle Vector without any obstacles in front of the car*

### Direction Vector
Conventionally and according to the research paper of the VFF algorithm, in order to obtain the direction vector we should add the target vector and the obstacle vector. But, due to an inherent problem behind the calculation of the obstacle vector, we cannot simply add them to obtain the resultant.

A simple observation reveals that we are required to **keep moving forward** for the purpose of this exercise. Hence, the component of the direction vector in the direction of motion of the car, has no effect on the motion of the car. Therefore, we can simply leave it as a **constant**, while adjusting the vector responsible for the steering of the car. It is the steering, that will in fact, provide us with the obstacle avoidance capabilities. Hence, the steering is going to be controlled by the Direction Vector.

Also, please note that this is **not the only solution** to this problem. We may also add an offset vector in the direction of motion of the car to cancel the effect of the redundant component.

### Illustrations

{% include gallery id="gifs" caption="On applying the rotation matrix (left) - Without applying the rotation matrix (right)" %}

{% include gallery id="gifs2" caption="Oscillation Problem in Narrow Corridors" %}

## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

*This solution is an illustration for the Web Templates*

## Contributors

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Eduardo Perdices](eperdices@gsyc.es), [Francisco Pérez](https://github.com/fqez), Victor Arribas, [Julio Vega](julio.vega@urjc.es), [Jose María Cañas](https://gsyc.urjc.es/jmplaza/), [Nacho Arranz](https://github.com/igarag).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna).

## References
1. [http://www-personal.umich.edu/~johannb/vff&vfh.htm](http://www-personal.umich.edu/~johannb/vff&vfh.htm)
2. [https://en.wikibooks.org/wiki/Robotics/Navigation](https://en.wikibooks.org/wiki/Robotics/Navigation)
3. [https://en.wikipedia.org/wiki/Robot_navigation](https://en.wikipedia.org/wiki/Robot_navigation)
4. [https://www.hindawi.com/journals/jat/2018/6392697/](https://www.hindawi.com/journals/jat/2018/6392697/)
5. [https://link.springer.com/chapter/10.1007/978-3-319-62533-1_7](https://link.springer.com/chapter/10.1007/978-3-319-62533-1_7)
6. [https://www.researchgate.net/figure/The-virtual-force-field-VFF-concept-Occupied-cells-exert-repulsive-forces-onto-the_fig1_224749557](https://www.researchgate.net/figure/The-virtual-force-field-VFF-concept-Occupied-cells-exert-repulsive-forces-onto-the_fig1_224749557)
7. [http://www.cs.cmu.edu/~./motionplanning/papers/sbp_papers/integrated1/borenstein_cluttered_obstacle_avoid.pdf](http://www.cs.cmu.edu/~./motionplanning/papers/sbp_papers/integrated1/borenstein_cluttered_obstacle_avoid.pdf)
8. [https://www.mathsisfun.com/algebra/vectors.html](https://www.mathsisfun.com/algebra/vectors.html)
9. [https://matthew-brett.github.io/teaching/rotation_2d.html](https://matthew-brett.github.io/teaching/rotation_2d.html)
 
