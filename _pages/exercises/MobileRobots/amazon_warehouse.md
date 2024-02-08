---
permalink: /exercises/MobileRobots/amazon_warehouse/
title: "Amazon Warehouse"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Amazon Warehouse"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_teaser.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_teaser.png
    alt: "Shelves to move"
    title: "Shelves to move"
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
    alt: "Robot"
    title: "Robot"

warehouse1:
  - url: /assets/images/exercises/amazon_warehouse/warehouse1.png
    image_path: /assets/images/exercises/amazon_warehouse/warehouse1.png
    alt: "Warehouse 1"
    title: "Warehouse 1"

warehouse2:
  - url: /assets/images/exercises/amazon_warehouse/warehouse2.png
    image_path: /assets/images/exercises/amazon_warehouse/warehouse2.png
    alt: "Warehouse 2"
    title: "Warehouse 2"

ompl:
  - url: /assets/images/exercises/amazon_warehouse/OMPL_structure.png
    image_path: /assets/images/exercises/amazon_warehouse/OMPL_structure.png
    alt: "OMPL structure"
    title: "OMPL structure"

example:
  - url: /assets/images/exercises/amazon_warehouse/example_plot.png
    image_path: /assets/images/exercises/amazon_warehouse/example_plot.png
    alt: "Example"
    title: "Example"

holonomicrobot:
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
    alt: "Robot"
    title: "Robot"

pashe1: EVt9vYqEoDg
ackermannRobot: NCC9bn-v_Ro
robotgeometry: FPPF27QIRHw

---

## Goal

The objective of this practice is to implement the logic that allows a holonomic logistics robot to deliver shelves to the required place by making use of the location of the robot. The robot is equipped with a map and knows its current location in it. The main objective will be to find the shortest path to complete the task.

{% include gallery caption="Gallery" %}


## Instructions
This is the preferred way for running the exercise.

### Installing and Launching
1. Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

2. Pull the current distribution of Robotics Academy Docker Image:

	```bash
  docker pull jderobot/robotics-academy:latest
  ```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

### How to perform the exercises?
- Start a new docker container of the image and keep it running in the background:

	```bash
  docker run --rm -it -p 7164:7164 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-academy
  ```

- On the local machine navigate to 127.0.0.1:7164/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

- The exercise can be used after the alert.

### Enable GPU Acceleration
- Follow the advanced launching instructions from [here](https://jderobot.github.io/RoboticsAcademy/user_guide/#enable-gpu-acceleration).

**Where to insert the code?**

In the launched webpage, type your code in the text editor,

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!


while True:
    # Enter iterative code!
```

### Using the Interface

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation(primarily, the position of the robot).

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Debugging Console**: This shows the error messages related to the student’s code that is sent. The student can also use it to visualize the output of the `print()` function.

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware (Gazebo).
* `from GUI import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getPose3d()` - returns x,y and theta components of the robot in world coordinates
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular speed
* `HAL.lift()` - to lift the platform
* `HAL.putdown()` - to put down the platform
* `GUI.showPath(array)` - shows a path on the map. The parameter should be a 2D array containing each of the points of the path
* `GUI.getMap(url)` - returns a numpy array with the image data in a 3 dimensional array (R, G, B, A). The URL of the Amazon Warehouse World 1 is '/RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png'. The URL of the Amazon Warehouse World 2 is '/RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map_2.png'.

## Supporting information

There are two **warehouses** to choose from:
#### Warehouse 1:
* The warehouse size is 20.62 meters long and 13.6 meters wide.
* The shelves coordinates from 1 to 6 are: (3.728, 0.579), (3.728, -1.242), (3.728, -3.039), (3.728, -4.827), (3.728, -6.781), (3.728, -8.665).
* You can get the warehouse's map from there: /RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png

{% include gallery id="warehouse1" caption="Warehouse 1" %}


#### Warehouse 2: 
* The warehouse size is 34 meters long and 22 meters wide.
* The shelves coordinates from 1 to 9 are: (-8.5, -6.0), (-1.5, -6.0), (5.5, -6.0), (-8.5, 4.0), (-1.5, 4.0), (5.5, 4.0), (-1.0, -15.5), (-2, 11.5), (1.0, -15.0). 
  * The separation distance between two neighboring shelves is 2 m on the x axis and 1.5 m on the y axis.
* You can get the warehouse's map from there: /RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map_2.png

{% include gallery id="warehouse2" caption="Warehouse 2" %}

There are two two **robots** to choose from:

#### Holonomic logistic robot
* holonomic movement
* square geometry
* lifting platform

{% include gallery id="holonomicrobot" caption="Holonomic logistic robot" %}

#### Ackermann logistic robot
* ackermann movement
* rectangular geometry
* lifting platform

{% include youtubePlayer.html id=page.pashe1 %}

## Theory

This exercise is a motion planning problem. Motion planning is a  Jderobot Academy already has [an exercise dedicated for this](http://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/), which I'd definitely recommend the readers to check it out, so the challenge in this exercise isn't to implement a motion planning algorithm but learning to use the [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library) for our purpose.

### [Open Motion Planning Library](https://ompl.kavrakilab.org/)

OMPL is a library for sampling-based motion planning, offering many state-of-the-art planning algorithms such as PRM, RRT, KPIECE, etc.

{% include gallery id="ompl" caption="OMPL structure" %}

As you can see in the diagram above, some key components of OMPL are:
* **State Space** defines the possible configurations that a robot can have. For example:
  * RealVectorStateSpace: represents an Euclidean space
  * SO2StateSpace, SO3StateSpace: represents rotations in 2D and 3D
  * SE2StateSpace, SE3StateSpace: combines translations and rotations in 2D and 3D
  * ...
* **State Validaty Checker** determines if the configuration is valid, that is to say the configuration doesn't collides with an enviroment obstacle and respects the constraints of the robot.
* **Motion Validator** checks the validity of motions between two states.
* **Control Space** defines the movements that a robot can have.
* **State Propagator** indicates the evolution of the system after applying a control.
* **Space Information** is the container that holds the state space, the state validity checker, and other information needed for planning.
* **Planner** responsible for generating a path from the start to the goal in the configuration space. OMPL supports a variety of planners, such as RRT, PRM, and FMT*.
* **Path** is the output of the planner, which is a sequence of states representing a trajectory for the robot to follow.

The following example shows a 2D point robot inside a 10x10 space with an ovular obstacle at position (5.5, 5.5):
```
from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

# specify valid state condition
def isStateValid(state):
  x = state.getX()
  y = state.getY()
  if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0:
    return False
  return True

def plan():
  # Construct the robot state space in which we're planning. We're
  # planning in [0,1]x[0,1], a subset of R^2.
  space = ob.SE2StateSpace()

  # set state space's lower and upper bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(0, dimensions[0])
  bounds.setLow(1, dimensions[1])
  bounds.setHigh(0, dimensions[2])
  bounds.setHigh(1, dimensions[3])
  space.setBounds(bounds)

  # construct a space information instance for this state space
  si = ob.SpaceInformation(space)
  # set state validity checking for this space
  si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

  # Set our robot's starting and goal state
  start = ob.State(space)
  start().setX(0)
  start().setY(0)
  start().setYaw(math.pi / 4)
  goal = ob.State(space)
  goal().setX(10)
  goal().setY(10)
  goal().setYaw(math.pi / 4)

  # create a problem instance
  pdef = ob.ProblemDefinition(si)

  # set the start and goal states
  pdef.setStartAndGoalStates(start, goal)

  # create a planner for the defined space
  planner = og.RRTConnect(si)

  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  solved = planner.solve(1.0)
  if solved:
    print(pdef.getSolutionPath())
    plot_path(pdef.getSolutionPath(), dimensions)

def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

def plot_path(solution_path, dimensions):
  matrix = solution_path.printAsMatrix()
  path = create_numpy_path(matrix)
  x, y = path.T
  ax = plt.gca()
  ax.plot(x, y, 'r--')
  ax.plot(x, y, 'go') 
  ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
  ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2]))

  plt.show()

if __name__ == "__main__":
  dimensions = [0, 0, 10, 10] 
  obstacle = [5.5, 5.5, 1]   # [x, y, radius]
  plan()
```

Output:
```
Info:    RRTConnect: Space information setup was not yet called. Calling now.
Debug:   RRTConnect: Planner range detected to be 3.142586
Info:    RRTConnect: Starting planning with 1 states already in datastructure
Info:    RRTConnect: Created 9 states (4 start + 5 goal)
Geometric path with 7 states
Compound state [
RealVectorState [0 0]
SO2State [0.785398]
]
Compound state [
RealVectorState [1.26041 2.4087]
SO2State [-0.0626893]
]
Compound state [
RealVectorState [3.38076 4.59672]
SO2State [0.128782]
]
Compound state [
RealVectorState [3.66427 7.34115]
SO2State [0.895879]
]
Compound state [
RealVectorState [4.7318 7.51811]
SO2State [0.808152]
]
Compound state [
RealVectorState [7.7113 8.01201]
SO2State [0.563304]
]
Compound state [
RealVectorState [10 10]
SO2State [0.785398]
]

```

Plot:
{% include gallery id="example" caption="Example" %}

To better understand how to use the library, it is highly recommended to go to the [tutorials](https://ompl.kavrakilab.org/tutorials.html) and [demos](https://ompl.kavrakilab.org/group__demos.html) sections of the official website.

### Conversion From 3D to 2D
**Robot Localization** is the process of determining, where robot is located with respect to it's environment. Localization is a an important resource to us in solving this exercise. Localization can be accomplished in any way possible, be it Monte Carlo, Particle Filter, or even Offline Algorithms. Since, we have a map available to us, offline localization is the best way to move forward. Offline Localization will involve converting from a 3D environment scan to a 2D map. There are again numerous ways to do it, but the technique used in exercise is using **transformation matrices**.

#### Transformation Matrices
In simple terms, transformation is an invertible function that maps a set _X_ to itself. Geometrically, it moves a point to some other location in some space. Algebraically, all the transformations can be mapped using matrix representation. In order to apply transformation on a point, we multiply the point with the specific transformation matrix to get the new location. Some important transformations are:

- **Translation**

Translation of Euclidean Space(2D or 3D world) moves every point by a fixed distance in the same direction

- **Rotation**

Rotation spins the object around a fixed point, known as center of rotation.

- **Scaling**

Scaling enlarges or diminishes objects, by a certain given scale factor.

- **Shear**

Shear rotates one axis so that the axes are no longer perpendicular.

In order to apply multiple transformations all at the same time, we use the concept of **Transformation Matrix** which enables us to multiply a single matrix for all the operations at once!

![Transformation Matrix](https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcRj8-LHyAz5s62zEx9fnQg_KLZX08E_rbdfHQ52kQZwh3BvPqMl)

*Transformation Matrix*

In our case we need to map a 3D Point in gazebo, to a 2D matrix map of our house. The equation used in the exercise was:

![Coordinate to Pixel]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/coord2pix.png)

*Coordinate to Pixel Conversion Equation*

In order to carry out the inverse operation of 3D to 2D, we can simply multiply, the pixel vector with the inverse of the transformation matrix to get the gazebo vector. The inverse of the matrix exists because the **mapping is invertible** and we **do not care about the z coordinate of the environment**, implying that each point in gazebo corresponds to a single point in the map.


## Hints

Simple hints provided to help you solve the Amazon Warehouse exercise. Please note that the **full solution has not been provided.** Also, the hints are more related to the reference solution, since multiple solutions are possible for this exercise.

### Ideas for solving the exercise

There are different ways to solve the exercise

* #### Phase 1: Euclidean planning 
  Define the robot as a 2D point and thicken the obstacles' edges according to the robot's radius to avoid collision, in this case, the state space can simply be an Euclidean space and all those black pixels will be invalid states. Maybe this [demo](https://ompl.kavrakilab.org/Point2DPlanning_8cpp_source.html) can help you!

* #### Phase 2: Planning taking into consideration robot's geometry
  Since the robot's geometry is not always suitable to be approximated as a point, as in the case when the rectangular shelving is loaded, it would be a better way to solve the problem taking into consideration the robot's geometry. And this is the challenge of this phase!
  
  You can represent the robot as a set of pixels, so that a state will be valid when all pixels in the set are free. Now, it is no longer enough to use only Euclidean space but you must take into account the rotation of the robot. Therefore, it would be convenient to use a state space that includes that variable, such as SE2StateSpace. And what else? A motion validator will be also essential, to ensure the feasibility of transitioning between states. Check this [demo](https://ompl.kavrakilab.org/GeometricCarPlanning_8cpp_source.html)!

{% include youtubePlayer.html id=page.robotgeometry %}

### How to find the shortest path?
The library offers the possibility to set an optimization objective, which could be a great help in finding the shortest path. Check out in [this tutorial](https://ompl.kavrakilab.org/optimalPlanningTutorial.html) how to do it!

### Points to consider
* The loaded shelf is no longer a obstacle but part of the robot, then:
  * the robot's geometry changes, its "radius" increases
  * remember to exclude the shelf itself when defining invalid states

### Important points to remember
* Convert the coordinates from meter to pixel before representing with *GUI.showPath(array)*.


## Videos

### Demonstrative video of completed solution

{% include youtubePlayer.html id=page.pashe1 %}

- Contributors: [Lucía Lishan Chen Huang](https://github.com/lu164), [Blanca Soria Rubio](https://github.com/Blancasr), [Jose María Cañas](https://github.com/jmplaza)
- Maintained by [Lucía Lishan Chen Huang](https://github.com/lu164).

## References

1. [https://ompl.kavrakilab.org/](https://ompl.kavrakilab.org/)
2. [https://ompl.kavrakilab.org/OMPL_Primer.pdf](https://ompl.kavrakilab.org/OMPL_Primer.pdf)