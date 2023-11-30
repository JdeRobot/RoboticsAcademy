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
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_general.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_general.png
    alt: "Warehouse"
    title: "Warehouse"
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_teaser.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_teaser.png
    alt: "Shelves to move"
    title: "Shelves to move"
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
    alt: "Robot"
    title: "Robot"

ompl:
  - url: /assets/images/exercises/amazon_warehouse/OMPL_structure.png
    image_path: /assets/images/exercises/amazon_warehouse/OMPL_structure.png
    alt: "OMPL structure"
    title: "OMPL structure"

youtubeId: EVt9vYqEoDg
---

## Goal

The objective of this practice is to implement the logic that allows a holonomic logistics robot to deliver a shelf to the required place by making use of the location of the robot. The robot is equipped with a map and knows its current location in it. The main objective will be to find the shortest path to complete the task.

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

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getPose3d()` - returns x,y and theta components of the robot in world coordinates
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular speed
* `HAL.load()` - to load the platform
* `HAL.unload()` - to unload the platform
* `GUI.showPath(array)` - shows a path on the map. The parameter should be a 2D array containing each of the points of the path

## Supporting information

You can get the warehouse's map from there: /RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png

The map image has a resolution of 415x279 pixels which corresponds to a space with a width of 20.62 meters and a height of 13.6 meters in Gazebo.

The shelves to be moved are at the following coordinates (from left to right): (3.72, 0.57), (3.72, -4.82), (3.72, -8.66), (3.72, -1.24), (3.72, -3.0), (3.72, -6.75)

## Theory

This exercise is a motion planning problem. Jderobot Academy already has [an exercise dedicated for this](http://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/), which I'd definitely recommend the readers to check it out, so the challenge in this exercise is not to implement a motion planning algorithm but learning to use the [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library) for our purpose.

### [Open Motion Planning Library]((https://ompl.kavrakilab.org/))

OMPL is a library for sampling-based motion planning, offering many state-of-the-art planning algorithms such as PRM, RRT, KPIECE, etc.

{% include gallery id="ompl" caption="OMPL structure" %}

As you can see, some key components of OMPL are:
* **State Space** defines the possible configurations that a robot can have. For example:
  * RealVectorStateSpace: represents an Euclidean space
  * SO2StateSpace, SO3StateSpace: represents rotations in 2D and 3D
  * SE2StateSpace, SE3StateSpace: combines translations and rotations in 2D and 3D
  * ...
* **State Validaty Checker** determines if the configuration is valid, that is to say the configuration doesn't collides with an enviroment obstacle and respects the constraints of the robot.
* **Control Space** defines the movements that a robot can have.
* **State Propagator** indicates the evolution of the system after applying a control.
* **Space Information** is the container that holds that state space, the state validity checker, and other information needed for planning.
* **Planner** responsible for generating a path from the start to the goal in the configuration space. OMPL supports a variety of planners, such as RRT, PRM, and FMT*.
* **Path** is the output of the planner, which is a sequence of states representing a trajectory for the robot to follow.

Here's a mini example:
```
from ompl import base as ob
from ompl import geometric as og

# specify valid state condition
def isStateValid(state):
    return state.getX() < .6

# create a SE2 state space
space = ob.SE2StateSpace()

# set state space's lower and upper bounds
bounds = ob.RealVectorBounds(2)
bounds.setLow(-1)
bounds.setHigh(1)
space.setBounds(bounds)

# Set up the problem
ss = og.SimpleSetup(space)

# set up state validity checker
ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

# define start and goal states
start = ob.State(space)
start().setX(.0)
start().setY(.0)
start().setYaw(0)
goal = ob.State(space)
goal().setX(-.5)
goal().setY(-.5)
goal().setYaw(3.14)
ss.setStartAndGoalStates(start, goal)

# solve the problem and print the solution if exists
solved = ss.solve(1.0)
if solved:
  # try to shorten the path
  ss.simplifySolution() 
  print(ss.getSolutionPath())
```

## Hints

Simple hints provided to help you solve the Amazon Warehouse exercise. Please note that the **full solution has not been provided.** Also, the hints are more related to the reference solution, since multiple solutions are possible for this exercise.

### Idea for solving the exercise
Define the robot as a point and thicken the obstacles' edges to avoid collision, in this case, the state space can simply be an Euclidean space and all those black pixels will be invalid states.

### Important points to remember
* Convert the coordinates from meter to pixel before representing with *GUI.showPath(array)*.
* The robot's geometry changes when loading the shelf, its radius "increases".

## Videos

### Demonstrative video of completed solution

{% include youtubePlayer.html id=page.youtubeId %}

- Contributors: [Lucía Lishan Chen Huang](https://github.com/lu164), [Blanca Soria Runio](https://github.com/Blancasr), [Jose María Cañas](https://github.com/jmplaza)
- Maintained by [Lucía Lishan Chen Huang](https://github.com/lu164).

## References

1. [https://ompl.kavrakilab.org/](https://ompl.kavrakilab.org/)
2. [https://ompl.kavrakilab.org/OMPL_Primer.pdf](https://ompl.kavrakilab.org/OMPL_Primer.pdf)