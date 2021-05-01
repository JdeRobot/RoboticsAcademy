---
permalink: /exercises/MobileRobots/vacuum_cleaner
title: "Vacuum Cleaner"

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
## Versions to run the exercise

Currently, there are 2 versions for running this exercise:

- ROSNode Templates
- Web Templates(Current Release)

The instructions for both of them are provided as follows.

## Goal

The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

<img src="/RoboticsAcademy/assets/images/exercises/vacuum_cleaner/vacuum_cleaner.png" width="100%" height="60%">
{% include gallery caption="Vacuum cleaner." %}

## Instructions for Web Templates
This is the preferred way for running the exercise.

### Installation 
- Clone the Robotics Academy repository on your local machine

	```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
	```

- Download [Docker](https://docs.docker.com/get-docker/)

- Pull the current distribution of Robotics Academy Docker Image

	```bash
docker pull jderobot/robotics-academy
	```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

#### Enable GPU Acceleration (For advanced users)
- For Linux machines, GPU acceleration can be enabled by downloading Nvidia Container Runtime, as given [here](https://github.com/NVIDIA/nvidia-container-runtime)

- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background

	```bash
docker run -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy python3.8 manager.py
	```

- On the local machine navigate to the follow_line exercise which is: `RoboticsAcademy/exercises/vacuum_cleaner/web-template`

- Launch the `exercise.html` web-page. Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected.

- The exercise can be used after the alert.

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

* **Frequency Slider**: This slider adjusts the running frequency of the iterative part of the code(under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The Target Frequency is the one set on the Slider and Measured Frequency is the one measured by the computer(a frequency of execution the computer is able to maintain despite the commanded one). The student should adjust the Target Frequency according to the Measured Frequency.

* **Debug Level**: This decides the debugging level of the code. A debug level of 1 implies no debugging at all. At this level, all the GUI functions written by the student are automatically removed when the student sends the code to the robot. A debug level greater than or equal to 2 enables all the GUI functions working properly.

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student is provided with `console.print()` similar to `print()` command in the Python Interpreter. 

**Application Programming Interface**

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.bumper.getBumperData().state` - To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* `HAL.bumper.getBumperData().bumper` - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its left and 2 if the collision is at its right.
* `HAL.laser.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
* `HAL.motors.sendV()` - to set the linear speed
* `HAL.motors.sendW()` - to set the angular velocity

## Instructions for ROSNode Templates

### Installation 
Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.

Enable Kobuki_msgs:

```bash
sudo apt-get install ros-melodic-kobuki-msgs
```

### How to run

1. Execution without watching the world: 
```bash
roslaunch vacuum_cleaner.launch
```
2. Execution of the practice and the user interface: 
```bash
python2 vacuumCleaner.py vacuumCleaner_conf.yml
```
3. Execution of the automatic evaluator:
```bash
python2 referee.py referee.yml
```

To simplify the closure of the environment, simply close the VacuumCleaner window(s). *Ctrl + C will give problems*.

### How to do the practice

To carry out the practice, you must edit the MyAlgorithm.py file and insert the control logic into it.

### Where to insert the code

MyAlgorithm.py

```python
print ('Execute')

# TODO

print self.pose3d.getPose3d().x
print self.pose3d.getPose3d().y
# Vacuum's yaw
yaw = self.pose3d.getPose3d().yaw
```

```python
# EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
vel = CMDVel()
myW=x*(self.motors.getMaxW())
myV=-y*(self.motors.getMaxV())
vel.vx = myV
vel.az = myW
self.motors.sendVelocities(vel)
# OR
self.motors.sendAZ(vel.az)
self.motors.sendV(vel.vx)
```

### API

* `self.pose3d.getPose3d().yaw` - to get the orientation of the robot
* `self.bumper.getBumperData().state` - to establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* `self.bumper.getBumperData().bumper` - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its left and 2 if the collision is at its right.
* `laser.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
* `self.motors.sendVelocities(vel)` - to set linear and angular velocity. Object 'vel' must be a CMDVel():
    - `vel` = CMDVel()
    - `vel.vx` = v - linear velocity
    - `vel.az` = w - angular velocity

For this example, it is necessary to ensure that the vacuum cleaner covers the highest possible percentage of the house. The application of the automatic evaluator (referee) will measure the percentage traveled, and based on this percentage, will perform the qualification of the solution algorithm.

### Types conversion

- **Laser**

```python
laser_data = self.laser.getLaserData()

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


## Demonstrative Video

{% include youtubePlayer.html id=page.youtubeId1 %}

*This solution is an illustration for the ROSNode Templates*

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
