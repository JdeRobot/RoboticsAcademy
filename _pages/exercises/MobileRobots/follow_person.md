---
permalink: /exercises/MobileRobots/follow_person
title: "Follow Person"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Follow Person"
toc_icon: "cog"

follow_person_demo:
  - url: /assets/images/exercises/follow_person/follow_person_teaser.png
    image_path: /assets/images/exercises/follow_person/follow_person_teaser.png
    alt: "Follow Person cover"
    title: "Follow Person demo"

simulated_turtlebot2:
  - url: /assets/images/exercises/follow_person/turtlebot2-sim.png
    image_path: /assets/images/exercises/follow_person/turtlebot2-sim.png
    alt: "Simulated Turtlebot2 (ROS Foxy)"
    title: "Simulated Turtlebot2 (ROS Foxy)"

youtubeId1: "Tt7RkdUgm_U"
---

## Goal

The objective of this practice is to implement the logic of a navigation algorithm for follow a person in a hospital using a CNN (Convolutional Neural Network) called SSD

{% include gallery id="follow_person_demo" caption="Follow Person Demo" %}

## Instructions
This is the preferred way for running the exercise.

### Installation 
- Clone the Robotics Academy repository on your local machine

	```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
	```

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

	```bash
docker pull jderobot/robotics-academy:latest
	```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

### Enable GPU Acceleration
- For Linux machines with NVIDIA GPUs, acceleration can be enabled by using NVIDIA proprietary drivers, installing  [VirtualGL](https://virtualgl.org/) and executing the following docker run command:
  ```bash
  docker run --rm -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:3.1.6 ./start.sh
  ```


- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background ([hardware accelerated version](#enable-gpu-acceleration))

	```bash
  docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:3.1.6 ./start.sh
  ```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

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

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Debugging Console**: This shows the error messages related to the student’s code that is sent. The student can also use it to visualize the output of the `print()` function.

## Simulated Turtlebot 2 (ROS Foxy)
The robot that we will use is a Turtlebot2 (a circular mobile robot) implemented and developed for ROS Foxy. It has a RGBD camera so that we can detect objects or people, and it has a laser 360º for implement algorithms as VFF if you need to avoid obstacles.

{% include gallery id="simulated_turtlebot2" caption="Simulated Turtlebot2" %}

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage()` - to obtain the current frame of the camera robot.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate)
* `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate)
* `HAL.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `HAL.getLaserData()` - it allows to obtain the data of the laser sensor.
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity
* `HAL.getBoundingBoxes()` - this method calls a detect() neural network's method to obtain a list of detected objets from an image passed as argument.
* `GUI.showImage()` - to show an opencv image in the web template

### Laser attributes
`HAL.getLaserData()` returns an instance of a Class with the following attributes:
* `minAngle` - start angle of the scan [rad]
* `maxAngle` - end angle of the scan [rad]
* `minRange` - minimum range value [m]
* `maxRange` - maximum range value [m]
* `values` - a list of 360 measurements [m] (Note: values < minRange or > maxRange should be discarded)

### Bounding Box attributes
`HAL.getBoundingBoxes()` returns an instance a list of Bounding Box Classes with the following attributes:
* `id` - identificator of the type of object (1, 2, 3)
* `class-id` - name of the object (1->person, 2->bicycle, 3->car, ...). It uses a coco_names.py file which you can see in this link: (TODO)
* `xmin` - x value of the top left point of the bounding box
* `ymin` - y value of the top left point of the bounding box
* `xmax` - x value of the bottom right point of the bounding box
* `ymax` - y value of the bottom right point of the boudning box


### Example of use
```python
HAL.setV(0.3)
HAL.setW(0.0)

while True:
    # -- Read from sensors
    img = HAL.getImage()
    bounding_boxes = HAL.getBoundingBoxes(img)
    laser_data = HAL.getLaserData()

    # -- Process sensors data.
    # -- Send commands to actuators.

    # -- Show some results
    GUI.showImage(img)
```

## Theory


## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

*Example of use of Person Teleoperator*

<br/>

## Contributors

- Contributors: [Carlos Caminero Abad](https://github.com/Carlosalpha1), [Jose María Cañas](https://github.com/jmplaza)
- Maintained by [Carlos Caminero Abad](https://github.com/Carlosalpha1).

<!--
Another possible solution is to implement the logic of a navigation algorithm for an autonomous vacuum with autolocation.
{% include youtubePlayer.html id=page.youtubeId2 %}
-->
