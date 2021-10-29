---
permalink: "/exercises/AutonomousCars/car_junction/"
title: "Car Junction"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Visual Follow Line"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/car_junction/car_junction.png
    image_path: /assets/images/exercises/car_junction/car_junction.png
    alt: "Car Junction"
    title: "Car Junction"

  - url: /assets/images/exercises/car_junction/car_junction_2.png
    image_path: /assets/images/exercises/car_junction/car_junction_2.png
    alt: "Car Junction"
    title: "Car Junction"

networks:
  - url: /assets/images/exercises/car_junction/unet.jpg
    image_path: /assets/images/exercises/car_junction/unet.jpg
    alt: "U Net"
    title: "U Net"

  - url: /assets/images/exercises/car_junction/fpn.png
    image_path: /assets/images/exercises/car_junction/fpn.png
    alt: "Feature Pyramid Network"
    title: "Feature Pyramid Network"

  - url: /assets/images/exercises/car_junction/rcnn.png
    image_path: /assets/images/exercises/car_junction/rcnn.png
    alt: "Region CNN"
    title: "Region CNN"

illustrations:
  - url: /assets/images/exercises/car_junction/segment.png
    image_path: /assets/images/exercises/car_junction/segment.png
    alt: "Road Segmentation"
    title: "Road Segmentation"

  - url: /assets/images/exercises/car_junction/stop.png
    image_path: /assets/images/exercises/car_junction/stop.png
    alt: "Stop Sign"
    title: "Stop Sign"

  - url: /assets/images/exercises/car_junction/wrong_turn.gif
    image_path: /assets/images/exercises/car_junction/wrong_turn.gif
    alt: "Wrong Lane"
    title: "Wrong Lane"


youtubeId: 4WIi2cpaLDA
---

## Versions to run the exercise

- Web Templates (Current Release)

## Goal

The goal of this practice is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must Stop at the T joint, where there is a stop sign, wait until there are no cars and pass once the road is clear.

{% include gallery caption="Gallery" %}

## Instructions for Web Templates
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
  docker run --rm -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:latest ./start.sh
  ```


- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background ([hardware accelerated version](#enable-gpu-acceleration))

	```bash
  docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:latest ./start.sh
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

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the position of the robot).

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student can use the print() command in the Editor. 

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage('left')` - to get the left image
* `HAL.getImage('right')` - to get the right image
* `HAL.getImage('center')` - to get the central image
* `HAL.getV()` - to get the linear velocity
* `HAL.getW()` - to get the angular velocity
* `HAL.setV()` - to set the linear velocity
* `HAL.setW()` - to set the angular velocity
* `HAL.getPose3D()` - to obtain the position of the robot
* `HAL.getYaw()` - to obtain the orientation of the robot
* `HAL.setPose3D()` - to set the position of the robot
* `HAL.getTemplate()` - to get the stop's image template
* `GUI.showImages(imageLeft, imageCentral, imageRight)` - allows you to view a debug images or with relevant information

### Example video with web template

## Theory
This exercise mostly revolves around simple Computer Vision and Control mechanisms. Let's start with the Computer Vision tasks:

### Image Segmentation
Image Segmentation is the process of partitioning a digital image into multiple segments. Segmentation is carried out to simplify the representation of information we receive from an input image. There are many ways to carry out image segmentation, but it all depends on the task at hand.

For extremely cluttered images, segmentation is carried out by the use of **Convolutional Neural Networks**. Some of the most useful Neural Networks developed for this task are: UNet, FPN and R-CNNs.

{% include gallery id="networks" caption="Neural Networks" %}

Some simpler methods involve the use of Clustering Algorithms like **K-Means Clustering**. The simplest of all methods is the use of **Color Filters**. 

**Color Filters** involve the use of color spaces and their intensity values. By thresholding the input image by appropriate intensity values, we segment a specific _color part_ of an image. For example, the Stop Sign in the exercise is a very specific red color which can be easily segmented. For more information about this topic, the Robotics Academy already has an exercise based on [Color Filter](http://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/color_filter).

### Motion Detection
Motion detection is the process of detecting a change in the position of an object relative to its surroundings or a change in the surroundings relative to an object. Difference of Frames is the simplest of all motion detection algorithms. By taking the difference between the frame of reference and the current frame, motion can be very easily detected. 

### Control
We require the use of control mechanisms in order to make the Car follow the road and take turns.

**PID** is an excellent control algorithm which works by calculating the error and making adjustments to keep the error as minimum as possible. A very simple example would be, in case of a quadcopter that is required to maintain a particular height: If the height of the quadcopter is extremely far away from the required height(very high or very low), the quadcopter has to make large adjustments in it's rotor speed. If the height is near the required value, the quadcopter has to make small adjustments. [This video](https://youtu.be/LSNVHkeDx-g?t=479) illustrates the concept very well.

[Follow Line](http://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/) is an exercise based completely on the concept of PID.

The physical equation concerning rotational kinematics is v = ωr . This equation governs the relation between the radius of curvature, the magnitude of velocity and the magnitude of angular velocity. In order to increase the radius of curvature, we need to increase the velocity and decrease the magnitude of angular velocity. In order to decrease the radius of curvature, we need to decrease the velocity and increase the magnitude of angular velocity. This is the fundamental equation that allows our autonomous car to take effective turns.

## Hints
Simple hints provided to help you solve the car_junction exercise.

### Following the Road
Color Segmentation is the way to go to detect roads. Thresholding and then applying morphological operations can easily help us detect the roads in the image. The centroid of the generated binary blob can be easily followed by the car. However, there are tiny problems with this approach that can be addressed by using tricks(which the reader is expected to figure out on his/her own).

A really interesting implementation would be, by means of lane detection. Here is a really good implementation. But again, there are some problems in this approach as well. (Do make a video and share with us in case someone solves using this)

### Stop Sign
The stop sign is of a very particular red color, which clearly stands out in the whole gazebo simulation. So, color filters are the best way to approach. Check out this video for more details.

### Turning
Turning is a challenging part of this exercise. The parameters have to be tuned in a specific way to carry out the turn, and then the control should be passed to the follow road method in order to continue with the task. The physical equation given above can simplify the process of setting the parameters.

### Illustrations

{% include gallery id="illustrations" caption="Illustrations" %}

## Contributors

- Contributors: [Irene Lope](https://github.com/ilope236), [Alberto Martín](https://github.com/almartinflorido), [Francisco Rivas](https://github.com/chanfr), [Francisco Pérez](https://github.com/fqez), [Jose María Cañas](https://github.com/jmplaza), [Nacho Arranz](https://github.com/igarag).
- Maintained by [Jessica Fernández](https://github.com/jessiffmm), [Vanessa Fernández](https://github.com/vmartinezf).

## References

[1](https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/)
[2](https://en.wikipedia.org/wiki/PID_controller)
[3](https://www.elprocus.com/the-working-of-a-pid-controller/)
[4](https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm)
[5](https://instrumentationtools.com/open-loop-and-closed-animation-loop/)
[6](https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/)
[7](http://homepages.math.uic.edu/~kauffman/DCalc.pdf)

