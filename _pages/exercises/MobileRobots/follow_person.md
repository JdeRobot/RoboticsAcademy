---
permalink: /exercises/MobileRobots/follow_person
title: "Follow Person RR"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Follow Person"
toc_icon: "cog"

follow_person_demo:
  - url: /assets/images/exercises/follow_person/follow_person_teaser.png
    image_path: /assets/images/exercises/follow_person/follow_person_teaser.png
    alt: "Follow Person cover"
    title: "Follow Person Cover"

simulated_turtlebot2:
  - url: /assets/images/exercises/follow_person/turtlebot2-sim.png
    image_path: /assets/images/exercises/follow_person/turtlebot2-sim.png
    alt: "Simulated Turtlebot2 (ROS Humble)"
    title: "Simulated Turtlebot2 (ROS Humble)"

r-cnn:
  - url: /assets/images/exercises/follow_person/r-cnn.png
    image_path: /assets/images/exercises/follow_person/r-cnn.png
    alt: "Region-based Convolutional Neural Network (R-CNN)"
    title: "Region-based Convolutional Neural Network (R-CNN)"

how_to_follow_person:
  - url: /assets/images/exercises/follow_person/how_to_follow_person.png
    image_path: /assets/images/exercises/follow_person/how_to_follow_person.png
    alt: "How to follow a person"
    title: "How to follow a person"

pid:
  - url: /assets/images/exercises/follow_person/ControlSystems.jpg
    image_path: assets/images/exercises/follow_person/ControlSystems.jpg
    alt: "Control Systems"
    title: "Control Systems"

  - url: /assets/images/exercises/follow_person/TypesofControlSystems.jpg
    image_path: /assets/images/exercises/follow_person/TypesofControlSystems.jpg
    alt: "Types of Control Systems"
    title: "Types of Control Systems"

  - url: /assets/images/exercises/follow_person/PID.png
    image_path: /assets/images/exercises/follow_person/PID.png
    alt: "PID"
    title: "PID"

vff:
  - url: /assets/images/exercises/follow_person/vff.png
    image_path: /assets/images/exercises/follow_person/vff.png
    alt: "Virtual Force Field (VFF)"
    title: "Virtual Force Field (VFF)"

joystick:
  - url: /assets/images/exercises/follow_person/joystick.png
    image_path: /assets/images/exercises/follow_person/joystick.png
    alt: "Joystick to move the person"
    title: "Joystick to move the person"

youtubeId1: "Tt7RkdUgm_U"
youtubeId2: "fDAU465eVxQ"
---

## Goal

The objective of this practice is to implement the logic of a navigation algorithm that will be used to follow a person in a hospital using a R-CNN (Region based Convolutional Neural Network) called SSD (Single Shot Detector)

{% include gallery id="follow_person_demo" caption="Follow Person Cover" %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Simulated Turtlebot 2 (ROS Humble)
The robot that we will use is a Turtlebot2 (a circular mobile robot) implemented and developed for ROS Foxy and ROS Humble. It has a RGBD camera so that we can detect objects or people, and it has a laser 360º for implement algorithms as VFF if you need to avoid obstacles.

{% include gallery id="simulated_turtlebot2" caption="Simulated Turtlebot2" %}

## Robot API

* `import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage()` - to obtain the current frame of the camera robot.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate)
* `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate)
* `HAL.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `HAL.getLaserData()` - it allows to obtain the data of the laser sensor. It returns a list of 180 laser measurements (0 - 180 degrees)
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity
* `HAL.getBoundingBoxes()` - this method calls a detect() neural network's method to obtain a list of detected objets from an image passed as argument.
* `GUI.showImage()` - to show an opencv image in the web template

## Laser attributes
`HAL.getLaserData()` returns an instance of a Class with the following attributes:
* `minAngle` - Start angle of the scan [rad]
* `maxAngle` - End angle of the scan [rad]
* `minRange` - minimum range value [m]
* `maxRange` - maximum range value [m]
* `values` - A list of 180 measurements [m] (Note: values < minRange or > maxRange should be discarded)

### Bounding Box attributes
`HAL.getBoundingBoxes()` returns an instance a list of Bounding Box Classes with the following attributes:
* `id` - identifier of the type of object (1, 2, 3)
* `class-id` - name of the object (1->person, 2->bicycle, 3->car, ...). It uses a coco_names.py file which you can see in this link: (TODO)
* `xmin` - x value of the top left point of the bounding box
* `ymin` - y value of the top left point of the bounding box
* `xmax` - x value of the bottom right point of the bounding box
* `ymax` - y value of the bottom right point of the boudning box


### Example of use
```python
# Move forward
HAL.setV(0.3)
HAL.setW(0.0)

while True:
    # -- Read from sensors
    img = HAL.getImage()
    bounding_boxes = HAL.getBoundingBoxes(img)
    laser_data = HAL.getLaserData()

    # -- Process sensors data (bounding boxes, laser ...).

    # -- Send commands to actuators.

    # -- Show some results
    GUI.showImage(img)
```

## Theory
When we are designing a robotic application that knows how to follow a person, the most important mission is knowing how to detect it and not lose it.


First step is the detection of persons; we perform this first task using a *Region-based Convolutional Neural Network (R-CNN)*. *CNN* are a type of networks where the first neurons capture groups of pixels and these neurons form new groups for next layers doing convolutions with *Kernel* filters. The neurons of the output layer return the percentage probability of an image to belong to a given class (*classification*). For more information, see this [link](https://www.analyticsvidhya.com/blog/2021/05/convolutional-neural-networks-cnn/). With a *R-CNN* we use a CNN on many regions of the image and we select those regions with more probability of success. There are many types of architectures based on R-CNN as Yolo or SSD. In this exercise you will use a SSD trained model. If you want to know how SSD works you can access this [link](https://developers.arcgis.com/python/guide/how-ssd-works/)

{% include gallery id="r-cnn" caption="Region-based Convolutional Neural Network (R-CNN)" %}

Once we have detected all the people in the image, we can establish several *criteria* to decide which person we are going to follow


In order to dont lose our target we can use *tracking* algorithms. A homemade method that usually works well consists in locating the Centroid of every Bounding Box in each iteration and comparing it with the chosen Centroid of the previous frame. We will stay with that bounding box that has the closest centroid and most similar area to the chosen bounding box of the previous frame.

The second step is to use the kobuki base actuators to move and get closer to the person. To achieve this goal, we look at the *location* of the centroid of the candidate bounding box. Depending on the position we will establish a certain angular speed.

An easy method to implement is *discretized case-based behavior*. We take the width of an image and divide it into X number of columns. We assign a certain angular velocity to each range, and, depending on where the centroid is, we will apply the corresponding velocity.

{% include gallery id="how_to_follow_person" caption="How to follow a person" %}

Another method, a bit more complicated but more efficient, is to implement a PID controller for the angular velocity. With a good design of a [**PID controller**](#pid-controller) we will obtain a more precise and less oscillatory turning response.

However, the robot moves through an environment where there may be obstacles. There is an algorithm called [**VFF (Virtual Force Field)**](#virtual-force-field) that allows us to avoid crashes while following a target. It is based on the sum of attraction and repulsion vectors that cause a different sense of direction.

## Virtual Force Field
The Virtual Force Field Algorithm works this way:

* The robot assigns an *Attraction Vector* to the objective (person). With a image, you will have to use the *Field of View* of the camera (60 degrees) to know the angle of each pixel with the center of the image. You can set a fixed module vector with a 2D camera.
* The robot assigns a *Repulsion Vector* to the obstacle according to its sensor readings that points away from the waypoint. This is done by summing all the vectors that are translated from the sensor readings.
* The robot follows the *Final Vector* obtained by summing the target and obstacle vector.

{% include gallery id="vff" caption="Virtual Force Field" %}

## PID Controller
To understand PID Control, let us first understand what is Control in general.

### Control System

A system of devices or set of devices, that manages, commands, directs or regulates the behavior of other devices or systems to achieve the desired results. Simply speaking, a system which controls other systems. Control Systems help a robot to execute a set of commands precisely, in the presence of unforeseen errors.

### Types of Control System
#### Open Loop Control System
A control system in which the control action is completely independent of the output of the system. A manual control system is on Open Loop System.

#### Closed Loop Control System
A control system in which the output has an effect on the input quantity in such a manner that the input will adjust itself based on the output generated. An open loop system can be converted to a closed one by providing feedback.

### PID Control
A control loop mechanism employing feedback. A PID Controller continuously calculates an error value as the difference between desired output and the current output and applies a correction based on proportional, integral and derivative terms(denoted by P, I, D respectively).

- **Proportional**

Proportional Controller gives an output which is proportional to the current error. The error is multiplied with a proportionality constant to get the output. And hence, is 0 if the error is 0.

- **Integral**

Integral Controller provides a necessary action to eliminate the offset error which is accumulated by the P Controller.It integrates the error over a period of time until the error value reaches to zero.

- **Derivative**

Derivative Controller gives an output depending upon the rate of change or error with respect to time. It gives the kick start for the output thereby increasing system response.

{% include gallery id="pid" caption="Control Systems and PID" %}

## Person model teleoperator
The web-template has a teleoperator that allows you to move the person inside the hospital. To Control the person click the button and then you will can use AWSD keys to move the model. And clicking the button again can return to autonomous mode.

{% include gallery id="joystick" %}


## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

*Example of use of Person Teleoperator*

{% include youtubePlayer.html id=page.youtubeId2 %}

*Reference solution of Simulated Follow Person*

<br/>

## Contributors

- Contributors: [Carlos Caminero Abad](https://github.com/Carlosalpha1), [Jose María Cañas](https://github.com/jmplaza), [Lucía Lishan Chen Huang](https://github.com/lu164)
- Maintained by [Carlos Caminero Abad](https://github.com/Carlosalpha1), [Lucía Lishan Chen Huang](https://github.com/lu164).

## References

[1](https://analyticsindiamag.com/r-cnn-vs-fast-r-cnn-vs-faster-r-cnn-a-comparative-guide/)
[2](https://www.analyticsvidhya.com/blog/2021/05/convolutional-neural-networks-cnn/)
[3](https://developers.arcgis.com/python/guide/how-ssd-works/)
