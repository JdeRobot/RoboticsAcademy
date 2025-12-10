---
permalink: "/exercises/AutonomousCars/car_junction/"
title: "Road Junction"

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

## Goal

The goal of this practice is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must Stop at the T joint, where there is a stop sign, wait until there are no cars and pass once the road is clear.



{% include gallery caption="Gallery" %}

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
* `HAL.getTemplate()` - to get the image template of the stop sign
* `GUI.showImages(imageLeft, imageCentral, imageRight)` - to visualize the images in the webpage

## Videos

{% include youtubePlayer.html id=page.youtubeId %}

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

1. [https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/](https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/)
2. [https://en.wikipedia.org/wiki/PID_controller](https://en.wikipedia.org/wiki/PID_controller)
3. [https://www.elprocus.com/the-working-of-a-pid-controller/](https://www.elprocus.com/the-working-of-a-pid-controller/)
4. [https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm](https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm)
5. [https://instrumentationtools.com/open-loop-and-closed-animation-loop/](https://instrumentationtools.com/open-loop-and-closed-animation-loop/)
6. [https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/](https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/)
7. [http://homepages.math.uic.edu/~kauffman/DCalc.pdf](http://homepages.math.uic.edu/~kauffman/DCalc.pdf)

