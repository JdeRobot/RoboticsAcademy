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

## Goal

The goal of this practice is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must Stop at the T joint, where there is a stop sign, wait until there are no cars and pass once the road is clear.

{% include gallery caption="Gallery" %}

## Installation 
Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.

## How to run your solution?

Navigate to the car_junction directory

```bash
cd exercises/car_junction
```

Launch Gazebo with the car_junction world through the command 

```bash
roslaunch ./launch/car_junction.launch
```

Then you have to execute the academic application, which will incorporate your code:

```bash
python2 ./stop.py stop_conf.yml
```

## Where to insert the code?

[MyAlgorithm.py](MyAlgorithm.py#L74)

```
def execute(self):
        
    # Add your code here
    print "Running"
    # Getting the images
    input_image = self.cameraC.getImage().data      
```


## API
* `self.cameraC.getImage().data` - to get image from center camera
* `self.cameraL.getImage().data` - to get image from left camera
* `self.cameraR.getImage().data` - to get image from right camera

* `self.pose3d.getPose3d().x` - to obtain the position of the robot
* `self.pose3d.getPose3d().y` - to obtain the position of the robot
* `self.pose3d.getPose3d().yaw` - to obtain the position of the robot

* `self.motors.sendW()` - to set the angular velocity
* `self.motors.sendV()` - to set the linear velocity

## How to perform the exercise?
To carry out the exercise, you have to edit the file `MyAlgorithm.py` and insert in it your code, which gives intelligence to the autonomous car.

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

## Illustrations

{% include gallery id="illustrations" caption="Illustrations" %}

## Reference Solution

{% include youtubePlayer.html id=page.youtubeId %}

