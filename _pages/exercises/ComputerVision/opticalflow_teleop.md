---
permalink: /exercises/ComputerVision/opticalflow_teleop
title: "Optical Flow Teleop"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Optical Flow Teleop"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/opticalflow_teleop/opticalflow_teleop_teaser.png
    image_path: /assets/images/exercises/opticalflow_teleop/opticalflow_teleop_teaser.png
    alt: "Optical Flow Teleop"
    title: "Optical Flow Teleop"
    
basic:
  - url: /assets/images/exercises/opticalflow_teleop/optical_flow_basic.jpg
    image_path: /assets/images/exercises/opticalflow_teleop/optical_flow_basic.jpg
    alt: "Optical Flow Theory"
    title: "Optical Flow Theory"
    
example:
  - url: /assets/images/exercises/opticalflow_teleop/opticalflow_example.jpg
    image_path: /assets/images/exercises/opticalflow_teleop/opticalflow_example.jpg
    alt: "Optical Flow Example"
    title: "Optical Flow Example"

youtubeId1: xUpTw0_jt5s

---

## Goal

In this practice the intention is to develop an optical flow algorithm to teleoperate the robot using the images obtained from a webcam.

{% include gallery caption="Gallery" %}



## Robot API

* `from HAL import HAL` - to import the HAL library class. This class contains the functions that receives information from the webcam.
* `from GUI import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage()` - to get the image
* `GUI.showImage()` - allows you to view a debug image or with relevant information
* `HAL.motors.sendV()` - to set the linear speed
* `HAL.motors.sendW()` - to set the angular velocity

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

## Theory

Optical flow is the pattern of apparent motion of image objects between two consecutive frames caused by the movement of object or camera. It is 2D vector field where each vector is a displacement vector showing the movement of points from first frame to second. Consider the image below:

{% include gallery id="basic" caption="It shows a ball moving in 5 consecutive frames. The arrow shows its displacement vector" %}

Optical flow works on several assumptions:
    1. The pixel intensities of an object do not change between consecutive frames.
    2. Neighbouring pixels have similar motion.  

{% include gallery id="example" caption="Example of optical flow motion estimation" %}

Optical flow has many applications in areas like:
    - Structure from Motion
    - Video Compression
    - Video Stabilization

## Contributors

* Contributors: [Jose María Cañas](https://github.com/jmplaza), [David Valladares](https://github.com/dvalladaresv)
* Maintained by [David Valladares](https://github.com/dvalladaresv)

## References

1. [https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html](https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html)
2. [https://medium.com/@jijupax/connect-the-webcam-to-docker-on-mac-or-windows-51d894c44468](https://medium.com/@jijupax/connect-the-webcam-to-docker-on-mac-or-windows-51d894c44468)
