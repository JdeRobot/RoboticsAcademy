---
permalink: "/exercises/AutonomousCars/autoparking/"
title: "Autoparking"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Autoparking"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/autoparking/autoparking.png
    image_path: /assets/images/exercises/autoparking/autoparking.png
    alt: "Autoparking"
    title: "Autoparking"

  - url: /assets/images/exercises/autoparking/autoparking_2.png
    image_path: /assets/images/exercises/autoparking/autoparking_2.png
    alt: "Autoparking"
    title: "Autoparking"
      
  - url: /assets/images/exercises/autoparking/autoparking_3.png
    image_path: /assets/images/exercises/autoparking/autoparking_3.png
    alt: "Autoparking"
    title: "Autoparking"

gifs:
  - url: /assets/images/exercises/autoparking/autoparking_web_template.gif
    image_path: /assets/images/exercises/autoparking/autoparking_web_template.gif
    alt: "Autoparking"
    title: "Autoparking Web Template"


youtubeId1: BpHSDrFqpVk
youtubeId2: EWTR9Y1QyBk
youtubeId3: 2tDKgsM8nyA
---
## Goal

The objective of this exercise is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must find a parking space and park properly.


{% include gallery caption="Gallery" %}

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

* `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getPose3d()` - to get all the position information.
* `HAL.getPose3d().x` - to get the position of the robot (x coordinate).
* `HAL.getPose3d().y` - to get the position of the robot (y coordinate).
* `HAL.getPose3d().yaw` - to get the orientation of the robot with
  regarding the map
* `HAL.getFrontLaserData()` - to obtain the front laser sensor data.
  It is composed of 180 pairs of values: (0-180º distance in millimeters)
* `HAL.getRightLaserData()` - to obtain the right laser sensor data.
It is composed of 180 pairs of values: (0-180º distance in millimeters)
* `HAL.getBackLaserData()` - to obtain the back laser sensor data.
It is composed of 180 pairs of values: (0-180º distance in millimeters)
* `HAL.setV()` - to set the linear speed.
* `HAL.setW()` - to set the angular velocity.

## Laser attributes
`HAL.getFrontLaserData()`, `HAL.getRightLaserData()` and `HAL.getBackLaserData()` returns an instance of a Class with the following attributes:
* `minAngle` - Start angle of the scan [rad]
* `maxAngle` - End angle of the scan [rad]
* `minRange` - minimum range value [m]
* `maxRange` - maximum range value [m]
* `values` - A list of 180 measurements [m] (Note: values < minRange or > maxRange should be discarded)

## Illustrations

{% include gallery id="gifs" caption="Autoparking Web Template" %}

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId2 %}

{% include youtubePlayer.html id=page.youtubeId3 %}

## Contributors

- Contributors: Vanessa Fernández Martínez, [Javier Izquierdo](https://github.com/javizqh).
- Maintained by Vanessa Fernández Martínez, [Javier Izquierdo](https://github.com/javizqh).
