---
permalink: /exercises/Drones/follow_road
title: "Follow Road"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Follow Road"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/follow_road/follow_road.jpg
    image_path: /assets/images/exercises/follow_road/follow_road.jpg
    alt: "Follow Road."
    title: "Follow Road."

youtubeId: KLDX4OPTL_c
---

The goal of this exercise is to implement the logic that allows a quadrotor to follow a road. In order to do this, you will have to establish a color filter to segment road lines and then develop an algorithm to follow them until the end of the road.

<!-- <img src="/RoboticsAcademy/assets/images/exercises/cloud.png" width="100%" height="60%"> -->

{% include gallery caption="Gallery." %}

## Requirements

As this is a drones exercise, you will need to additionally install the `jderobot-assets`, `dronewrapper` and `rqt_drone_teleop` packages. These can be installed as

```bash
sudo apt-get install ros-melodic-drone-wrapper ros-melodic-rqt-drone-teleop ros-melodic-jderobot-assets
```

There is an additional dependancy on MAVROS and PX4 that you can fulfill following the [Drones installation instructions](/RoboticsAcademy/installation/#specific-infrastructure).

## How to run

To launch the exercise, simply use the following command from this directory:

```bash
roslaunch follow_road.launch
```

As an easy way to find the values for the color filtering, you can use the colorTuner tool provided in your jderobot installation. After launching the previous command, launch the `colorTuner` in a seperate terminal as follows:

```bash
colorTuner colorTuner.conf
```

## How to do the practice

To solve the exercise, you must edit the `my_solution.py` file and insert the control logic into it.

### Where to insert the code
Your code has to be entered in the `execute` function between the `Insert your code` here comments.

my_solution.py

```python
def execute(event):
  global drone
  img_frontal = drone.get_frontal_image()
  img_ventral = drone.get_ventral_image()
  # Both the above images are cv2 images
  ################# Insert your code here #################################

  set_image_filtered(img_frontal)
  set_image_threshed(img_ventral)

  #########################################################################
```

### API
* `set_image_filtered(cv2_image)` - If you want to show a filtered image of the camera images in the GUI
* `set_image_threshed(cv2_image)` - If you want to show a thresholded image in the GUI
* `drone.get_frontal_image()` - Returns the latest image from the frontal camera as a cv2_image
* `drone.get_ventral_image()` - Returns the latest image from the ventral camera as a cv2_image
* `drone.get_position()` - Returns the position of the drone as a numpy array [x, y, z]
* `drone.get_orientation()` - Returns the roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw]
* `drone.get_roll()` - Returns the roll of the drone
* `drone.get_pitch()` - Returns the pitch of the drone
* `drone.get_yaw()` - Returns the yaw of the drone
* `drone.set_cmd_vel(vx, vy, vz, az)` - Commands the linear velocity of the drone in the x, y and z directions and the angular velocity in z in its body fixed frame


## Theory
**Comming soon.**

## Hints
Simple hints provided to help you solve the follow_road exercise. Please note that the **full solution has not been provided.**

**Comming soon.**

{% include youtubePlayer.html id=page.youtubeId %}

<br/>

# Contributors
- *Authors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt) and [Pedro Arias](https://github.com/pariaspe).* 
- *Maintained by [Pedro Arias](https://github.com/pariaspe).*
