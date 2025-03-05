---
permalink: /exercises/Drones/package_delivery
title: "Package Delivery"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Package Delivery"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/package_delivery/package_delivery_teaser.png
    image_path: /assets/images/exercises/package_delivery/package_delivery.png
    alt: "Package Delivery."
    title: "Package Delivery."

# youtubeId1: ID1
youtubeId2: aWkTDp2UobM
---
## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to deliver a package box from the warehouse to a target house. The task of the drone is to:

1. Pick the package box from the warehouse and reach the target house location
2. Locate the target beacon and drop the package box
3. Return to the warehouse

{% include gallery caption="Gallery." %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

### Sensors and drone state

* `HAL.get_position()` - Returns the actual position of the drone as a numpy array [x, y, z], in m.
* `HAL.get_velocity()` - Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s
* `HAL.get_yaw_rate()` - Returns the actual yaw rate of the drone, in rad/s.
* `HAL.get_orientation()` - Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad. 
* `HAL.get_roll()` - Returns the roll angle of the drone, in rad
* `HAL.get_pitch()` - Returns the pitch angle of the drone, in rad.
* `HAL.get_yaw()` - Returns the yaw angle of the drone, in rad. 
* `HAL.get_landed_state()` -  Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown. 
* `HAL.get_pkg_state()` -  Returns True if the drone has picked up the package box else returns False.

### Actuators and drone control

The three following drone control functions are *non-blocking*, i.e. each time you send a new command to the aircraft it immediately discards the previous control command. 

#### 1. Position control

* `HAL.set_cmd_pos(x, y, z, az)` - Commands the *position* (x,y,z) of the drone, in m and the *yaw angle (az)* (in rad) taking as reference the first takeoff point (map frame)

#### 2. Velocity control

* `HAL.set_cmd_vel(vx, vy, vz, az)` - Commands the *linear velocity* of the drone in the x, y and z directions (in m/s) and the *yaw rate (az)* (rad/s) in its body fixed frame

#### 3. Mixed control

* `HAL.set_cmd_mix(vx, vy, z, az)` - Commands the *linear velocity* of the drone in the x, y directions (in m/s), the *height* (z) related to the takeoff point and the *yaw rate (az)* (in rad/s) 

### Drone takeoff and land

Besides using the buttons at the drone teleoperator GUI, taking off and landing can also be controlled from the following commands in your code:

* `HAL.takeoff(height)` - Takeoff at the current location, to the given height (in m)
* `HAL.land()` - Land at the current location. 

### Drone cameras

* `HAL.get_frontal_image()` - Returns the latest image from the frontal camera as a OpenCV cv2_image
* `HAL.get_ventral_image()` - Returns the latest image from the ventral camera as a OpenCV cv2_image

### Package box control

* `HAL.set_cmd_pick()` - Drone picks up the package box from a minimum distance of 0.5 m
* `HAL.set_cmd_drop()` - Drone drops the package box

### GUI

* `GUI.showImage(cv2_image)` - Shows a image of the camera  in the GUI
* `GUI.showLeftImage(cv2_image)` - Shows another image of the camera in the GUI


## Hints

Simple hints provided to help you solve the package_delivery exercise. Please note that the **full solution has not been provided.**

### Position Detection of the Target

Notice that the target house has a visual signal near it. You can filter this signal in order to get the position of the target location.

### How do I get the waypoint coordinates?

1. **Green Pad:** Located at (x, y) = (-1, -4) m
2. **Red Pad:** Located at (x, y) = (-1, -1) m
2. **Target House:** Located at (x, y) = (20, 20) m


### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

Another possibility is to use **Nose Forward Mode**, where the drone follows the path similar to a fixed-wing aircraft. Then, to accomplish it, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

In this exercise, you should use the Nose Forward Mode.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

<!-- ## Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId1 %} -->

-------

## Contributors
- Contributors: [Arkajyoti Basak](https://github.com/iamarkaj), [JoseMaria Cañas](https://github.com/jmplaza), [Pedro Arias](https://github.com/pariaspe).
- Maintained by [Pedro Arias](https://github.com/pariaspe) and [Arkajyoti Basak](https://github.com/iamarkaj).
