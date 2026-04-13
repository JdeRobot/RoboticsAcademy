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
    
pid:
  - url: /assets/images/exercises/follow_road/ControlSystems.jpg
    image_path: assets/images/exercises/follow_road/ControlSystems.jpg
    alt: "Control Systems"
    title: "Control Systems"

  - url: /assets/images/exercises/follow_road/TypesofControlSystems.jpg
    image_path: /assets/images/exercises/follow_road/TypesofControlSystems.jpg
    alt: "Types of Control Systems"
    title: "Types of Control Systems"

  - url: /assets/images/exercises/follow_road/PID.png
    image_path: /assets/images/exercises/follow_road/PID.png
    alt: "PID"
    title: "PID"

youtubeId1: KLDX4OPTL_c
youtubeId2: DdwsbH944M8
youtubeId3: MKA5FcO0Jy8

---
## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to follow a road. In order to do this, you will have to establish a color filter to segment road lines and then develop an algorithm to follow them until the end of the road.

<!-- <img src="/RoboticsAcademy/assets/images/exercises/cloud.png" width="100%" height="60%"> -->

{% include gallery caption="Gallery." %}

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

* `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

* `HAL.get_position()` - Returns the current position of the drone as a numpy array [x, y, z], in m.
* `HAL.get_velocity()` - Returns the current velocities of the drone as a numpy array [vx, vy, vz], in m/s.
* `HAL.get_yaw_rate()` - Returns the current yaw rate of the drone, in rad/s.
* `HAL.get_orientation()` - Returns the current roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad. 
* `HAL.get_roll()` - Returns the roll angle of the drone, in rad
* `HAL.get_pitch()` - Returns the pitch angle of the drone, in rad.
* `HAL.get_yaw()` - Returns the yaw angle of the drone, in rad. 
* `HAL.get_landed_state()` -  Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown. 

### Actuators and drone control

The three following drone control functions are *non-blocking*, i.e. each time you send a new command to the aircraft it immediately discards the previous control command. 

#### 1. Position control

* `HAL.set_cmd_pos(x, y, z, az)` - Commands the *position* (x,y,z) of the drone, in m and the *yaw angle (az)* (in rad) taking as reference the first takeoff point (map frame).

#### 2. Velocity control

* `HAL.set_cmd_vel(vx, vy, vz, az)` - Commands the *linear velocity* of the drone in the x, y and z directions (in m/s) and the *yaw rate (az)* (rad/s) in its body fixed frame.

#### 3. Mixed control

* `HAL.set_cmd_mix(vx, vy, z, az)` - Commands the *linear velocity* of the drone in the x, y directions (in m/s), the *height* (z) related to the takeoff point and the *yaw rate (az)* (in rad/s).

### Drone takeoff and land

Besides through the buttons in the drone teleoperator WebGUI, take off and landing can also be controlled from the following commands in your code:

* `HAL.takeoff(height)` - Takeoff at the current location, to the given height (in m).
* `HAL.land()` - Land at the current location. 

### Drone cameras

* `HAL.get_frontal_image()` - Returns the latest image from the frontal camera as a OpenCV cv2_image.
* `HAL.get_ventral_image()` - Returns the latest image from the ventral camera as a OpenCV cv2_image.

### WebGUI

* `WebGUI.showImage(cv2_image)` - Shows a image of the camera in the WebGUI.
* `WebGUI.showLeftImage(cv2_image)` - Shows another image of the camera in the WebGUI.

### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication with the simulation.

This exercise uses Aerostack2 interfaces for drone control, so the ROS 2-direct version is more advanced than in ground robots. For more information about (Aerostack 2)[https://aerostack2.github.io/]

- `/drone0/frontal_cam/image_raw` - Subscribe to this topic to receive the frontal camera image. Message type: `sensor_msgs/msg/Image`

- `/drone0/ventral_cam/image_raw` - Subscribe to this topic to receive the ventral camera image. Message type: `sensor_msgs/msg/Image`

- `/drone0/self_localization/twist` - Subscribe to this topic to receive the drone twist, including yaw rate. Message type: `geometry_msgs/msg/TwistStamped`

- `/webgui/image_debug_right` - Publish to this topic to display a debug image in the right panel of the WebGUI. Message type: `sensor_msgs/msg/Image`

- `/webgui/image_debug_left` - Publish to this topic to display a debug image in the left panel of the WebGUI. Message type: `sensor_msgs/msg/Image`

- `/drone0/platform/state_machine_event` - Service used internally for takeoff and landing state transitions. Service type: `as2_msgs/srv/SetPlatformStateMachineEvent`

For motion commands, this exercise does not usually publish directly to a simple `/cmd_vel` topic.  
Instead, it uses the Aerostack2 Python API through methods such as:

- `set_cmd_pos(x, y, z, az)` - position command with yaw
- `set_cmd_vel(vx, vy, vz, az)` - velocity command with yaw rate
- `set_cmd_mix(vx, vy, z, az)` - mixed planar velocity + altitude + yaw rate command
- `takeoff(h)` - takeoff command
- `land()` - landing command

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

## Theory

PID Control is the main principle behind this exercise. To understand PID Control, let us first understand what is Control in general.

### Control System

A system of devices or set of devices, that manages, commands, directs or regulates the behavior of other devices or systems to achieve the desired results. Simply speaking, a system which controls other systems. Control Systems help a robot to execute a set of commands precisely, in the presence of unforeseen errors or complications.

### Types of Control System
#### Open Loop Control System
A control system in which the control action is completely independent of the output of the system. An Open Loop System is a manual control system.

#### Closed Loop Control System
A control system in which the output has an effect on the input quantity in such a manner that the input will adjust itself based on the output generated. An open loop system can be converted to a closed one by providing feedback.

### PID Control
A control loop mechanism employing feedback. A PID Controller continuously calculates an error value as the difference between the desired output and the current output and applies a correction based on proportional, integral and derivative terms(denoted by P, I, D respectively).

- **Proportional**

A Proportional Controller gives an output which is proportional to the current error. The error is multiplied with a proportionality constant to get the output. And hence, is 0 if the error is 0.

- **Integral**

An Integral Controller provides a necessary action to eliminate the offset error which is accumulated by the P Controller. It integrates the error over a period of time until the error value reaches zero.

- **Derivative**

A Derivative Controller gives an output depending on the rate of change or error regarding time. It gives the kick start for the output thereby increasing system response.

{% include gallery id="pid" caption="Control Systems and PID" %}

### Tuning Methods

In order for the PID equation to work, we need to determine the constants of the equation. There are 3 constants called the gains of the equation. We have 2 main tuning methods for this.

- **Trial and Error**

It is a simple method of PID controller tuning. While the system or controller is working, we can tune the controller. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches oscillating behavior. Once it is oscillating, we adjust Ki (Integral term) so that oscillations stops and finally adjust D to get a fast response.

- **Zeigler-Nichols method**

Zeigler-Nichols proposed closed loop methods for tuning the PID controller. Those are: continuous cycling method and damped oscillation method. The procedures for both methods are the same but their oscillation is different. Therefore, first we have to set the p-controller constant, Kp to a particular value while Ki and Kd values are zero. Proportional gain is increased until the system oscillates at a constant amplitude.


## Hints

Simple hints provided to help you solve the follow_road exercise. Please note that the **full solution has not been provided.**

### Detecting the road to follow
The first task of the assignment is to detect the line to be followed. This can be achieved easily by **filtering the color of the road** from the image and applying basic image processing to find the point or line to follow.

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but that's not the best way of flying a drone.

In this exercise, your drone should follow the path similarly to how a fixed-wing aircraft would do, namely **nose forward**.  Then, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

### Coding the Controller
The Controller can be designed in various configurations. 3 configurations have been described in detail below:

- **P Controller**
The simplest way to do the assignment is using the P Controller. Just find the error which is the difference between our *Set Point* (the point where our drone should be heading) and the *Current Output* (where the drone is actually heading). Keep adjusting the value of the constant, until we get a value where there occurs no **unstable oscillations** and no **slow response**.

- **PD Controller**
This is an interesting way to see the effects of Derivative on the Control. To do this, we need to calculate the derivative of the output we are receiving. Since we are dealing with *discrete outputs in our case, we simply calculate the difference between our previous error and the present error*, then adjust the proportional constant. Adjust this value along with the P gain to get a good result.

- **PID Controller**
This is the completely implemented controller. Now, to add the I Controller we need to integrate the output from the point where error was zero, to the present output. While dealing with discrete outputs, we can achieve this using *accumulated error*. Then, comes the task of adjusting the gain constants until we get our desired result.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Videos

{% include youtubePlayer.html id=page.youtubeId2 %}

### Demonstrative video of the solution
 
{% include youtubePlayer.html id=page.youtubeId3 %}

---------

## Contributors

- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe), [Prakarsh Kaushik](https://github.com/RUFFY-369) and [Arkajyoti Basak](https://github.com/iamarkaj).
- Maintained by [Pedro Arias](https://github.com/pariaspe), [Prakarsh Kaushik](https://github.com/RUFFY-369) and [Arkajyoti Basak](https://github.com/iamarkaj).
