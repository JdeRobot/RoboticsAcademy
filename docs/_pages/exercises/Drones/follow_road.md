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

youtubeId: KLDX4OPTL_c

---

The goal of this exercise is to implement the logic that allows a quadrotor to follow a road. In order to do this, you will have to establish a color filter to segment road lines and then develop an algorithm to follow them until the end of the road.

<!-- <img src="/RoboticsAcademy/assets/images/exercises/cloud.png" width="100%" height="60%"> -->

{% include gallery caption="Gallery." %}

## Version Clarification
Currently, there are 2 versions for running this exercise:

- v2.3 - Web Templates [*Current Release*]
- v2.1 - ROSNode Templates

Both versions are valid. Web Templates is a dockerized version of ROSNode Templates. It simplifies a lot the installation process while the exercise must be solved through a web browser.

The instructions for both of them are provided as follows.

## Web Templates installation instructions
### Installation instructions

First, pull the last version of our [docker image](https://hub.docker.com/r/jderobot/robotics-academy):
```bash
docker pull jderobot/robotics-academy
```

Notice that you have to have installed [Docker](https://docs.docker.com/get-docker/) to complete the previous step.

Secondly, clone the Robotics Academy repository on your local machine:
```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
```
#### Enable GPU Acceleration (For advanced users)
- For Linux machines, GPU acceleration can be enabled by downloading Nvidia Container Runtime, as given [here](https://github.com/NVIDIA/nvidia-container-runtime)

- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How can I run the exercise?
- Start a new docker container of the image and keep it running in the background:

	```bash
docker run -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy ./start.sh
	```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected. 

### How should I solve the exercise?
The launched webpage contains several widgets that will help you to solve the exercise.

- **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the position of the robot).
- **Frequency Slider**: This slider adjusts the running frequency of the iterative part of the code(under the while True:). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The Target Frequency is the one set on the Slider and Measured Frequency is the one measured by the computer(a frequency of execution the computer is able to maintain despite the commanded one). The student should adjust the Target Frequency according to the Measured Frequency.
- **Debug Level**: This decides the debugging level of the code. A debug level of 1 implies no debugging at all. At this level, all the GUI functions written by the student are automatically removed when the student sends the code to the robot. A debug level greater than or equal to 2 enables all the GUI functions working properly.
- **Debugging Console**: This shows the error messages related to the student’s code that is sent. The student can also use it to visualize the output of the print() function.

### Where to insert the code
To solve the exercise, you must edit the text editor in the launched webpage.

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!


while True:
    # Enter iterative code!
```

Some explanations about the above code:
- It has two parts, a sequential one and iterative one. The sequential (before the while loop) just execs once, while the iterative execs forever.
- `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
- `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.


## ROSNode Templates 

### Installation instructions

Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.

As this is a drones exercise, you will need to additionally install the `jderobot-assets`, `dronewrapper` and `rqt_drone_teleop` packages. These can be installed as

```bash
sudo apt-get install ros-melodic-drone-wrapper ros-melodic-rqt-drone-teleop ros-melodic-jderobot-assets
```

There is an additional dependency on MAVROS and PX4 that you can fulfill following the [Drones installation instructions](/RoboticsAcademy/installation/#specific-infrastructure).

## How can I run the exercise?

To launch the exercise, simply use the following command from this directory:

```bash
roslaunch follow_road.launch
```

## How should I solve the exercise?

To solve the exercise, you must edit the `my_solution.py` file and insert the control logic into it.

### Where to insert the code

Your code has to be entered in the `execute` function between the `Insert your code` here comments.

`my_solution.py`

```python
def execute(event):
  global HAL
  img_frontal = HAL.get_frontal_image()
  img_ventral = HAL.get_ventral_image()
  # Both the above images are cv2 images
  ################# Insert your code here #################################

  set_image_filtered(img_frontal)
  set_image_threshed(img_ventral)

  #########################################################################
```

You must assume your drone has already took off from the central green pad and its already airbone (i.e. you already pressed the Takeoff button). Then your drone has to **navigate autonomously** following the road path reaching the end of the road.

**To remember:** *At the moment, each time you update your code you must to run again the launch file in order to insert the updated code in the drone teleoperator GUI*


## API

### Sensors and drone state

* `HAL.get_position()` - Returns the actual position of the drone as a numpy array [x, y, z], in m.
* `HAL.get_velocity()` - Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s
* `HAL.get_yaw_rate()` - Returns the actual yaw rate of the drone, in rad/s.
* `HAL.get_orientation()` - Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad. 
* `HAL.get_roll()` - Returns the roll angle of the drone, in rad
* `HAL.get_pitch()` - Returns the pitch angle of the drone, in rad.
* `HAL.get_yaw()` - Returns the yaw angle of the drone, in rad. 
* `HAL.get_landed_state()` -  Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown. 

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

### GUI
#### Web Template
* `GUI.showImage(cv2_image)` - Shows a image of the camera  in the GUI
* `GUI.showLeftImage(cv2_image)` - Shows another image of the camera in the GUI

#### ROSNode Template
* `set_image_filtered(cv2_image)` - Shows a filtered image of the camera images in the GUI
* `set_image_threshed(cv2_image)` - Shows a thresholded image in the GUI


## Theory

PID Control is the main fundamental behind this exercise. To understand PID Control, let us first understand what is Control in general.

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

### Tuning Methods

In order for the PID equation to work, we need to determine the constants of the equation. There are 3 constants called the gains of the equation. We have 2 main tuning methods for this.

- **Trial and Error**

It is a simple method of PID controller tuning. While system or controller is working, we can tune the controller. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches to oscillating behavior. Once it is oscillating, adjust Ki (Integral term) so that oscillations stops and finally adjust D to get fast response.

- **Zeigler Nichols method**

Zeigler-Nichols proposed closed loop methods for tuning the PID controller. Those are continuous cycling method and damped oscillation method. Procedures for both methods are same but oscillation behavior is different. In this, first we have to set the p-controller constant, Kp to a particular value while Ki and Kd values are zero. Proportional gain is increased till system oscillates at constant amplitude.


## Hints

Simple hints provided to help you solve the follow_road exercise. Please note that the **full solution has not been provided.**

### Detecting the road to follow
The first task of the assignment is to detect the line to be followed. This can be achieved easily by **filtering the color of the road** from the image and applying basic image processing to find the point or line to follow.

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

In this exercise, your drone should follow the path similarly to how a fixed-wing aircraft would do, namely **nose forward**.  Then, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

### Coding the Controller
The Controller can be designed in various configurations. 3 configurations have been described in detail below:

- **P Controller**
The simplest way to do the assignment is using the P Controller. Just find the error which is the difference between our *Set Point* (the point where our drone should be heading) and the *Current Output* (where the drone is actually heading). Keep adjusting the value of the constant, till we get a value where there occurs no **unstable oscillations** and no **slow response**.

- **PD Controller**
This is an interesting way to see the effect of Derivative on the Control. For this, we need to calculate the derivative of the output we are receiving. Since, we are dealing with *discrete outputs in our case, we simply calculate the difference between our previous error and the present error*, then adjust the proportional constant. Adjust this value along with the P gain to get a good result.

- **PID Controller**
This is the complete implemented controller. Now, to add the I Controller we need to integrate the output from the point where error was zero, to the present output. While dealing with discrete outputs, we can achieve this using *accumulated error*. Then, comes the task of adjustment of gain constants till we get our desired result.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.


## Demonstrative video of the solution
 
{% include youtubePlayer.html id=page.youtubeId %}

---------

## Contributors

- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe) and [Prakarsh Kaushik](https://github.com/RUFFY-369).
- Maintained by [Pedro Arias](https://github.com/pariaspe) and [Prakarsh Kaushik](https://github.com/RUFFY-369).
