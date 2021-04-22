---
permalink: /exercises/Drones/follow_turtlebot
title: "Follow Turtlebot"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Follow Turtlebot"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/follow_turtlebot/follow_turtlebot.jpg
    image_path: /assets/images/exercises/follow_turtlebot/follow_turtlebot.jpg
    alt: "Follow Turtlebot."
    title: "Follow Turtlebot."

youtubeId: uehDVlBzpmU
---

The goal of this exercise is to implement the logic that allows a quadrotor to follow a turtlebot on the ground by following the movements made by it.

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
docker run -it -p 8080:8080 -p 7681:7681 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 jderobot/robotics-academy python3.8 manager.py
```

- Go to *RoboticsAcademy/exercises/follow_turtlebot/web-template* and open `exercise.html` on you web browser.

- Launch the `exercise.html` web-page. Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected.

### How should I solve the exercise?
The launched webpage contains several widgets that will help you to solve the exercise.

- **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the position of the robot).
- **Frequency Slider**: This slider adjusts the running frequency of the iterative part of the code(under the while True:). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The Target Frequency is the one set on the Slider and Measured Frequency is the one measured by the computer(a frequency of execution the computer is able to maintain despite the commanded one). The student should adjust the Target Frequency according to the Measured Frequency.
- **Debug Level**: This decides the debugging level of the code. A debug level of 1 implies no debugging at all. At this level, all the GUI functions written by the student are automatically removed when the student sends the code to the robot. A debug level greater than or equal to 2 enables all the GUI functions working properly.
- **Pseudo Console**: This shows the error messages related to the student’s code that is sent. In order to print certain debugging information on this console. The student is provided with console.print() similar to print() command in the Python Interpreter.

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

Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy. You won't need jderobot-base, so you can skip those steps.

However, there are some additional dependencies. Install **JdeRobot-drones**, **MAVROS** and **PX4** following the [Drones installation instructions](/RoboticsAcademy/installation/#specific-infrastructure).

Finally, install `turtlebot3`:

```bash
sudo apt-get install -y ros-melodic-turtlebot3-description
```

## How can I run the exercise?

To launch the exercise, simply use the following command from this directory:

```bash
roslaunch follow_turtlebot.launch
```

Notice that the ground robot will start moving by itself after a few seconds of launching.

## How should I solve the exercise?

To solve the exercise, you must edit the `my_solution.py` file and insert the control logic into it.

### Where to insert the code

Your code has to be entered in the `execute` function between the `Insert your code` here comments.

`my_solution.py`

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

**To remember:** *At the moment, each time you update your code you must to run again the launch file in order to insert the updated code in the drone teleoperator GUI*

## API
You can access to the drone methods through the Hardware Abstraction Layer (HAL).

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

<!--## Theory

**Comming soon.**-->

## Hints

Simple hints provided to help you solve the follow_road exercise. Please note that the **full solution has not been provided.**

### TurtleBot Detection

Notice that the ground robot has a visual signal over it. You can filter this signal in order to follow the ground robot.

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

Another possibility is to use **Nose Forward Mode**, where the drone follows the path similar to a fixed-wing aircraft. Then, to accomplish it, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

In this exercise, you should use the Nose Forward Mode.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId %}

---------

## Contributors

- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt) and [Pedro Arias](https://github.com/pariaspe).
- Maintained by [Pedro Arias](https://github.com/pariaspe).
