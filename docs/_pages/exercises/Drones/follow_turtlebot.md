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
## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to follow a turtlebot on the ground by following the movements made by it.

{% include gallery caption="Gallery." %}

## Instructions for Web Templates
This is the preferred way for running the exercise.

### Installation 

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

	```bash
  docker pull jderobot/robotics-academy:2.4.2
  ```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

### Enable GPU Acceleration
- For Linux machines with NVIDIA GPUs, acceleration can be enabled by using NVIDIA proprietary drivers and executing the following docker run command:
  ```bash
  docker run -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:2.4.2 ./start.sh
  ```

- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background ([hardware accelerated version](#enable-gpu-acceleration))

	```bash
  docker run -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:2.4.2 ./start.sh
  ```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected. 

- The exercise can be used after the alert.

### Where to insert the code?

In the launched webpage, type your code in the text editor,

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!
```

### Using the Interface

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation(primarily, the position of the robot).

* **Frequency Slider**: This slider adjusts the running frequency of the iterative part of the code(under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The Target Frequency is the one set on the Slider and Measured Frequency is the one measured by the computer(a frequency of execution the computer is able to maintain despite the commanded one). The student should adjust the Target Frequency according to the Measured Frequency.

* **Debug Level**: This decides the debugging level of the code. A debug level of 1 implies no debugging at all. At this level, all the GUI functions written by the student are automatically removed when the student sends the code to the robot. A debug level greater than or equal to 2 enables all the GUI functions working properly.

* **Psuedo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student can use the `print()` command in the Editor. 

## Application Programming Interface

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

* `GUI.showImage(cv2_image)` - Shows a image of the camera  in the GUI
* `GUI.showLeftImage(cv2_image)` - Shows another image of the camera in the GUI

<!--## Theory
**Comming soon.**-->


## Hints

Simple hints provided to help you solve the follow_turtlebot exercise. Please note that the **full solution has not been provided.**

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

- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe) and [Arkajyoti Basak](https://github.com/iamarkaj).
- Maintained by [Pedro Arias](https://github.com/pariaspe) and [Arkajyoti Basak](https://github.com/iamarkaj).
