---
permalink: /exercises/Drones/labyrinth_escape
title: "Labyrinth Escape"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Labyrinth Escape"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/labyrinth_escape/labyrinth_escape.jpg
    image_path: /assets/images/exercises/labyrinth_escape/labyrinth_escape.jpg
    alt: "Labyrinth Escape."
    title: "Labyrinth Escape."

youtubeId: JR5OH_XHw7U
youtubeId2: e2-BE1KKtm0
---

The goal of this exercise is to implement the logic that allows a quadrotor to escape a labyrinth through visual signals placed on the ground.

{% include gallery caption="Gallery." %}

## Web Templates installation instructions
### Installation instructions

First, pull the last version of our [docker image](https://hub.docker.com/r/jderobot/robotics-academy):
```bash
docker pull jderobot/robotics-academy:latest
```

Notice that you have to have installed [Docker](https://docs.docker.com/get-docker/) to complete the previous step.

Secondly, clone the Robotics Academy repository on your local machine:
```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
```
### Enable GPU Acceleration
- For Linux machines with NVIDIA GPUs, acceleration can be enabled by using NVIDIA proprietary drivers, installing  [VirtualGL](https://virtualgl.org/) and executing the following docker run command:
  ```bash
  docker run -it --rm --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:latest ./start.sh
  ```


- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### Optional: Store terminal output
- To store the terminal output of manager.py and launch.py to a file execute the following docker run command and keep it running in the background:
```bash
docker run -it --rm -v $HOME/.roboticsacademy:/logs --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:latest ./start_logs.sh
```

- After the session, execute the following command to view logs:
```bash
more $HOME/.roboticsacademy/launch.log
more $HOME/.roboticsacademy/stderr.log
```

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background ([hardware accelerated version](#enable-gpu-acceleration)/[store terminal output](#optional-store-terminal-output))

	```bash
  docker run -it --rm -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:latest ./start.sh
  ```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

### How should I solve the exercise?
The launched webpage contains several widgets that will help you to solve the exercise.

- **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the position of the robot).

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

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

## Robot API

Some explanations about the above code:
- It has two parts, a sequential one and iterative one. The sequential (before the while loop) just execs once, while the iterative execs forever.
- `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
- `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

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

* `GUI.showImage(cv2_image)` - Shows a image of the camera  in the GUI
* `GUI.showLeftImage(cv2_image)` - Shows another image of the camera in the GUI

<!--## Theory

**Comming soon.**-->

## Hints

Simple hints provided to help you solve the labyrinth_escape exercise. Please note that the **full solution has not been provided.**

### Detecting the visual signals
The first task of the assignment is to detect the visual signals. This can be achieved easily by **filtering the color of the road** from the image.

Secondly, you might process the signal extracting the direction to follow. Notice that there are only four different possibilities.

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

Another possibility is to use **Nose Forward Mode**, where the drone follows the path similar to a fixed-wing aircraft. Then, to accomplish it, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

In this exercise, you can use the one you prefer.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Web Template teaser video

{% include youtubePlayer.html id=page.youtubeId2 %}

## Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId %}


----------

## Contributors
- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe) and [Arkajyoti Basak](https://github.com/iamarkaj).
- Maintained by [Pedro Arias](https://github.com/pariaspe) and [Arkajyoti Basak](https://github.com/iamarkaj).
