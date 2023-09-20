---
permalink: /exercises/Drones/drone_tello
title: "Real Drone Tello Square"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Drone Tello"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/drone_tello/drone_tello1.png
    image_path: /assets/images/exercises/drone_tello/drone_tello1.png
    alt: "Drone Tello."
    title: "Drone Tello."

---
## Goal

The goal of this exercise is to implement the logic to make the tello drone be able to take off, draw a square with its flight and land back in the same place from which it took off.

<!-- <img src="/RoboticsAcademy/assets/images/exercises/drone_tello/drone_tello1.png" width="100%" height="60%"> -->

{% include gallery caption="Gallery." %}

## Instructions
This is the preferred way for running the exercise.

### Installing and Launching
1. Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

2. Pull the current distribution of Robotics Academy Docker Image:

	```bash
  docker pull jderobot/robotics-academy:latest
  ```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RADI (Robotics-Academy Docker Image) can be found [here](https://hub.docker.com/r/jderobot/robotics-academy/tags).

### How to perform the exercises?
- Start a new docker container of the image and keep it running in the background:

	```bash
  docker run --name $name $device -e DRI_NAME=$device_name -v $route:/home/shared_dir --rm -it -p 8889:8889/udp -p 38065:38065/udp -p 8890:8890/udp -p 11111:11111/udp -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 6081:6081 -p 1108:1108 -p 6082:6082 -p 7163:7163 jderobot/robotics-academy
  ```

- On the local machine navigate to 127.0.0.1:7164/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

- The exercise can be used after the alert.

### Enable GPU Acceleration
- Follow the advanced launching instructions from [here](https://jderobot.github.io/RoboticsAcademy/user_guide/#enable-gpu-acceleration).

### Optional: Store terminal output
- To store the terminal output of manager.py and launch.py to a file execute the following docker run command and keep it running in the background:
```bash
docker run --name $name $device -e DRI_NAME=$device_name -v $route:/home/shared_dir --rm -it -p 8889:8889/udp -p 38065:38065/udp -p 8890:8890/udp -p 11111:11111/udp -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 6081:6081 -p 1108:1108 -p 6082:6082 -p 7163:7163 jderobot/robotics-academy --logs
```

- The log files will be stored inside `$HOME/.roboticsacademy/{year-month-date-hours-mins}/`. After the session, use `more` to view the logs, for example:
```bash
more $HOME/.roboticsacademy/log/2021-11-06-14-45/manager.log
```

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

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student can use the `print()` command in the Editor. 

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

### Sensors and drone state

* `HAL.getImage()` - The user receives the last image perceived by the Tello drone's built-in camera.
* `HAL.takeoff()` - Causes the Tello to take off at a predetermined height of one meter.
* `HAL.land()` - Causes the Tello to land from the place where it is.
* `HAL.pause()` - Causes the Tello to stop at the place where it receives the call, restarting speeds it had previously commanded.
* `HAL.turn_left(degrees)` - Tello will turn __degrees__ counterclockwise.
* `HAL.turn_right(degrees)` - Tello will turn __degrees__ clockwise.
* `HAL.forward(distance)` - Tello will move __distance__ (measured in cm) forward
* `HAL.up(distance)` - Tello will move __distance__ (measured in cm) upward
* `HAL.left(distance)` - Tello will move __distance__ (measured in cm) to the left
* `HAL.right(distance)` - Tello will move __distance__ (measured in cm) to the right
* `HAL.back(distance)` - Tello will move __distance__ (measured in cm) backward
* `HAL.setVX` - Tello will move __velocity__ (measured in cm/s) in X
* `HAL.setVY` - Tello will move __velocity__ (measured in cm/s) in Y
* `HAL.setVZ` - Tello will move __velocity__ (measured in cm/s) in Z
* `HAL.setW` - Tello will turn __wvelocity__ (measured in cm/s) clockwise

### Actuators and drone control

The three following drone control functions are *non-blocking*, i.e. each time you send a new command to the aircraft it immediately discards the previous control command. 

### Theory 

The main principle that allows a drone to fly is related to aerodynamics, specifically, a principle known as Bernoulli's Principle.

Bernoulli's Principle explains how the movement of air over a surface can generate lift. This principle is essential for understanding how drones, airplanes, and other types of aircraft can fly.

In a drone, the rotors or propellers play a crucial role in generating lift. When the drone's motors spin the rotors, this causes air to be pushed downwards, creating an upward force known as lift. This lift needs to be greater than the drone's weight for it to ascend.

There are also four primary forces acting on a drone: lift, weight (or gravity), thrust, and drag.

    Lift is the upward force generated by the drone's propellers.
    Weight, or gravity, is the downward force pulling the drone towards the ground.
    Thrust is the force that propels the drone forward.
    Drag is the resistance encountered by the drone as it moves through the air.

By managing these four forces, a drone can lift off, hover, and move in any direction. The drone's on-board computer systems and sensors help to balance these forces and control the drone's flight.

Additionally, drones utilize a method known as differential thrust for maneuvering and stability. This involves varying the speed of each rotor to make the drone pitch (tilt forward and backward), roll (tilt side to side), or yaw (rotate around a vertical axis).

### Drone cameras

* You will be able to see the drone's front camera at all times while it operates.

## Hints

Simple hints provided to help you solve the drone_tello exercise. Please note that the **full solution has not been provided.**

### Take off before performing any action
The drone will not perform any action provided to it if it has not taken off before, please note that the take-off takes a couple of seconds before it is fully completed.

### Warnings

### Drone inaccuracy

Keep in mind that the drone stays in a still place thanks to the sensor underneath it. It uses an infrared sensor, where depending on the ground it bounces on it can get noise. It can also happen that the 90 degrees that it rotates are not exactly 90 degrees due to the motors, propellers or battery of the drone.

### Drone connection

The connection with the Tello works because it creates a wifi network to which the user connects from his machine. This connection can be unstable due to external factors.
Make sure you are connected to the Tello network before running the exercise.

### Drone emergency

In case you notice that the behavior of the drone you have programmed may lead to a crash, remember that you have the HAL.emergency() function that stops the drone motors immediately. Keep this function in mind to avoid possible accidents with the drone.

---------

## Contributors

- Contributors: [Guillermo Bernal](https://github.com/gbernalr).
