---
permalink: /exercises/Drones/drone_cat_mouse
title: "Drone Cat and Mouse"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Drone Cat and Mouse"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/drone_cat_mouse/drone_cat_mouse.jpg
    image_path: /assets/images/exercises/drone_cat_mouse/drone_cat_mouse.jpg
    alt: "Drone Cat and Mouse."
    title: "Drone Cat and Mouse."

youtubeId: Hd2nhOx1tqI?t=510
youtubeId1: ykbw1kv6Cgw
youtubeId2: 0dV8OkTG0pM
youtubeId3: cPVsjWLAd_A
youtubeId4: Jj9ORzrbMdQ
---
## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to play a game of cat and mouse with a second quadrotor.

There are two drones in the same world. The mouse drone is preprogrammed and flies away from you. The cat drone is the one you program, and it has to chase the mouse down and catch it without crashing into anything.

The cat is never told where the mouse is. You only get the camera images from your own drone, so you have to find the mouse in the picture and chase what you can see. This makes it a real perception and pursuit problem rather than a "fly to these coordinates" problem.

{% include gallery caption="Gallery." %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RoboticsBackend and how to perform the exercises.

## Difficulty levels

The exercise comes with three worlds. They all use the same mouse program, but the mouse behaves differently in each one, so you can start simple and work your way up.

| World | What the mouse does | Time limit |
|---|---|---|
| Drone Cat Mouse Easy | Flies a straight line at a fixed height and ignores you completely | 30 s |
| Drone Cat Mouse Medium | Flies a lap and dodges sideways when you get close | 60 s |
| Drone Cat Mouse Hard | Uses height as well, dodges harder, and takes sharp turns around obstacles | 90 s |

Pick the world from the world selector before you press Play. Catch the mouse inside the time limit and you get a score based on how much of the clock was left. If the clock runs out, the mouse escaped.

The mouse drops to the ground once you catch it, so you can see clearly when the run is over.

## Robot API

* `import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `import WebGUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

### Sensors and drone state

* `HAL.get_position()` - Returns the actual position of the drone as a numpy array [x, y, z], in m.
* `HAL.get_velocity()` - Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s
* `HAL.get_yaw_rate()` - Returns the actual yaw rate of the drone, in rad/s.
* `HAL.get_orientation()` - Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad. 
* `HAL.get_roll()` - Returns the roll angle of the drone, in rad
* `HAL.get_pitch()` - Returns the pitch angle of the drone, in rad.
* `HAL.get_yaw()` - Returns the yaw angle of the drone, in rad. 
* `HAL.get_landed_state()` -  Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown. 

### About the mouse

* `HAL.get_mouse_position()` - Returns the real position of the mouse drone as [x, y, z].
* `HAL.is_caught()` - Returns True once you are close enough to the mouse to count as a catch.
* `HAL.CATCH_RADIUS` - How close you have to get for it to count, in m.

`HAL.get_mouse_position()` is there so you can check your tracking against the truth while you are debugging. It is not meant to be what you fly on. If your cat flies straight to those coordinates it will work, but you will have skipped the whole exercise.

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
* `HAL.IMG_WIDTH`, `HAL.IMG_HEIGHT` - The size of those images, in pixels. Useful for working out how far the mouse is from the centre of the picture.

### GUI

* `WebGUI.showImage(cv2_image)` - Shows a image of the camera  in the GUI
* `WebGUI.showLeftImage(cv2_image)` - Shows another image of the camera in the GUI

<!--## Theory
**Comming soon.**-->

## How to write your solution

The whole exercise comes down to three things, done over and over in a loop: find the mouse in the image, turn until it is in the middle of the image, and fly forward.

A reasonable place to start:

```python
import HAL
import WebGUI
import cv2
import numpy as np
import time

HAL.takeoff(3.0)

while True:
    image = HAL.get_frontal_image()

    # 1. find the mouse in the image
    # 2. work out how far off centre it is
    # 3. turn towards it and fly forward

    HAL.set_cmd_vel(0.0, 0.0, 0.0, 0.0)
    WebGUI.showImage(image)
    time.sleep(0.05)
```

### When you lose sight of it

You will lose the mouse. What you do in the next second decides whether you get it back.

Do not stop and spin on the spot. The last thing you saw is the best information you have: which side of the image it left from, and which way it was sliding when it went. Keep turning that way.

It also helps to remember that it kept moving while you worked out that it had gone, so aiming at the place it was last seen leaves your turn short. Project its movement a little further on and aim there.

If it has been gone long enough that this is stale, only then fall back to sweeping and looking around, and keep drifting forward while you do so you cover new ground.

## Hints

Simple hints provided to help you solve the drone_cat_mouse exercise. Please note that the **full solution has not been provided.**

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

Another possibility is to use **Nose Forward Mode**, where the drone follows the path similar to a fixed-wing aircraft. Then, to accomplish it, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

In this exercise, you should use the Nose Forward Mode in order to detect the cat drone.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Videos

### Multi-robot version (2026)

The exercise running on the three difficulty levels, with the cat chasing the mouse using only its camera.

{% include youtubePlayer.html id=page.youtubeId3 %}

This second video shows the multi-robot support the exercise is built on, with more than two robots sharing a single simulation.

{% include youtubePlayer.html id=page.youtubeId4 %}

### Earlier versions

{% include youtubePlayer.html id=page.youtubeId2 %}

#### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId %}

---------

## Contributors

- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe), [Arkajyoti Basak](https://github.com/iamarkaj) and [Anish Kumar](https://github.com/anishk85).
- Maintained by [Pedro Arias](https://github.com/pariaspe), [Arkajyoti Basak](https://github.com/iamarkaj) and [Anish Kumar](https://github.com/anishk85).

The exercise was originally created and maintained by the contributors above. During Google Summer of Code 2026, [Anish Kumar](https://github.com/anishk85) ported it to ROS 2, rebuilt it on top of the new multi-robot support in Robotics Academy so that both drones run as separate programs in one simulation, and added the three difficulty levels and the camera-only chase.
