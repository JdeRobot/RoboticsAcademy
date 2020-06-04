---
permalink: /exercises/Drones/drone_gymkhana
title: "Drone Gymkhana"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Drone Gymkhana"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/drone_gymkhana/drone_gymkhana_complete.png
    image_path: /assets/images/exercises/drone_gymkhana/drone_gymkhana_complete.png
    alt: "Drone Gymkhana"
    title: "Drone Gymkhana"
  - url: /assets/images/exercises/drone_gymkhana/drone_detail.png
    image_path: /assets/images/exercises/drone_gymkhana/drone_detail.png
    alt: "Drone detail"
    title: "Drone detail"

drone_teleop:
  - url: /assets/images/exercises/drone_gymkhana/drone_teleop_vel.png
    image_path: /assets/images/exercises/drone_gymkhana/drone_teleop_vel.png
    alt: "Drone Teleoperator"
    title: "Drone Teleoperator"

youtubeId: Guq3nPIQmdM
---

To goal of this exercise is to learn how to control a drone to complete a gymkhana course, composed of several waypoints that you'll have to navigate through. 

{% include gallery caption="Gymkhana course. 3DR Iris drone it its launch pad" %}

You will have contact with the drone infrastructure in our Robotics Academy for the first time, learning how to access the drone sensors, actuators and cameras via an *Application Programing Interface* (API), which is a key point to master the rest of the proposed exercises.

## Installation
First, make sure you have installed the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.

Second, install the specific infrastructure for working with drones in the Academy, which includes the [PX4 open source flight controller platform](http://jderobot.github.io/RoboticsAcademy/installation/#px4)  and [MAVROS](http://jderobot.github.io/RoboticsAcademy/installation/#mavros), a MAVLink-based communication node between PX4 and ROS.  

And finally, install our ROS metapackage for controlling drones, called `jderobot-drones`. It contains a  teleoperator GUI, the drone HAL-API and some other useful stuff for running these drone exercises:

```bash
sudo apt-get install ros-melodic-jderobot-drones
```

## How to run the exercise

To launch the exercise, open a terminal window, navigate to the drone_gymkhana exercise folder and execute the following roslaunch command:

```bash
roslaunch drone_gymkhana.launch
```

Two different windows will pop up:

- The **Gazebo simulator**, showing the 3DR IRIS quadcopter in the central takeoff (green) pad, the gymkhana course and the landing pad, in red. 
- The **Drone teleoperator**, a GUI which provides the following functionalities:
  - *Buttons* to takeoff, land and stop the drone.
  - A button to *play and stop* your own programming code, together with an status messages window.
  - A *drone teleoperator* (top, left) mimicking a RC transmitter configured in Mode 2, which shows the commanded velocities. 
  - An *info box (bottom, left)* showing the actual position (m), linear velocities (m/s) and angular yaw rate (rad/s) of the drone, and the corresponding ROS frames for each data.
  - An *extra sensors* window composed by graphical attitude, altitude and velocity indicators (click in Sensors)

{% include gallery id="drone_teleop" caption="Drone teleoperator GUI" %}

## How should I solve the exercise?

To solve the exercise, you must edit the `my_solution.py` file and insert the control logic into it. In this drone_gymkhana exercise, your code has to be entered in:

1. The `position_control` function, where you have to program a *blocking position control*, so each time you call this function the drone has to reach a given target position before accepting more control commands. Please have a look a the [Hints section](#hints) to get some help about how to deal with this.  
2. The `execute` function, where you can define all waypoints of the gymkhana course and call iteratively the `position control` function. 

```python
####### Build your position control function here ######
def position_control():
	global drone
	# Insert your code here
 
########################################################

def execute(event):
	global drone
	############## Insert your code here ###############
	# Waypoint list  
	waypoint_list = []
	
	# Navigation using position control
	for waypoint in waypoint_list:
	    position_control()
        
	####################################################
```

You must assume your drone has already took off from the central green pad and its already airbone (i.e. you already pressed the Takeoff button). Then your drone has to **navigate autonomously** to complete the gymkana course following the numbered cubes (1, 2, 3 and 4). It has to **pierce all red obstacles** present at each course leg without any collision. Finally, it hast to go to the landing red pad and land on it. 

**To remember:** *At the moment, each time you update your code you must to run again the launch file in order to insert the updated code in the drone teleoperator GUI*

## API

### Sensors and drone state

* `drone.get_position()` - Returns the actual position of the drone as a numpy array [x, y, z], in m.
* `drone.get_velocity()` - Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s
* `drone.get_yaw_rate()` - Returns the actual yaw rate of the drone, in rad/s.
* `drone.get_orientation()` - Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad. 
* `drone.get_roll()` - Returns the roll angle of the drone, in rad
* `drone.get_pitch()` - Returns the pitch angle of the drone, in rad.
* `drone.get_yaw()` - Returns the yaw angle of the drone, in rad. 
* `drone.get_landed_state()` -  Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown. 

### Actuators and drone control

The three following drone control functions are *non-blocking*, i.e. each time you send a new command to the aircraft it immediately discards the previous control command. 

### 1. Position control

* `drone.set_cmd_pos(x, y, z, yaw)` - Commands the *position* (x,y,z) of the drone, in m and the *yaw angle* (in rad) taking as reference the first takeoff point (map frame)

### 2. Velocity control

* `drone.set_cmd_vel(vx, vy, vz, yaw_rate)` - Commands the *linear velocity* of the drone in the x, y and z directions (in m/s) and the *yaw rate* (rad/s) in its body fixed frame

### 3. Mixed control

* `drone.set_cmd_mix(vx, vy, z, yaw_rate)` - Commands the *linear velocity* of the drone in the x, y directions (in m/s), the *height* (z) related to the takeoff point and the *yaw rate* (in rad/s) 

### Drone takeoff and land

Besides using the buttons at the drone teleoperator GUI, taking off and landing can also be controlled from the following commands in your code:

* `drone.takeoff(height)` - Takeoff at the current location, to the given height (in m)
* `drone.land()` - Land at the current location. 

### Drone cameras (not used in this exercise)

* `drone.get_frontal_image()` - Returns the latest image from the frontal camera as a OpenCV cv2_image
* `drone.get_ventral_image()` - Returns the latest image from the ventral camera as a OpenCV cv2_image
* `set_image_filtered(cv2_image)` - Shows a filtered image of the camera images in the GUI
* `set_image_threshed(cv2_image)` - Shows a thresholded image in the GUI

## Hints

### How can I build a blocking position control?

There are several ways of solving this exercise, that differ in the approach and complexity when building the blocking position control function:

1. **Time-based open loop control:** The easiest alternative, recommended just for testing purposes. If you command the drone using the non-blocking API function `drone.set_cmd_pos(x, y, z, yaw)` and wait long enough (using, for example, `rospy.sleep(time_in_sec)`), the drone will surely reach its destination. However, this is not recommended, as we don't get confirmation of arrival at the given target location.
2. **Closed loop position control**: Based on getting the current drone position, comparing it with the target position, and executing the position or velocity API control functions till the geometric distance between current and target 3D positions is within certain *tolerance*  (which can also be passed to the blocking as input parameter)

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

In this exercise, we want you to try programming your drone to travel to the target destinations similarly to how a fixed-wing aircraft would do, namely **nose forward**.  Then, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

If you know your current position and your target one, you can easily compute the direction (yaw angle) the drone must be turned to by applying some elementary geometry. Probably both `math.sqrt()` and `math.atan2()` Python functions will be very useful for you here. 

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId %}

---------

## Contributors

- Contributors: [Pedro Arias](https://github.com/pariaspe), [Diego Martín](https://github.com/diegomrt) and [JoseMaria Cañas](https://github.com/jmplaza).
- Maintained by [Pedro Arias](https://github.com/pariaspe) and [Diego Martín](https://github.com/diegomrt) 
