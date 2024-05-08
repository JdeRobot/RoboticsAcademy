---
permalink: /exercises/Drones/rescue_people
title: "Rescue People"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Rescue People"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/rescue_people/rescue_people.png
    image_path: /assets/images/exercises/rescue_people/rescue_people.png
    alt: "Rescue People."
    title: "Rescue People."

youtubeId: c0heCUeDm7I
youtubeId2: ee_T46fpHN8
youtubeId3: fISm9Q2_ogg
youtubeId4: O8atiuEamp4
---
## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to recognize the faces of lost people and save their locations in order to perform a subsequent rescue maneuver.

{% include gallery caption="Gallery." %}

**Note**: If you haven't, take a look at the [user guide](https://jderobot.github.io/RoboticsAcademy/user_guide/#installation) to understand how the installation is made, how to launch a RADI and how to perform the exercises.

### Orientative GPS Locations

Safety boat is located at **40º16'48.2" N**, **3º49'03.5" W**.
While survivors are known to be close to **40º16'47.23" N**, **3º49'01.78" W**.

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

Simple hints provided to help you solve the rescue_people exercise. Please note that the **full solution has not been provided.**

### Face detection

You can use the Face Detection using Haar Cascades from opencv [[1]](https://docs.opencv.org/4.5.0/db/d28/tutorial_cascade_classifier.html).

### Does the drone API understand GPS Locations?

No, it doesn't. But don't worry, you can easily convert them to UTM locations which are in meters. There are several open converters that can make the job for you [[2]](http://rcn.montana.edu/Resources/Converter.aspx).

**FYI**: A Quick Guide to Using UTM Coordinates [[3]](https://www.maptools.com/tutorials/utm/quick_guide).

### Coordinates Frames

- **UTM**: Easting (+x), North (+y).
- **Drone Position Control**: North (+x), West (+y), Height (+z).
- **Drone Velocity Control**: Forward (+x), Left (+y), Up (+z).

Also notice that UTM coordinates are global, while drone positions are local, refered to its initial position. Velocities are body related.

### Directional control. How should drone yaw be handled? 

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

Another possibility is to use **Nose Forward Mode**, where the drone follows the path similar to a fixed-wing aircraft. Then, to accomplish it, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate. 

In this exercise, you can use the one you prefer.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Videos

{% include youtubePlayer.html id=page.youtubeId4 %}

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId3 %}

-------

## Contributors

- Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe), [Arkajyoti Basak](https://github.com/iamarkaj) and [Pawan Wadhwani](https://github.com/pawanw17).
- Maintained by [Pedro Arias](https://github.com/pariaspe) and [Pawan Wadhwani](https://github.com/pawanw17).
