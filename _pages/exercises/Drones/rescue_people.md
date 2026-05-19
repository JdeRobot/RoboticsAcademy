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
youtubeId5: pwtvTTDCi2I
---
## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to recognize the faces of lost people and save their locations in order to perform a subsequent rescue maneuver.

{% include gallery caption="Gallery." %}


### Orientative GPS Locations

Safety boat is located at **40º16'48.2" N**, **3º49'03.5" W**.
While survivors are known to be close to **40º16'47.23" N**, **3º49'01.78" W**.

## Frequency API

### Python

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instanciate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-native implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

#### Python

* `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

* `HAL.get_position()` - Returns the actual position of the drone as a numpy array [x, y, z], in m.
* `HAL.get_velocity()` - Returns the actual velocities of the drone as a numpy array [vx, vy, vz], in m/s.
* `HAL.get_yaw_rate()` - Returns the actual yaw rate of the drone, in rad/s.
* `HAL.get_orientation()` - Returns the actual roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw], in rad.
* `HAL.get_roll()` - Returns the roll angle of the drone, in rad
* `HAL.get_pitch()` - Returns the pitch angle of the drone, in rad.
* `HAL.get_yaw()` - Returns the yaw angle of the drone, in rad.
* `HAL.get_landed_state()` -  Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown.

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL::get_pose3d();` - Returns the current pose of the drone as a `HAL::Pose3d` struct with fields `x`, `y`, `z` (position in m), `yaw`, `pitch`, `roll` (orientation in rad) and `timeStamp`.
- `HAL::get_velocity();` - Returns the current velocity of the drone as a `HAL::Velocity3d` struct with fields `vx`, `vy`, `vz` (in m/s) and `yaw_rate` (in rad/s).
- `HAL::get_landed_state();` - Returns 1 if the drone is on the ground (landed), 2 if the drone is in the air and 4 if the drone is landing. 0 could be also returned if the drone landed state is unknown.
- `HAL::set_cmd_pos(x, y, z, az);` - Commands the *position* (x,y,z) of the drone, in m and the *yaw angle (az)* (in rad) taking as reference the first takeoff point (map frame).
- `HAL::set_cmd_vel(vx, vy, vz, az);` - Commands the *linear velocity* of the drone in the x, y and z directions (in m/s) and the *yaw rate (az)* (rad/s) in its body fixed frame.
- `HAL::set_cmd_mix(vx, vy, z, az);` - Commands the *linear velocity* of the drone in the x, y directions (in m/s), the *height* (z) related to the takeoff point and the *yaw rate (az)* (in rad/s).
- `HAL::takeoff(height);` - Takeoff at the current location, to the given height (in m).
- `HAL::land();` - Land at the current location.
- `HAL::get_frontal_image();` - Returns the latest image from the frontal camera as a cv::Mat.
- `HAL::get_ventral_image();` - Returns the latest image from the ventral camera as a cv::Mat.
- `WebGUI::show_right_image(image);` - Shows an image in the right panel of the WebGUI (cv::Mat).
- `WebGUI::show_left_image(image);` - Shows an image in the left panel of the WebGUI (cv::Mat).

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"

void exercise() {
    Frequency freq = Frequency();
    // Enter sequential code!

    while (true)
    {
        // Enter iterative code!
        freq.tick();


    }
}
```

### Actuators and drone control

The three following drone control functions are *non-blocking*, i.e. each time you send a new command to the aircraft it immediately discards the previous control command.

#### 1. Position control

* `HAL.set_cmd_pos(x, y, z, az)` - Commands the *position* (x,y,z) of the drone, in m and the *yaw angle (az)* (in rad) taking as reference the first takeoff point (map frame)

#### 2. Velocity control

* `HAL.set_cmd_vel(vx, vy, vz, az)` - Commands the *linear velocity* of the drone in the x, y and z directions (in m/s) and the *yaw rate (az)* (rad/s) in its body fixed frame

#### 3. Mixed control

* `HAL.set_cmd_mix(vx, vy, z, az)` - Commands the *linear velocity* of the drone in the x, y directions (in m/s), the *height* (z) related to the takeoff point and the *yaw rate (az)* (in rad/s)

### Drone takeoff and land

Besides using the buttons at the drone teleoperator WebGUI, taking off and landing can also be controlled from the following commands in your code:

* `HAL.takeoff(height)` - Takeoff at the current location, to the given height (in m)
* `HAL.land()` - Land at the current location.

### Drone cameras

* `HAL.get_frontal_image()` - Returns the latest image from the frontal camera as a OpenCV cv2_image
* `HAL.get_ventral_image()` - Returns the latest image from the ventral camera as a OpenCV cv2_image

### WebGUI

* `WebGUI.showImage(cv2_image)` - Shows a image of the camera  in the WebGUI
* `WebGUI.showLeftImage(cv2_image)` - Shows another image of the camera in the WebGUI

### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication with the simulation.

This exercise uses Aerostack2, so the ROS 2-direct version is more advanced than in ground robots. For more information about [Aerostack 2](https://aerostack2.github.io/)

The drone namespace is `/drone0`.

- `/drone0/frontal_cam/image_raw` - Subscribe to this topic to receive the frontal camera image. Message type: `sensor_msgs/msg/Image`

- `/drone0/ventral_cam/image_raw` - Subscribe to this topic to receive the ventral camera image. Message type: `sensor_msgs/msg/Image`  

- `/drone0/self_localization/twist` - Subscribe to this topic to receive the drone twist, including yaw rate. Message type: `geometry_msgs/msg/TwistStamped`  

- `/drone0/motion_reference/pose` - Publish to this topic to send position references with orientation. Message type: `geometry_msgs/msg/PoseStamped`  

- `/drone0/motion_reference/twist` - Publish to this topic to send velocity references. Message type: `geometry_msgs/msg/TwistStamped`  

- `/drone0/platform/info` - Subscribe to this topic to receive the platform state information. Message type: `as2_msgs/msg/PlatformInfo`  

- `/drone0/platform/state_machine_event` - Service used for takeoff and landing state transitions. Service type: `as2_msgs/srv/SetPlatformStateMachineEvent`

For image debugging:

- `/webgui/image_debug_right` - Publish to this topic to display a debug image in the right panel of the WebGUI. Message type: `sensor_msgs/msg/Image`

- `/webgui/image_debug_left` - Publish to this topic to display a debug image in the left panel of the WebGUI. Message type: `sensor_msgs/msg/Image`

#### Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

#### C++

In order to use direct ros controls you must include the following lines:

```cpp
#ifndef USER_NODE
#define USER_NODE

#include "rclcpp/rclcpp.hpp"

class UserNode : public rclcpp::Node {
  // Your class
};

#endif
```

You must define `USER_NODE` and a `UserNode` node class.

To have frequency control you may use a timer and a control function as follows:

```cpp
  UserNode() : Node("user_node")
  {
    // More subscribers and publishers
    timer_ = create_wall_timer(100ms, std::bind(&UserNode::control_cycle, this));
  };

// More Code

  void control_cycle(){
    // Your function
  };
```

## Hints

Simple hints provided to help you solve the rescue_people exercise. Please note that the **full solution has not been provided.**

### Face detection

You can use the Face Detection using Haar Cascades from opencv [[1]](https://docs.opencv.org/4.5.0/db/d28/tutorial_cascade_classifier.html).

It can be found in this location: `/resources/exercises/rescue_people/haarcascade_frontalface_default.xml`.

### Does the drone API understand GPS Locations?

No, it doesn't. But don't worry, you can easily convert them to UTM locations which are in meters. There are several open converters that can make the job for you [[2]](http://rcn.montana.edu/Resources/Converter.aspx).

**FYI**: A Quick Guide to Using UTM Coordinates [[3]](https://www.maptools.com/tutorials/utm/quick_guide).

### Coordinates Frames

* **UTM**: Easting (+x), North (+y).
* **Drone Position Control**: North (+x), West (+y), Height (+z).
* **Drone Velocity Control**: Forward (+x), Left (+y), Up (+z).

Also notice that UTM coordinates are global, while drone positions are local, refered to its initial position. Velocities are body related.

### Directional control. How should drone yaw be handled?

If you don't take care of the drone yaw angle or yaw_rate in your code (keeping them always equal to zero), you will fly in what's generally called **Heads Free Mode**. The drone will always face towards its initial orientation, and it will fly sideways or even backwards when commanded towards a target destination. Multi-rotors can easily do that, but what's not the best way of flying a drone.

Another possibility is to use **Nose Forward Mode**, where the drone follows the path similar to a fixed-wing aircraft. Then, to accomplish th exercise's goal, you'll have to implement by yourself some kind of directional control, to rotate the nose of your drone left or right using yaw angle, or yaw_rate.

In this exercise, you can use whichever you prefer.

### Do I need to know when the drone is in the air?

No, you can solve this exercise without taking care of the **land state** of the drone. However, it could be a great enhancement to your blocking position control function if you make it only work when the drone is actually flying, not on the ground.

## Videos

{% include youtubePlayer.html id=page.youtubeId4 %}

### Demonstrative video of the solution

{% include youtubePlayer.html id=page.youtubeId5 %}

-------

## Contributors

* Contributors: [Nikhil Khedekar](https://github.com/nkhedekar), [JoseMaria Cañas](https://github.com/jmplaza), [Diego Martín](https://github.com/diegomrt), [Pedro Arias](https://github.com/pariaspe), [Arkajyoti Basak](https://github.com/iamarkaj) and [Pawan Wadhwani](https://github.com/pawanw17).
* Maintained by [Pedro Arias](https://github.com/pariaspe) and [Pawan Wadhwani](https://github.com/pawanw17).
