---
permalink: /exercises/AutonomousCars/f1_pursuit/
title: "F1 Pursuit"

sidebar:
    nav: "docs"

toc: true
toc_label: "TOC F1 Pursuit"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
    - url: /assets/images/exercises/f1_pursuit/simple_circuit.jpg
      image_path: /assets/images/exercises/f1_pursuit/simple_circuit.jpg
      alt: "Simple circuit."
      title: "Simple circuit."
    - url: /assets/images/exercises/f1_pursuit/montreal_circuit.jpg
      image_path: /assets/images/exercises/f1_pursuit/montreal_circuit.jpg
      alt: "Montreal circuit."
      title: "Montreal circuit."
    - url: /assets/images/exercises/f1_pursuit/montmelo_circuit.jpg
      image_path: /assets/images/exercises/f1_pursuit/montmelo_circuit.jpg
      alt: "Montmelo circuit."
      title: "Montmelo circuit."

gifs:
    - url: /assets/images/exercises/follow_line/oscillations.gif
      image_path: /assets/images/exercises/follow_line/oscillations.gif
      alt: "examples"
      title: "examples"
    - url: /assets/images/exercises/follow_line/slowresponse.gif
      image_path: /assets/images/exercises/follow_line/slowresponse.gif
      alt: "examples"
      title: "examples"

pid:
    - url: /assets/images/exercises/follow_line/ControlSystems.jpg
      image_path: assets/images/exercises/follow_line/ControlSystems.jpg
      alt: "Control Systems"
      title: "Control Systems"

    - url: /assets/images/exercises/follow_line/TypesofControlSystems.jpg
      image_path: /assets/images/exercises/follow_line/TypesofControlSystems.jpg
      alt: "Types of Control Systems"
      title: "Types of Control Systems"

    - url: /assets/images/exercises/follow_line/PID.png
      image_path: /assets/images/exercises/follow_line/PID.png
      alt: "PID"
      title: "PID"

---

## Goal

The goal of this exercise is to implement a PID reactive control capable of following the line painted on the racing circuit fast enough to catch a rival car.

{% include gallery caption="Gallery" %}

The students will program a Formula1 car in a race circuit to follow the red line in the middle of the road. A blue rival car starts half a lap ahead, on the second start line, and drives the same line on its own. The student has to catch it as fast as possible.

- The rival is caught when the student car gets within 1.5 m of it along the track. Both cars stop and the race is over.
- If the rival comes round and catches the student car from behind, the student is lapped and the race is over too.
- The race has three circuits (Simple, Montreal and Montmelo) and three levels each (Easy, Medium and Hard). The rival is faster on every level.
- The score is 100 at the start and drops to 0 at the par time, which is the time the rival needs for 2 laps. A later catch scores below 0 and being lapped scores -100.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instanciate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

#### Python

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL.getImage()` - to get the image (BGR8).
- `HAL.setV(velocity)` - to set the linear speed.
- `HAL.setW(velocity)` - to set the angular velocity.
- `HAL.isCaught()` - returns `True` once the rival has been caught. When the race is over the car stops and ignores new speed commands.
- `WebGUI.showImage(image)` - allows you to view a debug image or with relevant information.

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL::get_image();` - to get the image (cv::Mat).
- `HAL::set_v(velocity);` - to set the linear speed.
- `HAL::set_w(velocity);` - to set the angular velocity.
- `HAL::is_caught();` - returns `true` (bool) once the rival has been caught. When the race is over the car stops and ignores new speed commands.
- `WebGUI::show_image(image);` - allows you to view a debug image (cv::Mat) or with relevant information.

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

### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication with the simulation.

- `/f1/camera/image_raw` - Subscribe to this topic to receive camera images (BGR8). Message type: `sensor_msgs/msg/Image`
- `/f1/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`
- `/f1_pursuit/status` - Subscribe to this topic to receive the race status as a JSON string with the fields `state` (`racing`, `caught` or `lapped`), `level`, `elapsed`, `limit`, `gap` and `score`. Message type: `std_msgs/msg/String`. QoS: `TRANSIENT_LOCAL`, depth 1. The car is not stopped for you, so stop it when `state` is no longer `racing`.

For image debugging:

- `/webgui_image` - Publish to this topic to set the debug image in the GUI interface. Message type: `sensor_msgs/msg/Image`

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

## Theory

PID Control is the foundation of the implementation for this exercise. To understand PID Control, let us first understand what is Control in general.

### Control System

A system or set of devices, that manages, commands, directs or regulates the behavior of other devices or systems to achieve the desired results. Simply speaking, a system which controls other systems. Control Systems help a robot execute a set of commands precisely, in the presence of unforeseen errors or complications.

### Types of Control System

#### Open Loop Control System

A control system in which the control action is completely independent of the output of the system. An Open Loop System is a manual control system.

#### Closed Loop Control System

A control system in which the output has an effect on the input quantity in such a manner that the input will adjust itself based on the output generated. An open loop system can be converted to a closed one by providing feedback.

### PID Control

A control loop mechanism using feedback. A PID Controller continuously calculates an error value as the difference between the desired output and the current output and applies a correction based on proportional, integral and derivative terms(denoted by P, I, D respectively).

- **Proportional**

A Proportional Controller gives an output which is proportional to the current error. The error is multiplied with a proportionality constant to get the output. And hence, is 0 if the error is 0.

- **Integral**

An Integral Controller provides a necessary action to eliminate the offset error which is accumulated by the P Controller. It integrates the error over a period of time until the error value reaches zero.

- **Derivative**

A Derivative Controller gives an output depending on the rate of change or error regarding time. It gives the kick start for the output thereby increasing system response.

{% include gallery id="pid" caption="Control Systems and PID" %}

### Tuning Methods

In order for the PID equation to work, we need to determine the constants of the equation. There are 3 constants called the gains of the equation. We have 2 major tuning methods for this.

- **Trial and Error**

It is a simple method of PID controller tuning. While the system or controller is working, we can tune the controller. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches oscillating behavior. Once it is oscillating, we adjust Ki (Integral term) so that oscillations stops and finally adjust D to get a fast response.

- **Zeigler-Nichols method**

Zeigler-Nichols proposed closed loop methods for tuning the PID controller. Those are: continuous cycling method and damped oscillation method. The procedures for both methods are the same but their oscillation is different. Therefore, first we have to set the p-controller constant, Kp to a particular value while Ki and Kd values are zero. Proportional gain is increased until the system oscillates at a constant amplitude.

### Real Life Example

1. [https://accautomation.ca/tag/pid-control-car-analogy/](https://accautomation.ca/tag/pid-control-car-analogy/)
2. [https://www.youtube.com/watch?v=UR0hOmjaHp0](https://www.youtube.com/watch?v=UR0hOmjaHp0)

## Hints

Simple hints provided to help you solve the f1_pursuit exercise.

### References to ROS 2 Concepts

Understanding these ROS 2 concepts will help you implement the exercise natively. Refer to these links for more details:

1. ROS 2 Publisher & Subscriber – [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
2. ROS 2 Spin & Spin Once – [https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.html](https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.html)
 <!-- 3. ROS 2 Rate - add content for rate -->

### Detecting the Line to Follow

The first task of the assignment is to detect the line to be followed. This can be achieved easily by **filtering the color of the line** from the image and applying basic image processing to find the point or line to follow, or in Control terms our _Set Point_. Refer to these links for more information:

1. [https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/](https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/)
2. [https://stackoverflow.com/questions/10469235/opencv-apply-mask-to-a-color-image](https://stackoverflow.com/questions/10469235/opencv-apply-mask-to-a-color-image)
3. [https://stackoverflow.com/questions/22470902/understanding-moments-function-in-opencv](https://stackoverflow.com/questions/22470902/understanding-moments-function-in-opencv)

### Coding the Controller

The Controller can be designed with various configurations. 3 configurations have been described in detail below:

- **P Controller**
  The simplest way to do the assignment is using the P Controller. Just find the error which is the difference between our _Set Point_ (The point where our car should be heading) and the _Current Output_ (Where the car is actually heading). Keep adjusting the value of the constant, until we get a value where there occurs no [**unstable oscillations**](#Illustrations) and no [**slow response**](#Illustrations).

- **PD Controller**
  This is an interesting way to see the effect of the derivative on the Control. For this, we need to calculate the derivative of the output we are receiving. Since, we are dealing with _discrete outputs in our case, we simply calculate the difference between our previous error and the present error_, then adjust the proportional constant. Adjust this value along with the P gain to get a good result.

- **PID Controller**
  This is the completely implemented controller. Now, to add the I Controller we need to integrate the output from the point where error was zero, into the present output. While dealing with discrete outputs, we can achieve this using _accumulated error_. Then, comes the task of adjustment of gain constants until we get our desired result.

### Illustrations

{% include gallery id="gifs" caption="Unstable Oscillations (left) - Slow Response (right)" %}

## References

1. [https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/](https://www.electrical4u.com/control-system-closed-loop-open-loop-control-system/)
2. [https://en.wikipedia.org/wiki/PID_controller](https://en.wikipedia.org/wiki/PID_controller)
3. [https://www.elprocus.com/the-working-of-a-pid-controller/](https://www.elprocus.com/the-working-of-a-pid-controller/)
4. [https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm](https://www.tutorialspoint.com/control_systems/control_systems_introduction.htm)
5. [https://instrumentationtools.com/open-loop-and-closed-animation-loop/](https://instrumentationtools.com/open-loop-and-closed-animation-loop/)
6. [https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/](https://trinirobotics.com/2019/03/26/arduino-uno-robotics-part-2-pid-control/)
7. [http://homepages.math.uic.edu/~kauffman/DCalc.pdf](http://homepages.math.uic.edu/~kauffman/DCalc.pdf)
