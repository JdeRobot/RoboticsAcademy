---
permalink: /exercises/AutonomousCars/dynamic_window_approach
title: "Local navigation with DWA"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Dynamic Window Approach"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

youtubeId1: 0fsE49EijDc
---

## Goal

The objective of this exercise is to implement the Dynamic Window Approach (DWA) for local navigation.

The robot must:
- Generate a set of admissible linear and angular velocities
- Simulate trajectories for each velocity pair
- Evaluate them based on:
  - Distance to obstacles
  - Alignment with the target
  - Forward velocity
- Select the optimal velocity command

This allows the robot to reach the target while avoiding obstacles in a smooth and dynamically feasible way.

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
- `HAL.getPose3d().x` - to get the position of the robot (x coordinate).
- `HAL.getPose3d().y` - to obtain the position of the robot (y coordinate).
- `HAL.getPose3d().yaw` - to get the orientation of the robot with regard to the map.
- `HAL.getLaserData()` - to obtain laser sensor data. It is composed of 180 pairs of values: (0-180º distance in meters).
- `HAL.setV(velocity)` - to set the linear speed.
- `HAL.setW(velocity)` - to set the angular velocity.
- `HAL.getVelocity()` - returns the current real velocity of the robot as read from its odometry, `(v, w)`.
- `HAL.getDynamicWindowLimits(A_V, A_W, DT, V_MAX, W_MAX)` - computes the admissible velocity range around the current velocity, given the acceleration limits `A_V`, `A_W`, the time step `DT` and the maximum speeds `V_MAX`, `W_MAX`. Returns `(v_min, v_max, w_min, w_max)`. All arguments are optional and default to `A_V=3.0`, `A_W=3.0`, `DT=0.1`, `V_MAX=2.0`, `W_MAX=2.0`.
- `WebGUI.getNextTarget()` - to obtain the next target object on the scenario.
- `WebGUI.setTargetx(x)` - sets the x coordinate of the target on the WebGUI.
- `WebGUI.setTargety(y)` - sets the y coordinate of the target on the WebGUI.
- `WebGUI.showLocalTarget([x, y])` - displays the local target the robot is currently steering towards on the WebGUI.
- `WebGUI.showDynamicWindow(dynamic_window)` - displays the sampled velocity space on the WebGUI. `dynamic_window` is a list of `(v, w, score)` tuples.
- `WebGUI.showBestVelocity([v, w])` - displays the selected velocity command on the WebGUI.

To access the target 'x' and 'y' coordinates use (target is the object obtained from `WebGUI.getNextTarget()`):

- `target.getPose().x` - to obtain the x position of the target.
- `target.getPose().y` - to obtain the y position of the target.
- `target.isReached()` - returns `True` if the target has already been marked as reached.
- `target.setReached(True)` - marks the target as reached, so the next call to `WebGUI.getNextTarget()` returns the following one.

**Own API**

To simplify the exercise, the implementation of control points is offered.
To use it, only two actions must be carried out:

1. Obtain the following point:

   `currentTarget = WebGUI.getNextTarget()`

2. Mark it as visited when necessary:

   `currentTarget.setReached(True)`

As well as the destination that we have assigned:

```python
# Current target
target = [1.0, 1.0]
WebGUI.showLocalTarget(target)
```

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL::set_v(velocity);` - sets the linear velocity of the robot. The input is a `float`. Returns `void`.
- `HAL::set_w(velocity);` - sets the angular velocity of the robot. The input is a `float`. Returns `void`.
- `HAL::get_pose3d();` - returns the ground-truth robot pose as a `HAL::Pose3d`.
- `HAL::get_pose3d().x;` - gets the robot x position in world coordinates (`double`).
- `HAL::get_pose3d().y;` - gets the robot y position in world coordinates (`double`).
- `HAL::get_pose3d().yaw;` - gets the robot orientation around the vertical axis in world coordinates (`double`).
- `HAL::get_laser_data();` - returns the laser sensor data as a `HAL::LaserData`, composed of 180 pairs of values (0-180º distance in meters).
- `HAL::get_velocity();` - returns the current real velocity of the robot as read from its odometry, `std::pair<double, double>` `{v, w}`.
- `HAL::get_dynamic_window_limits(A_V, A_W, DT, V_MAX, W_MAX);` - computes the admissible velocity range around the current velocity. Returns a `std::tuple<double, double, double, double>` `{v_min, v_max, w_min, w_max}`. All arguments are optional `double` and default to `A_V=3.0`, `A_W=3.0`, `DT=0.1`, `V_MAX=2.0`, `W_MAX=2.0`.
- `WebGUI::get_next_target();` - returns a `std::shared_ptr<Target>` to the next target on the scenario.
- `WebGUI::set_target_x(x);` / `WebGUI::set_target_y(y);` - set the x/y coordinate of the target on the WebGUI. Input is `double`.
- `WebGUI::show_local_target({x, y});` - displays the local target the robot is currently steering towards on the WebGUI. Input is `std::array<double, 2>`.
- `WebGUI::show_dynamic_window(dw);` - displays the sampled velocity space on the WebGUI. `dw` is a `std::vector<std::array<double, 3>>` of `{v, w, score}`.
- `WebGUI::show_best_velocity({v, w});` - displays the selected velocity command on the WebGUI. Input is `std::array<double, 2>`.
- `WebGUI::mark_target_reached();` - marks the current target as reached, so the next call to `WebGUI::get_next_target()` returns the following one.

The `Target` object returned by `WebGUI::get_next_target()` exposes:

- `target->get_pose();` - returns the target pose as a `HAL::Pose3d` (use `.x` and `.y`).
- `target->is_reached();` - returns `bool`.
- `target->set_reached(true);` - marks the target as reached.

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

- `/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`

- `/odom` - Subscribe to this topic to receive the robot odometry. Both the pose (`x`, `y`, `yaw`) and the real velocity (`v`, `w`, from the twist fields) are available here. Message type: `nav_msgs/msg/Odometry`

- `/f1/laser/scan` - Subscribe to this topic to receive laser data. Message type: `sensor_msgs/msg/LaserScan`

The `/webgui/*` topics mirror the WebGUI debugging and target-tracking methods:

- `/webgui/local_target` - Publish to this topic to display the local target the robot is steering towards, equivalent to `showLocalTarget`. Message type: `geometry_msgs/msg/Point`.

- `/webgui/dynamic_window` - Publish to this topic to display the sampled velocity space, equivalent to `showDynamicWindow`. Message type: `std_msgs/msg/String`, containing a JSON array of `[v, w, score]` entries.

- `/webgui/best_velocity` - Publish to this topic to display the selected velocity command, equivalent to `showBestVelocity`. Message type: `std_msgs/msg/String`, containing a JSON array `[v, w]`.

- `/webgui/target_reached` - Publish `true` to this topic to mark the current target as reached and advance to the next one. Message type: `std_msgs/msg/Bool`.

- `/webgui/current_target` - Subscribe to this topic to receive the `(x, y)` coordinates of the target the robot must currently reach. Message type: `geometry_msgs/msg/Point`. QoS: `TRANSIENT_LOCAL`, depth `1`, so a late-joining node still receives the current target.

All other `/webgui/*` topics use the default QoS profile with a history depth of `10`.

The dynamic window limits are not exposed as a topic, since they only depend on quantities already available through `/odom`: a ROS 2-direct solution should compute `(v_min, v_max, w_min, w_max)` from the current `(v, w)` read from the odometry twist, following the same formula used by `HAL.getDynamicWindowLimits()`.

#### Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing debug information.

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

### Conversion of types

**Laser**

The following function parses laser data taking into account 1) laser only has 180º coverage and 2) the measure read at 90º corresponds to the 'front' of the robot.

You must apply the conversions needed to transform that laser data to a vector of the polar coordinates and a vector in the relative coordinate system of the robot.

```python
import math
import numpy as np

def parse_laser_data(laser_data):
    """ Parses the LaserData object and returns a tuple with two lists:
        1. List of  polar coordinates, with (distance, angle) tuples,
           where the angle is zero at the front of the robot and increases to the left.
        2. List of cartesian (x, y) coordinates, following the ref. system noted below.

        Note: The list of laser values MUST NOT BE EMPTY.
    """
    laser_polar = []  # Laser data in polar coordinates (dist, angle)
    laser_xy = []  # Laser data in cartesian coordinates (x, y)
    for i in range(180):
        # i contains the index of the laser ray, which starts at the robot's right
        # The laser has a resolution of 1 ray / degree
        #
        #                (i=90)
        #                 ^
        #                 |x
        #             y   |
        # (i=180)    <----R      (i=0)

        # Extract the distance at index i
        dist = laser_data.values[i]
        # The final angle is centered (zeroed) at the front of the robot.
        angle = math.radians(i - 90)
        laser_polar += [(dist, angle)]
        # Compute x, y coordinates from distance and angle
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        laser_xy += [(x, y)]
    return laser_polar, laser_xy

# Usage
laser_data = HAL.getLaserData()
if len(laser_data.values) > 0:
    laser_polar, laser_xy = parse_laser_data(laser_data)
```

**Coordinate system**

We have 2 different coordinate systems in this exercise.

- **Absolute coordinate system**: Its origin (0,0) is located in the finish line of the circuit (exactly where the F1 starts the lap).
- **Relative coordinate system**: It is the coordinate system solidary to the robot (F1). Positive values of X means 'forward', and positive values of Y means 'left'.

You can use the following code to convert absolute coordinates to relative ones (solidary to the F1).

```python
def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel, y_rel
```

## Theory

This exercise requires the implementation of a local navigation algorithm called the **Dynamic Window Approach (DWA)**.

### Navigation

Robot Navigation involves all the tasks required to move a robot from point A to point B **autonomously** without collisions.

The main subproblems are:

- **Localisation**: The robot needs to know where it is.
- **Collision Avoidance**: Detect and avoid obstacles.
- **Mapping**: Represent the environment.
- **Planning**: Compute a path to the goal.
- **Exploration**: Discover unknown areas.

Navigation is typically divided into global navigation, which computes a path using a map, and local navigation, which reacts to the environment in real time.

### Local Navigation

Local navigation adapts the robot motion based on sensor data and robot constraints.

Unlike methods based on artificial forces, the **Dynamic Window Approach (DWA)** operates directly in the **velocity space**, selecting the best motion command at each iteration.

### Dynamic Window Approach

The Dynamic Window Approach selects the optimal control command by evaluating possible velocities of the robot.

Instead of computing forces, the robot evaluates a set of candidate velocities `(v, w)`, simulates their resulting trajectories and selects the best one according to a cost function.

### Algorithm Steps

At each control cycle:

1. **Obtain Dynamic Window.** The valid velocity ranges are provided by the system:

   ```python
   v_min, v_max, w_min, w_max = HAL.getDynamicWindowLimits()
   ```

   These limits already take into account the current robot velocity, the maximum acceleration and the velocity constraints. **Note:** you are not required to implement this step.

2. **Sample Velocities.** Generate candidate velocity pairs `(v, w)` within the dynamic window.

3. **Simulate Trajectories.** For each candidate velocity, predict the robot motion over a short time horizon using the motion model:

   ```python
   x = x + v * cos(theta) * dt
   y = y + v * sin(theta) * dt
   theta = theta + w * dt
   ```

4. **Evaluate Trajectories.** Each trajectory is scored based on its heading, meaning the alignment with the goal, its clearance, meaning the distance to obstacles, and its velocity, meaning the forward speed.

5. **Select Best Command.** The velocity pair `(v, w)` with the highest score is applied to the robot.

### Cost Function

Each trajectory is evaluated using a weighted sum:

```python
score = alpha * heading + beta * clearance + gamma * velocity
```

Where **heading** measures how well the robot is oriented towards the goal, **clearance** measures the distance to the closest obstacle and **velocity** measures the forward speed. The weights `alpha`, `beta` and `gamma` determine the robot behavior.

### Debugging and Visualization

To help understand the behavior of the Dynamic Window Approach, the WebGUI provides visualization tools.

You can visualize the sampled velocity space:

```python
WebGUI.showDynamicWindow(dynamic_window)
```

Where `dynamic_window` contains the sampled velocity pairs `(v, w, score)`. This allows you to see which velocities are being considered at each iteration.

You can also visualize the selected control command:

```python
WebGUI.showBestVelocity(best_vw)
```

Where `best_vw` is the selected pair `(v, w)`. This helps to understand how the algorithm chooses the optimal motion.

Use these functions for debugging and tuning. They are especially useful to verify the velocity sampling, the cost function behavior and the stability of the controller.

### Advantages of DWA

DWA takes the robot dynamics into account, so it produces smooth and realistic motion, avoids the oscillations typical of force-based methods, and always selects velocity commands that the robot can actually reach.

## Hints

Visualizing the dynamic window can help detect issues such as:
- Poor sampling resolution
- Incorrect cost weighting
- Unsafe trajectory selection

When displaying the dynamic window in the WebGUI, it may be useful to **normalize the scores only for visualization purposes**.

- Do **not** normalize values when computing the real cost function.
- Use normalization only when sending data to the GUI.

This improves readability of the dynamic window representation, making it easier to compare candidate velocities visually.

For example, you can scale values to the range [0, 1]:

```python
normalized_value = (value - min_value) / (max_value - min_value)
```

## Videos

{% include youtubePlayer.html id=page.youtubeId1 %}

## Contributors

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Eduardo Perdices](eperdices@gsyc.es), [Francisco Pérez](https://github.com/fqez), Victor Arribas, [Julio Vega](julio.vega@urjc.es), [Jose María Cañas](https://gsyc.urjc.es/jmplaza/), [Nacho Arranz](https://github.com/igarag), [Javier Izquierdo](https://github.com/javizqh).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna), [Javier Izquierdo](https://github.com/javizqh).
