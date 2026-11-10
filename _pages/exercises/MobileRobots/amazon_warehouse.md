---
permalink: /exercises/MobileRobots/amazon_warehouse/
title: "Amazon Warehouse"

sidebar:
    nav: "docs"

toc: true
toc_label: "TOC Amazon Warehouse"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
    - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_teaser.png
      image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_teaser.png
      alt: "Shelves to move"
      title: "Shelves to move"
    - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
      image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse1_robot.png
      alt: "Robot"
      title: "Robot"

holonomicrobot:
    - url: /assets/images/exercises/amazon_warehouse/holonomico_robot.png
      image_path: /assets/images/exercises/amazon_warehouse/holonomico_robot.png
      alt: "Holonomic robot"
      title: "Holonomic robot"

ackermannrobot:
    - url: /assets/images/exercises/amazon_warehouse/ackermann_robot.png
      image_path: /assets/images/exercises/amazon_warehouse/ackermann_robot.png
      alt: "Ackermann robot"
      title: "Ackermann robot"

ackermannrobotmesh:
    - url: /assets/images/exercises/amazon_warehouse/ackermann_robot_dae.png
      image_path: /assets/images/exercises/amazon_warehouse/ackermann_robot_dae.png
      alt: "Ackermann robot mesh"
      title: "Ackermann robot mesh"
    - url: /assets/images/exercises/amazon_warehouse/ackemann_robot_shelf_dae.png
      image_path: /assets/images/exercises/amazon_warehouse/ackemann_robot_shelf_dae.png
      alt: "Ackermann robot shelf mesh"
      title: "Ackermann robot shelf mesh"

warehouse1:
    - url: /assets/images/exercises/amazon_warehouse/warehouse1.png
      image_path: /assets/images/exercises/amazon_warehouse/warehouse1.png
      alt: "Ackermann logistics robot"
      title: "Ackermann logistics robot"

warehouse1mesh:
    - url: /assets/images/exercises/amazon_warehouse/warehouse1_dae.png
      image_path: /assets/images/exercises/amazon_warehouse/warehouse1_dae.png
      alt: "Warehouse 1 mesh"
      title: "Warehouse 1 mesh"
    - url: /assets/images/exercises/amazon_warehouse/shelf_dae.png
      image_path: /assets/images/exercises/amazon_warehouse/shelf_dae.png
      alt: "Shelf mesh"
      title: "Shelf mesh"

warehouse2:
    - url: /assets/images/exercises/amazon_warehouse/warehouse2.png
      image_path: /assets/images/exercises/amazon_warehouse/warehouse2.png
      alt: "Warehouse 2"
      title: "Warehouse 2"

ompl:
    - url: /assets/images/exercises/amazon_warehouse/OMPL_structure.png
      image_path: /assets/images/exercises/amazon_warehouse/OMPL_structure.png
      alt: "OMPL structure"
      title: "OMPL structure"

example:
    - url: /assets/images/exercises/amazon_warehouse/example_plot.png
      image_path: /assets/images/exercises/amazon_warehouse/example_plot.png
      alt: "Example"
      title: "Example"

original_axis:
    - url: /assets/images/exercises/amazon_warehouse/original_axis.JPG
      alt: "Warehouse 1"
      image_path: /assets/images/exercises/amazon_warehouse/original_axis.JPG
      title: "Warehouse 1"

eucliean_eg:
    - url: /assets/images/exercises/amazon_warehouse/euclidean_solution_validity_eg.png
      alt: "Validity checker"
      image_path: /assets/images/exercises/amazon_warehouse/euclidean_solution_validity_eg.png
      title: "Validity checker"
    - url: /assets/images/exercises/amazon_warehouse/euclidean_solution_eg.png
      alt: "Found path"
      image_path: /assets/images/exercises/amazon_warehouse/euclidean_solution_eg.png
      title: "Found path"

geometry_eg:
    - url: /assets/images/exercises/amazon_warehouse/geometry_solution_eg.png
      alt: "Path planning"
      image_path: /assets/images/exercises/amazon_warehouse/geometry_solution_eg.png
      title: "Path planning"

gazebo_move:
    - url: /assets/images/exercises/amazon_warehouse/gazebo_move_tutorial.png
      image_path: /assets/images/exercises/amazon_warehouse/gazebo_move_tutorial.png
      alt: "Moving the robot in Gazebo Harmonic"
      title: "Moving the robot in Gazebo Harmonic"

youtubeId1: EVt9vYqEoDg
youtubeId2: -2D90I-wZKs
youtubeId3: 0PO4S8Omn30
youtubeId4: xsitnQkU8Os
ackermannRobot: NCC9bn-v_Ro
robotgeometry: FPPF27QIRHw
---

## Goal

The objective of this exercise is to implement the logic that allows a logistics robot to deliver shelves to the required place by making use of the location of the robot. The robot is equipped with a map and knows its current location in it. The main objective will be to find the shortest path to complete the task.

{% include gallery caption="Gallery" %}

The exercise offers two kinds of robot. Choose the world in the exercise page and read the section of its robot.

- **Logistic robots**, holonomic or Ackermann, with a lifting platform to carry the shelves, in **Warehouse 1** and **Warehouse 2**. Their API is in [Robot API](#robot-api) and the details of the robots and warehouses in [Supporting information](#supporting-information).
- [**MiR100**](#mir100), a differential drive robot with two lasers and no lifting platform, simulated in **Warehouse 1 Mir100** or real in **Mir100 Physical**, see [Running on the real robots](#running-on-the-real-robots).

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

| World | Robot | Python | C++ |
| ----- | ----- | ------ | --- |
| Warehouse 1, Warehouse 2 | Logistic robots | `import HAL` | `#include "HAL.hpp"` |
| Warehouse 1 Mir100, Mir100 Physical | [MiR100](#mir100) | `import HAL_mir100 as HAL` | Not available |

This section is about the logistic robots, the MiR100 API is in [its section](#mir100).

### HAL-based Implementation

#### Python

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

- `HAL.getPose3d()` - returns x, y and theta components of the robot in world coordinates. The function returns an x,y based in this axis reference, with (0,0) next to the robot spawn in Warehouse 1:

{% include gallery id="original_axis" caption="HAL.getPose3d()" %}

- `HAL.getSimTime()` - returns simulation time.

```python
# simulation time in second
sec = HAL.getSimTime().sec + HAL.getSimTime().nanosec / 1000000000
```

- `HAL.setV(velocity)` - to set the linear speed.
- `HAL.setW(velocity)` - to set the angular speed.
- `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values (0-180º, distance in meters).
- `HAL.lift()` - to lift the platform.
- `HAL.putdown()` - to put down the platform.
- `HAL.getLiftState()` - Returns the current lift state (`True` if lifted).
- `WebGUI.showPath(array)` - shows a path on the map. The parameter should be a 2D array containing each of the points of the path.
- `WebGUI.getMap(url)` - returns a numpy array with the image data in a 3 dimensional array (R, G, B) of values between 0-1. The URLs of the worlds are in the **Supporting information** section.
- `WebGUI.showNumpy(mat)` - Displays the matrix sent. Accepts an uint8 numpy matrix, values ranging from 0 to 127 for grayscale and values 128 to 134 for predetermined colors (128 = red; 129 = orange; 130 = yellow; 131 = green; 132 = blue; 133 = indigo; 134 = violet).

To reset the map to the original state you can use the next function with **orig_map** being the original map:

```python
def reset_map(orig_map):
    rows, cols, _ = orig_map.shape
    old_map = np.zeros((rows, cols), dtype=np.uint8)
    for i in range(rows):
        for j in range(cols):
            old_map[i, j] = np.average(orig_map[i, j]) * 127

    WebGUI.showNumpy(old_map)
```

To draw more complex shapes you can use the following opencv2 functions with mat being the matrix you will use to call `WebGUI.showNumpy(mat)`:

- Draw a line: `cv2.line(mat, (start_x, start_y), (end_x, end_y), color, thickness)`
- Draw a circle: `cv2.circle(mat, (center_x, center_y), radius, color, thickness)`
- Draw a rectangle: `cv2.rectangle(mat, (start_x, start_y), (end_x, end_y), color, thickness)`
- Write text: `image = cv2.putText(mat, text, (start_x, start_y), font, fontScale, color, thickness, cv2.LINE_AA)`

See the example below on how to use them:

```python
    rows, cols, _ = orig_map.shape
    mat = np.zeros((rows, cols), dtype=np.uint8)

    cv2.circle(mat, (100, 100), 40, 50, 5)
    cv2.line(mat, (100, 100), (200, 200), 129, 2)
    cv2.rectangle(mat, (100, 100), (200, 200), 128, 2)
    cv2.putText(mat, "Text", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, 129, 2, cv2.LINE_AA)
    WebGUI.showNumpy(mat)
```

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- `HAL::get_pose3d();` - Returns the current pose as a `HAL::Pose3d` struct with fields `x`, `y` (in m) and `yaw` (in rad).
- `HAL::get_sim_time();` - Returns the simulation time in seconds as a `double`.
- `HAL::get_laser_data();` - Returns laser sensor data as a `HAL::LaserData` struct.
- `HAL::set_v(velocity);` - to set the linear speed.
- `HAL::set_w(velocity);` - to set the angular speed.
- `HAL::lift();` - to lift the platform.
- `HAL::putdown();` - to put down the platform.
- `HAL::get_lift_state();` - Returns the current lift state as a `bool`.
- `WebGUI::show_path(array);` - shows a path on the map. The parameter should be a `std::vector<std::vector<double>>` containing each of the points of the path.
- `WebGUI::get_map(url);` - Returns the map image as a `cv::Mat`. The URLs of the worlds are in the **Supporting information** section.
- `WebGUI::show_image(image);` - Displays the given `cv::Mat` image in the WebGUI.

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

#### ROS 2 Topics

Use standard ROS 2 topics for direct communication with the simulation and the WebGUI.

- `/logistic_robot/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`
- `/logistic_robot/odom` - Subscribe to this topic to receive the current pose of the robot. Message type: `nav_msgs/msg/Odometry`
- `/logistic_robot/scan` - Subscribe to this topic to receive the 2D laser data. Message type: `sensor_msgs/msg/LaserScan`
- `/logistic_robot/platform/cmd_vel` - Publish to this topic to control the lifting platform. Positive values lift the platform (`5.0`) and negative values (`-5.0`) lower it. Message type: `std_msgs/msg/Float64`

For WebGUI debugging:

- `/webgui/path` - Publish the planned path so it can be displayed in the WebGUI. Message type: `std_msgs/msg/String`. QoS: `TRANSIENT_LOCAL` (depth 1)
- `/webgui/image` - Publish a debug image for visualization in the WebGUI. Message type: `sensor_msgs/msg/Image`. QoS: `TRANSIENT_LOCAL` (depth 1)

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

## Gazebo Harmonic tools

In Gazebo Harmonic it is possible to manually place the robot in the world using the **Entity Tree** and **Transform Control** tools. This can be useful for inspecting world coordinates and verifying the alignment between the simulation and the map used in the exercise, which is particularly helpful when working with the coordinate transformations described in [Conversion From 3D to 2D](#conversion-from-3d-to-2d).

This is the standard way to manually manipulate entities in Gazebo Harmonic through the graphical interface.

To enable this feature:

1. Open the **three-dot menu** in Gazebo Harmonic.
2. Open the **plugin selector**.
3. Enable:
    - **Entity Tree**
    - **Transform Control**

Once enabled, select the robot in the **Entity Tree**. The **Transform tool** will display the manipulation axes directly on the robot model. Using these axes, the robot can be translated in the world to inspect positions and coordinates.
{% include gallery id="gazebo_move" caption="Moving the robot in Gazebo Harmonic" %}

## Supporting information

There are two robots to choose from:

#### Holonomic robot:

- The robot size is 0.55 meters long and 0.55 meters wide.

{% include gallery id="holonomicrobot" caption="Holonomic robot" %}

#### Ackermann robot:

- The robot size is 0.72 meters long and 0.32 meters wide.

{% include gallery id="ackermannrobot" caption="Ackemann robot" %}

- You can get the robot's mesh files from there:
    - Only robot: /resources/exercises/amazon_warehouse/meshes/robot/ackermann_robot.dae
    - Robot lifting a shelf: /resources/exercises/amazon_warehouse/meshes/robot/ackermann_robot_lifting_shelf.dae

{% include gallery id="ackermannrobotmesh" caption="Ackemann robot meshes" %}

There are two warehouses to choose from:

#### Warehouse 1:

- The warehouse size is 20.62 meters long and 13.6 meters wide.
- The shelves coordinates from 1 to 6 are: (3.728, 0.579), (3.728, -1.242), (3.728, -3.039), (3.728, -4.827), (3.728, -6.781), (3.728, -8.665).
- You can get the warehouse's map from there: /resources/exercises/amazon_warehouse/images/map.png

{% include gallery id="warehouse1" caption="Warehouse 1" %}

- You can get the warehouse's mesh file from there:
    - Warehouse without shelves: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/warehouse1.dae
    - Shelf 1: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/shelf1.dae
    - Shelf 2: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/shelf2.dae
    - Shelf 3: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/shelf3.dae
    - Shelf 4: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/shelf4.dae
    - Shelf 5: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/shelf5.dae
    - Shelf 6: /resources/exercises/amazon_warehouse/meshes/world/warehouse1/shelf6.dae

{% include gallery id="warehouse1mesh" caption="Warehouse 1 meshes" %}

#### Warehouse 2:

- The warehouse size is 34 meters long and 22 meters wide.
- The shelves coordinates from 1 to 9 are: (-8.5, -6.0), (-1.5, -6.0), (5.5, -6.0), (-8.5, 4.0), (-1.5, 4.0), (5.5, 4.0), (-1.0, -15.5), (-2, 11.5), (1.0, -15.0).
    - The separation distance between two neighboring shelves is 2 m on the x axis and 1.5 m on the y axis.
- You can get the warehouse's map from there: /resources/exercises/amazon_warehouse/images/map_2.png

{% include gallery id="warehouse2" caption="Warehouse 2" %}

---

## MiR100
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/amazon_warehouse/mir100_robot.jpg" alt="MiR100" caption="The MiR100." %}

The [MiR100](https://mobile-industrial-robots.com/) is a differential drive robot with two laser scanners and no lifting platform, so with it the exercise is about moving around the warehouse. It runs in two worlds: **Warehouse 1 Mir100**, in simulation, and **Mir100 Physical**, the real robot, see [Running on the real robots](#running-on-the-real-robots). The same code moves the simulated and the real robot.

### The robot

{% include figure image_path="/assets/images/exercises/amazon_warehouse/mir100_top.png" alt="MiR100 seen from above" caption="Top view of the MiR100 with its frame and the field of both lasers." %}

- The robot is 0.89 meters long and 0.58 meters wide. It moves forward and backward and turns in place, but it cannot move sideways.
- The two lasers are at opposite corners, each one rotated 45 degrees. The front laser is at (0.4288, 0.2358) looking 45 degrees to the left, the back laser at (-0.3548, -0.2352) looking 135 degrees to the right, in metres from the robot origin, `x` forward and `y` to the left.
- In the **Warehouse 1 Mir100** world it starts at (0, 0) of Warehouse 1. On the real robot the odometry does not start at (0, 0).

### HAL-based Implementation in Python

- `import HAL_mir100 as HAL` - to import the HAL (Hardware Abstraction Layer) library class of the MiR100. This class contains the functions that send and receive information to and from the Hardware (Gazebo or the real robot).
- `HAL.getPose3d()` - returns the robot pose from the odometry: `x`, `y` in metres and `yaw` in radians.
- `HAL.setV(velocity)` - to set the linear speed in m/s, positive forward. Limited to [-1, 1] m/s.
- `HAL.setW(velocity)` - to set the angular speed in rad/s, positive to the left. Limited to [-1, 1] rad/s.
- `HAL.getFrontLaserData()` - returns the scan of the front laser.
- `HAL.getBackLaserData()` - returns the scan of the back laser.

Each scan has `values`, a list of 541 distances in metres covering 270 degrees, one every 0.5 degrees, plus `minAngle`, `maxAngle`, `minRange` and `maxRange`. `values[0]` is at -135 degrees and `values[540]` at +135 degrees of the laser. Readings that hit nothing are returned as the maximum range, or `inf` in simulation.

The C++ HAL is not available for the MiR100.

### ROS 2-direct Implementation

The topics are the same for the simulated and the real robot.

- `/mir100/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`
- `/mir100/odom` - Subscribe to this topic to receive the odometry of the robot. Message type: `nav_msgs/msg/Odometry`
- `/mir100/front_laser/scan` - Subscribe to this topic to receive the front laser data. Message type: `sensor_msgs/msg/LaserScan`
- `/mir100/back_laser/scan` - Subscribe to this topic to receive the back laser data. Message type: `sensor_msgs/msg/LaserScan`
- `/mir100/imu` - Subscribe to this topic to receive the IMU data. Message type: `sensor_msgs/msg/Imu`

---

## Theory

This exercise is a motion planning problem. Jderobot Academy already has [an exercise dedicated to this](http://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/), which I'd definitely recommend the readers to check out, so the challenge in this exercise isn't to implement a motion planning algorithm but learning to use the [OMPL](https://ompl.kavrakilab.org/) (Open Motion Planning Library) for our purpose.

### [Open Motion Planning Library](https://ompl.kavrakilab.org/)

OMPL is a library for sampling-based motion planning, offering many state-of-the-art planning algorithms such as PRM, RRT, KPIECE, etc.

{% include gallery id="ompl" caption="OMPL structure" %}

As you can see in the diagram above, some key components of OMPL are:

- **State Space** defines the possible configurations that a robot can have. For example:
    - RealVectorStateSpace: represents an Euclidean space.
    - SO2StateSpace, SO3StateSpace: represents rotations in 2D and 3D.
    - SE2StateSpace, SE3StateSpace: combines translations and rotations in 2D and 3D.
    - ...
- **State Validaty Checker** determines if the configuration is valid, that is to say the configuration doesn't collide with an enviroment obstacle and respects the constraints of the robot.
- **Control Space** defines the movements that a robot can perform.
- **State Propagator** indicates the evolution of the system after applying a control.
- **Space Information** is the container that holds the state space, the state validity checker, and other information needed for planning.
- **Planner** responsible for generating a path from the start to the goal in the configuration space. OMPL supports a variety of planners, such as RRT, PRM, and FMT\*.
- **Path** is the output of the planner, which is a sequence of states representing a trajectory for the robot to follow.

The following example shows a 2D point robot inside a 10x10 space with an ovular obstacle at position (5.5, 5.5):

```python
from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

# specify valid state condition
def isStateValid(state):
  x = state.getX()
  y = state.getY()
  if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0:
    return False
  return True

def plan():
  # Construct the robot state space in which we're planning. We're
  # planning in [0,1]x[0,1], a subset of R^2.
  space = ob.SE2StateSpace()

  # set state space's lower and upper bounds
  bounds = ob.RealVectorBounds(2)
  bounds.setLow(0, dimensions[0])
  bounds.setLow(1, dimensions[1])
  bounds.setHigh(0, dimensions[2])
  bounds.setHigh(1, dimensions[3])
  space.setBounds(bounds)

  # construct a space information instance for this state space
  si = ob.SpaceInformation(space)
  # set state validity checking for this space
  si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

  # Set our robot's starting and goal state
  start = ob.State(space)
  start().setX(0)
  start().setY(0)
  start().setYaw(math.pi / 4)
  goal = ob.State(space)
  goal().setX(10)
  goal().setY(10)
  goal().setYaw(math.pi / 4)

  # create a problem instance
  pdef = ob.ProblemDefinition(si)

  # set the start and goal states
  pdef.setStartAndGoalStates(start, goal)

  # create a planner for the defined space
  planner = og.RRTConnect(si)

  # set the problem we are trying to solve for the planner
  planner.setProblemDefinition(pdef)

  # perform setup steps for the planner
  planner.setup()

  # solve the problem and print the solution if exists
  solved = planner.solve(1.0)
  if solved:
    print(pdef.getSolutionPath())
    plot_path(pdef.getSolutionPath(), dimensions)

def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

def plot_path(solution_path, dimensions):
  matrix = solution_path.printAsMatrix()
  path = create_numpy_path(matrix)
  x, y = path.T
  ax = plt.gca()
  ax.plot(x, y, 'r--')
  ax.plot(x, y, 'go')
  ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
  ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2]))

  plt.show()

if __name__ == "__main__":
  dimensions = [0, 0, 10, 10]
  obstacle = [5.5, 5.5, 1]   # [x, y, radius]
  plan()
```

Output:

```
Info:    RRTConnect: Space information setup was not yet called. Calling now.
Debug:   RRTConnect: Planner range detected to be 3.142586
Info:    RRTConnect: Starting planning with 1 states already in datastructure
Info:    RRTConnect: Created 9 states (4 start + 5 goal)
Geometric path with 7 states
Compound state [
RealVectorState [0 0]
SO2State [0.785398]
]
Compound state [
RealVectorState [1.26041 2.4087]
SO2State [-0.0626893]
]
Compound state [
RealVectorState [3.38076 4.59672]
SO2State [0.128782]
]
Compound state [
RealVectorState [3.66427 7.34115]
SO2State [0.895879]
]
Compound state [
RealVectorState [4.7318 7.51811]
SO2State [0.808152]
]
Compound state [
RealVectorState [7.7113 8.01201]
SO2State [0.563304]
]
Compound state [
RealVectorState [10 10]
SO2State [0.785398]
]

```

Plot:
{% include gallery id="example" caption="Example" %}

To better understand how to use the library, it is highly recommended to go to the [tutorials](https://ompl.kavrakilab.org/tutorials.html) and [demos](https://ompl.kavrakilab.org/group__demos.html) sections of the official website.

### Conversion From 3D to 2D

**Robot Localization** is the process of determining, where the robot is located regarding it's environment. Localization is an important resource for us in solving this exercise. Localization can be accomplished in any way possible, be it Monte Carlo, Particle Filter, or even Offline Algorithms. Since we have a map available to us, offline localization is the best way to move forward. Offline Localization will involve converting from a 3D environment scan to a 2D map. There are again numerous ways to do it, but the technique used in exercise is using **transformation matrices**.

#### Transformation Matrices

In simple terms, transformation is an invertible function that maps a set _X_ to itself. Geometrically, it moves a point to some other location in some space. Algebraically, all the transformations can be mapped using matrix representation. In order to apply transformation on a point, we multiply the point with the specific transformation matrix to get the new location. Some important transformations are:

- **Translation**

Translation of Euclidean Space(2D or 3D world) moves every point by a fixed distance in the same direction.

- **Rotation**

Rotation spins the object around a fixed point, known as center of rotation.

- **Scaling**

Scaling enlarges or diminishes objects, by a certain given scale factor.

- **Shear**

Shear rotates one axis so that the axes are no longer perpendicular.

In order to apply multiple transformations all at the same time, we use the concept of **Transformation Matrix** which enables us to multiply a single matrix for all the operations at once!

![Transformation Matrix](https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcRj8-LHyAz5s62zEx9fnQg_KLZX08E_rbdfHQ52kQZwh3BvPqMl)

_Transformation Matrix_

In our case we need to map a 3D Point in gazebo, to a 2D matrix map of our house. The equation used in the exercise was:

![Coordinate to Pixel]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/coord2pix.png)

_Coordinate to Pixel Conversion Equation_

In order to carry out the inverse operation of 3D to 2D, we can simply multiply, the pixel vector with the inverse of the transformation matrix to get the gazebo vector. The inverse of the matrix exists because the **mapping is invertible** and we **do not care about the z coordinate of the environment**, implying that each point in gazebo corresponds to a single point in the map.

## Hints

Simple hints provided to help you solve the Amazon Warehouse exercise. Please note that the **full solution has not been provided.** Also, the hints are more related to the reference solution, since multiple solutions are possible for this exercise.

### Ideas for solving the exercise

There are different ways to solve the exercise:

- #### Euclidean planning

    Define the robot as a 2D point and thicken the obstacles' edges according to the robot's radius to avoid collision, in this case, the state space can simply be an Euclidean space and all those black pixels will be invalid states. Maybe this [demo](https://ompl.kavrakilab.org/Point2DPlanning_8cpp_source.html) can help you!

    {% include gallery id="eucliean_eg" caption="Euclidean planning" %}

- #### Planning with robot's geometry constraints

    Since the robot's geometry is not always suitable to be approximated as a point, as in the case when the rectangular shelving is loaded, it would be a better way to solve the problem taking into consideration the robot's geometry.

    You can represent the robot as a set of pixels, so that a state will be valid when all pixels in the set are free. Now, it is no longer enough to use only Euclidean space but you must take into consideration the rotation of the robot. Therefore, it would be convenient to use a state space that includes that variable, such as [SE2StateSpace](https://ompl.kavrakilab.org/classompl_1_1base_1_1SE2StateSpace.html). And what else? A motion validator will be also essential, to ensure the feasibility of transitioning between states.

    You can make use of the [DubinsStateSpace](https://ompl.kavrakilab.org/classompl_1_1base_1_1DubinsStateSpace.html) or [ReedsSheppStateSpace](https://ompl.kavrakilab.org/classompl_1_1base_1_1ReedsSheppStateSpace.html) to solve the exersice. Check this [demo](https://ompl.kavrakilab.org/GeometricCarPlanning_8cpp_source.html)!

    {% include gallery id="geometry_eg" caption="Geometry constraints planning" %}

- #### Control-based planning

    In control-based planning, we need to define a control space and specify control propagation to set the robot movement constraints.

    However, there is another approach using the OMPL app class. This class includes predefined robot types such as dynamic car, kinematic car, quadrotor, etc. And, we can define the state space using mesh files. Check this [demo](https://ompl.kavrakilab.org/demos_2SE2RigidBodyPlanning_2SE2RigidBodyPlanning_8cpp_source.html)!
    - For now, only the meshes of the Ackermann robot and Warehouse 1 are available (listed in the **Supporting information** section).

    In this case, the solution path is composed of a sequence of controls (v, w, duration), instead of coordinates:

    ```python
    At state Compound state [
    RealVectorState [0 0]
    SO2State [0]
    ]
    apply control RealVectorControl [3.76703 0.00946806]
    for 1 steps
    ```

    Besides, the Gazebo RTF factor can impact the accurary of plan execution, so using simulated time can lead to better results.

#### How to find the shortest path?

The library offers the possibility to set an optimization objective, which could be a great help in finding the shortest path. Check out in [this tutorial](https://ompl.kavrakilab.org/optimalPlanningTutorial.html) how to do it!

#### Points to consider

- The loaded shelf is no longer a obstacle but part of the robot, then:
    - the robot's geometry changes.
    - remember to exclude the shelf itself when defining invalid states.

### Important points to remember

- Convert the coordinates from meter to pixel before representing with _WebGUI.showPath(array)_.

## Videos

### Demonstrative video of completed solution

{% include youtubePlayer.html id=page.youtubeId4 %}

### Euclidean planning

{% include youtubePlayer.html id=page.youtubeId1 %}

### Planning with robot's geometry constraints

{% include youtubePlayer.html id=page.youtubeId2 %}

### Control-based planning

{% include youtubePlayer.html id=page.youtubeId3 %}

* Contributors: [Lucía Lishan Chen Huang](https://github.com/lu164), [Blanca Soria Rubio](https://github.com/Blancasr), [Jose María Cañas](https://github.com/jmplaza)
* Maintained by [Lucía Lishan Chen Huang](https://github.com/lu164), [Javier Izquierdo](https://github.com/javizqh).

## Running on the real robots

The **Mir100 Physical** world moves a real MiR100 instead of a simulated one. No simulator runs: a bridge inside the Robotics Backend talks to the rosbridge server of the robot and publishes the same topics as the simulated MiR100, so `HAL_mir100` runs unchanged. It works with the MiR software 2.x, nothing has to be installed on the robot.

The steps are the usual ones of the [Unibotics user guide](https://unibotics.org/user_guide): launch the Robotics Backend on your computer, open the exercise in [unibotics.org](https://unibotics.org) and connect. The difference is that the Robotics Backend is launched with an extra option for the robot. The Robotics Backend runs inside a Docker container and the robot is outside it, so the container has to be told how to talk to the hardware. The MiR100 is reached over the network, so the container needs the robot address. The section below gives the full command to copy. It is the launch command without GPU acceleration of the user guide, since no simulator runs with a real robot, and it works on any computer.

### MiR100

**1. Robot.** Power the MiR100 on, release its emergency stop and check in its web interface that it has no errors.

**2. Network.** Connect the computer to the wifi of the robot, or to the network the robot is on. By default the robot has the address `192.168.12.20` on its own wifi, but it can be different: the robot may have been given another address, or be reached through the network of your lab. In that case find out its address first, from whoever manages the robot or from the list of devices connected to the router of that network, and use it in the next steps instead of `192.168.12.20`.

The bridge connects to the rosbridge server of the robot, always on port `9090`, the one of the MiR software. Check that the address answers and that the port is open:

```bash
ping 192.168.12.20
nc -zv 192.168.12.20 9090
```

**3. Launch the Robotics Backend.** `MIR100_ROBOT_IP` is the robot address of step 2, change it if it is not the default one.

```bash
docker run --rm -it --name robotics-backend -p 6080-6090:6080-6090 -p 1108:1108 -p 7163:7163 -e MIR100_ROBOT_IP=192.168.12.20 jderobot/robotics-backend:latest
```

**4. Open Amazon Warehouse** in [unibotics.org](https://unibotics.org), connect to the Robotics Backend and choose the **Mir100 Physical** world.

**5. Check the robot data** from a second terminal. The odometry has to arrive at a steady rate, stop it with Ctrl+C:

```bash
docker exec -it robotics-backend bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /mir100/odom"
```

**6. Run your code** with `import HAL_mir100 as HAL`, as in the Warehouse 1 Mir100 world. Start with low speeds, 0.1 to 0.3 m/s, and work with poses relative to the start of your program, since the odometry of the real robot does not start at (0, 0).

The bridge stops the robot when it stops receiving velocity commands, so set the velocity inside the loop of your code.

**Possible errors**

| Symptom | Fix |
| ------- | --- |
| The robot does not move | Check that the code imports `HAL_mir100` and not `HAL`, that the emergency stop is released and that the robot has no errors in its web interface. If it still does not move, put the robot in **Manual** mode in the web interface |
| The robot moves in short jerks | Set the velocity inside the loop, not only once |
| The code gets stuck on the first laser call | No laser data arrives from the robot. Check the network connection and that `MIR100_ROBOT_IP` is the robot address of step 2 |
| `nc` reports the port `9090` closed | The address is not the one of the robot, or the rosbridge server of the robot is not running. Check the address and the MiR software version, it has to be 2.x |
| The robot stops near obstacles | The safety system of the robot, this is expected |

## References

1. [https://ompl.kavrakilab.org/](https://ompl.kavrakilab.org/)
2. [https://ompl.kavrakilab.org/OMPL_Primer.pdf](https://ompl.kavrakilab.org/OMPL_Primer.pdf)
