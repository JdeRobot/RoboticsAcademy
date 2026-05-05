---
permalink: /exercises/AutonomousCars/global_navigation/
title: "Global Navigation using TeleTaxi"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Global Navigation"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/global_navigation/global_navigation.png
    image_path: /assets/images/exercises/global_navigation/global_navigation.png
    alt: "City View."
    title: "City View."
  - url: /assets/images/exercises/global_navigation/global_navigation_teaser.png
    image_path: /assets/images/exercises/global_navigation/global_navigation_teaser.png
    alt: "GUI."
    title: "GUI."
  - url: /assets/images/exercises/global_navigation/taxi.png
    image_path: /assets/images/exercises/global_navigation/taxi.png
    alt: "Model."
    title: "Model."

gifs:
  - url: /assets/images/exercises/global_navigation/miss.gif
    image_path: /assets/images/exercises/global_navigation/miss.gif
    alt: "examples"
    title: "examples"

  - url: /assets/images/exercises/global_navigation/two_step.gif
    image_path: /assets/images/exercises/global_navigation/two_step.gif
    alt: "examples"
    title: "examples"

potential:
  - url: /assets/images/exercises/global_navigation/potential_drop.png
    image_path: /assets/images/exercises/global_navigation/potential_drop.png
    alt: "Potential Drop"
    title: "Potential Drop"

  - url: /assets/images/exercises/global_navigation/potential_well.png
    image_path: /assets/images/exercises/global_navigation/potential_well.png
    alt: "Potential Well"
    title: "Potential Well"

  - url: /assets/images/exercises/global_navigation/potential_wall.png
    image_path: /assets/images/exercises/global_navigation/potential_wall.png
    alt: "Potential Wall"
    title: "Potential Wall"

  - url: /assets/images/exercises/global_navigation/total_potential.png
    image_path: /assets/images/exercises/global_navigation/total_potential.png
    alt: "Total Potential"
    title: "Total Potential"

Probability_Roadmap:
  - url: /assets/images/exercises/global_navigation/pr1.png
    image_path: /assets/images/exercises/global_navigation/pr1.png
    alt: "Step1"
    title: "Step1"

  - url: /assets/images/exercises/global_navigation/pr2.png
    image_path: /assets/images/exercises/global_navigation/pr2.png
    alt: "Step2"
    title: "Step2"

  - url: /assets/images/exercises/global_navigation/pr3.png
    image_path: /assets/images/exercises/global_navigation/pr3.png
    alt: "Step3"
    title: "Step3"

  - url: /assets/images/exercises/global_navigation/pr4.png
    image_path: /assets/images/exercises/global_navigation/pr4.png
    alt: "Step4"
    title: "Step4"

TreeBasedPlanner:
  - url: /assets/images/exercises/global_navigation/tr1.png
    image_path: /assets/images/exercises/global_navigation/tr1.png
    alt: "Step1"
    title: "Step1"

  - url: /assets/images/exercises/global_navigation/tr2.png
    image_path: /assets/images/exercises/global_navigation/tr2.png
    alt: "Step2"
    title: "Step2"

  - url: /assets/images/exercises/global_navigation/tr3.png
    image_path: /assets/images/exercises/global_navigation/tr3.png
    alt: "Step3"
    title: "Step3"

youtubeId1: zUtK6seVL5g
youtubeId2: q6G6BHqljP4
youtubeId3: itTbU4uLwfE
youtubeId4: zcS4X-ZO68U
youtubeId5: lO9Ru2mNAR4
youtubeId6: FZ8bP_VW1AA

---

## Goal

The objective of this exercise is to implement the logic of a Gradient Path Planning (GPP) algorithm. Global navigation through GPP, consists of:

{% include gallery caption="Gallery" %}

* Once selected a destination, the GPP algorithm is responsible for finding the shortest path to it, avoiding, in this our case, everything that is not road.

* Once the path has been selected, the logic necessary to follow this path and reach the objective must be implemented in the robot.

With this, it is possible for the robot to go to the marked destination autonomously and following the shortest path.

The solution can integrate one or more of the following difficulty increasing goals, as well as any other one that occurs to you:

* Reach the goal.

* Optimize the way to find the shortest path.

* Arrive as quickly as possible to the destination.

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the `tick` function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instantiate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options.

### HAL-based Implementation

#### Python

* `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
* `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.setV()` - to set the linear speed.
* `HAL.setW()` - to set the angular velocity.
* `HAL.getPose3d()` - returns x,y and theta components of the robot in world coordinates.
* `WebGUI.showNumpy(numpy)` - shows Gradient Path Planning field on the user interface. It represents the values of the field that have been assigned to the array passed as a parameter. Accepts as input a two-dimensional uint8 numpy array whose values can range from 0 to 255 (grayscale). In order to have a grid with the same resolution as the map, the array should be 400x400.
* `WebGUI.showPath(array)` - shows a path on the map. The parameter should be a 2D array containing each of the points of the path.
* `WebGUI.getTargetPose()` - returns x,y coordinates of chosen destionation in the world. Destination is set by clicking on the map image.
* `WebGUI.getMap(url)` - - Returns a numpy array with the image data in grayscale as a 2 dimensional array. The URL of the Global Navigation map is '/resources/exercises/global_navigation/images/cityLargenBin.png', so the instruction to get the map is
```
array = WebGUI.getMap('/resources/exercises/global_navigation/images/cityLargenBin.png')
```
* `WebGUI.rowColumn(vector)` - returns the index in map coordinates corresponding to the vector in world coordinates passed as parameter.
    
The map image has a resolution of 400x400 pixels and indicates whether there is an obstacle or not by its color. The map in the Gazebo world has its center in [0, 0] and it has a width and height of 500 meters. Therefore, each of the pixels in the map image represent a cell in the Gazebo world with a width and height of 1.25 meters.

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

- `HAL::set_v(velocity)` - sets the linear velocity of the taxi. The input is a `float`. Returns `void`.
- `HAL::set_w(velocity)` - sets the angular velocity of the taxi. The input is a `float`. Returns `void`.
- `HAL::get_pose3d()` - returns the current robot pose as a `HAL::Pose3d`.
- `HAL::get_pose3d().x` - gets the robot x position in world coordinates (`double`).
- `HAL::get_pose3d().y` - gets the robot y position in world coordinates (`double`).
- `HAL::get_pose3d().yaw` - gets the robot orientation around the vertical axis in world coordinates (`double`).
- `WebGUI::show_image(image)` - displays a debug image in the WebGUI. The input must be a `cv::Mat`. Returns `void`.
- `WebGUI::show_path(path)` - displays a path on the map. The input must be a `std::vector<std::vector<int>>`, where each inner vector represents a grid cell of the path. Returns `void`.
- `WebGUI::get_target_pose()` - returns the selected destination in world coordinates as a `std::vector<double>`.
- `WebGUI::get_map(url)` - returns the map image as a `cv::Mat`. The input must be a `std::string` with the map URL.
- `WebGUI::world_to_grid(pose)` - converts world coordinates to map grid coordinates. The input must be a `std::vector<double>` and the function returns a `std::vector<int>`.
- `WebGUI::grid_to_world(cell)` - converts map grid coordinates to world coordinates. The input must be a `std::vector<double>` and the function returns a `std::vector<double>`.

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

1. Example to load the map:

    ```cpp
    cv::Mat map = WebGUI::get_map("/resources/exercises/global_navigation/images/cityLargenBin.png");
    ```

2. Example to get the selected target:

    ```cpp
    std::vector<double> target = WebGUI::get_target_pose();

    double target_x = target[0];
    double target_y = target[1];
    ```

3. Example to convert coordinates:

    ```cpp
    // World coordinates to grid coordinates
    std::vector<double> world_pose = {target_x, target_y};
    std::vector<int> grid_cell = WebGUI::world_to_grid(world_pose);

    int row = grid_cell[0];
    int col = grid_cell[1];

    // Grid coordinates to world coordinates
    std::vector<double> grid_cell_as_double = {120.0, 250.0};
    std::vector<double> world_pose_from_grid = WebGUI::grid_to_world(grid_cell_as_double);

    double world_x = world_pose_from_grid[0];
    double world_y = world_pose_from_grid[1];
    ```

4. Example to show a planned path:

    ```cpp
    std::vector<std::vector<int>> path = {
        {10, 20},
        {11, 20},
        {12, 21}
    };

    WebGUI::show_path(path);
    ```


### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication with the simulation.

- `/cmd_vel` - Publish to this topic to set both linear and angular velocities. Message type: `geometry_msgs/msg/Twist`

- `/odom` - Subscribe to this topic to receive the robot odometry.  
  Message type: `nav_msgs/msg/Odometry`

- `/webgui/current_target` - Subscribe to this topic to receive the current goal selected from the WebGUI.  
  Message type: `geometry_msgs/msg/Point`  
  QoS: `TRANSIENT_LOCAL`, depth `1`

For image debugging:

- `/webgui/path` - Publish to this topic to display the planned path in the WebGUI.
  Message type: `nav_msgs/msg/Path`

- `/webgui/debug_image` - Publish to this topic to display a debug image typically used to visualize the cost map, wavefront expansion, or any other intermediate grid representation.
  Message type: `sensor_msgs/msg/Image`

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

#### C++

In order to use native ros controls you must include the following lines:

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

## Videos

{% include youtubePlayer.html id=page.youtubeId5 %}

## Theory

Motion Planning is a term used in robotics to find a sequence of valid configurations that moves the robot from source to destination. Motion Planning algorithms find themselves in a variety of settings, be it industrial manipulators, mobile robots, artificial intelligence, animations or study of biological molecules.

There are mainly 2 methods to solve the exercise, **Gradient Path Planning**, **Sampling Based Path Planning**

### Gradient Path Planning
One such method for Motion Planning is Gradient Path Planning. GPP works on the principle of potential fields. The obstacles in the path serve as potential walls to the path planner, and the target serves as potential well. By combining all the potential walls and wells, a path is constructed as a downward slope. The robot follows that path to reach it's destination.

{% include gallery id="potential" caption="Various illustrations based on potential" %}

Gradient Path Planning can be implemented using Brushfire Algorithm or Wave Front Algorithm. Next section explains the working of Wave Front Algorithm.

#### Wave Front Algorithm

Wave Front Algorithm is BFS based approach to build a path from source to destination. The algorithm works by assigning weights to a grid of cells. Given the source and target, the algorithm starts from the target node and moves outwards like a ripple, while progressively assigning weights to the neighboring cells.

![Assigning Weights]({{ site.url }}/RoboticsAcademy/assets/images/exercises/global_navigation/weights.png)

*Assigning Weights*

As for obstacles, additional weights are added to the cells that are close to obstacles. Intuitively, the weights represent the superposition of waves that are reflected from the walls of obstacles.

![Superposition of Waves]({{ site.url }}/RoboticsAcademy/assets/images/exercises/global_navigation/superposition.gif)

*Superposition of Waves*

The algorithm stops upon reaching the source. To navigate through the generated path, the robot follows the path indicated by decreasing weights (a downhill drive). A grayscale image representation quite clearly depicts the path the robot might follow!

![GrayScale Representation]({{ site.url }}/RoboticsAcademy/assets/images/exercises/global_navigation/grayscale.png)

*Gray Scale Representation*

### Sampling Based Path Planning

Sampling based Path Planning employs sampling of the state space of the robot in order to quickly and effectively plan paths, even with differential constraints or those with many degrees of freedom. Some of the algorithms under this class are:

#### Probabilistic Roadmap

These methods work by randomly sampling points in the **workspace**. Once the desired number of samples are obtained, the roadmap is constructed by connecting the random samples to form edges. On the resulting graph formed, any shortest path algorithm (A*, Dijkstra, BFS) is applied to get our resulting path.

{% include gallery id="Probability_Roadmap" caption="Probabilistic Roadmap" %}

#### Tree Based Planner

Tree Based Planners are very similar to Probabilistic Roadmaps, except for the fact that there are no cycles involved in tree based planners. There are a variety of tree based planners, like RRT, EST, SBL and KPIECE. These algorithms work heuristically, working from the root node, a tree (a graph without cycles) is constructed.

{% include gallery id="TreeBasedPlanner" caption="Tree Based Planner" %}

## Hints

Simple hints provided to help you solve the global navigation exercise. The hints are focused on solving the exercise through GPP algorithm.

### Path Planning

Since, the map has already been divided into grids, we can directly work on the grid! Path Planning involves a BFS based search starting from the destination. The psuedo code for the algorithm is:

- **Step1**:	insert Target Node into priority queue
- **Step2**:	c = pop node from priority queue
- **Step3**:	if c == start node	End
- **Step4**:	if c == obstacle	Save to another list and goto Step2
- **Step5**:	assign weight to neighbors of c if **previously unassigned**
- **Step6**:	insert neighbors of c to priority queue
- **Step7**:	goto Step2

Assignment of weights to the cells is arbitrary. Generally, diagonally neighboring cells are assigned a greater weight than the other neighbors. Apart from all the assignment, the additional weight of cells close to obstacles is extremely important, as it is the one that is going to help avoid collisions!

**Important Points to Remember**

- You may use `WebGUI.getMap()` to know whether an obstacle is present at (i, j) coordinate of the map. Also, in order to work with this grid, we have to invert our usage of coordinates. Implying, (i, j) can be accessed using (j, i).

- In order to assign those extra weights, we may take the obstacle points we saved earlier, and add extra values to the neighbors of the obstacle cell afterwards.

### Path Navigation

The next step involves Path Navigation. We can start by defining the low level implementation through linear speed and angular speed. Using the orientation of the robot, distance and direction of the target, linear speed and angular speeds can be assigned to the robot.

To navigate effectively, we assign local goal points which eventually lead to the final destination. These local goal points can be selected by choosing 2 points, which occur one after another **recursively**.

Also, make sure that the selection is done from a big enough radius, otherwise during navigation the robot may pass the grid and reach a **local minimum value**.

{% include gallery id="gifs" caption="Choose a large radius to avoid missing on turns (left) - Select the next point conditioned on the first point we select, and then plan accordingly (right)" %}

All in all, the exercise is a little on the tough side. But after spending some time working on this exercise, all the bugs, issues and hints provided may make sense!

## Demonstrative Video

{% include youtubePlayer.html id=page.youtubeId6 %}

Global Navigation with GPP:

{% include youtubePlayer.html id=page.youtubeId1 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId2 %}

<br/>

Global Navigation teletaxi with OMPL:

{% include youtubePlayer.html id=page.youtubeId3 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId4 %}

## Contributors

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Francisco Rivas](https://github.com/chanfr), [Francisco Pérez](https://github.com/fqez), [Jose María Cañas](https://github.com/jmplaza), [Nacho Arranz](https://github.com/igarag), [Nay Oo Lwin](https://github.com/NayOoLwin5), [Javier Izquierdo](https://github.com/javizqh).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna), [Javier Izquierdo](https://github.com/javizqh).

## References
1. [https://en.wikipedia.org/wiki/Motion_planning](https://en.wikipedia.org/wiki/Motion_planning)
2. [http://ompl.kavrakilab.org/OMPL_Primer.pdf](http://ompl.kavrakilab.org/OMPL_Primer.pdf)
3. [https://gsyc.urjc.es/jmplaza/students/tfg-Robotics_Academy-vanessa_fernandez-2017.pdf](https://gsyc.urjc.es/jmplaza/students/tfg-Robotics_Academy-vanessa_fernandez-2017.pdf)
4. [https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

