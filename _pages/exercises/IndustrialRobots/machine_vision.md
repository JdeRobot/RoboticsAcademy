---
permalink: /exercises/IndustrialRobots/machine_vision
title: "Machine Vision"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Machine Vision"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/machine_vision/world_secondexercise.png
    image_path: /assets/images/exercises/machine_vision/world_secondexercise.png
    alt: "Gazebo World"
    title: "Gazebo World"
  - url: /assets/images/exercises/machine_vision/rviz_secondexercise.png
    image_path: /assets/images/exercises/machine_vision/rviz_secondexercise.png
    alt: "Rviz"
    title: "Rviz"
  - url: /assets/images/exercises/machine_vision/machine_vision_teaser.png
    image_path: /assets/images/exercises/machine_vision/machine_vision_teaser.png
    alt: "Machine Vision"
    title: "Machine Vision"

color_filter:
  - url: /assets/images/exercises/machine_vision/green_filter_image.png
    image_path: /assets/images/exercises/machine_vision/green_filter_image.png
    alt: "Green colour filter"
    title: "Green colour filter"
  - url: /assets/images/exercises/machine_vision/blue_filter.png
    image_path: /assets/images/exercises/machine_vision/blue_filter.png
    alt: "Blue colour filter"
    title: "Blue colour filter"

shape_filter:
  - url: /assets/images/exercises/machine_vision/green_cylinder_image.png
    image_path: /assets/images/exercises/machine_vision/green_cylinder_image.png
    alt: "Green cylinder shape filter"
    title: "Green cylinder shape filter"
  - url: /assets/images/exercises/machine_vision/blue_cylinder_filter.png
    image_path: /assets/images/exercises/machine_vision/blue_cylinder_filter.png
    alt: "Blue cylinder shape filter"
    title: "Blue cylinder shape filter"

youtubeId: 719pIDC94RU
---

## Goal

The goal of this exercise is to learn how to **use vision to assist an industrial robot** by detecting known objects and unknown obstacles, and then completing a pick-and-place task with a robot arm and a **two-finger gripper**.

{% include gallery caption="Gallery." %}

Two depth cameras are available (one fixed to the world and another mounted on the robot end effector). The **shape, size and colour** of the objects are known, but their **poses** and the **surrounding obstacles** must be perceived using the cameras. The exercise runs on **Gazebo Harmonic** with ROS 2 and MoveIt 2.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instanciate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise supports a ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options, in Python and C++.

### HAL-based Implementation

#### Python

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).

**Robot information**

- `HAL.get_TCP_pose()` → `(xyz, ypr)` - Returns the current TCP pose. `xyz = [x, y, z]` in metres and `ypr = [yaw, pitch, roll]` in degrees.
- `HAL.get_Joint_states()` → `[j1..j6]` - Returns the current robot joint positions in degrees.

**Direct Kinematics**

- `HAL.MoveAbsJ(absolute_joints, speed, wait_time)` - Moves the robot to an absolute joint-space configuration (6 joint angles in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.
- `HAL.MoveSingleJ(joint_number, relative_angle, speed, wait_time)` - Moves a single joint [1 to 6] by a relative angular increment (in degrees).

**Inverse Kinematics**

- `HAL.MoveJoint(abs_xyz, abs_ypr, speed, wait_time)` - Point-to-Point (PTP) movement to an absolute Cartesian pose. `abs_xyz = [x, y, z]` in metres and `abs_ypr = [yaw, pitch, roll]` in degrees.
- `HAL.MoveLinear(abs_xyz, abs_ypr, speed, wait_time)` - Moves the TCP to an absolute Cartesian pose following a linear trajectory. Useful for pick-and-place approach motions.
- `HAL.MoveRelLinear(relative_xyz, speed, wait_time)` - Moves the TCP by a relative Cartesian displacement (in metres), keeping the orientation unchanged.
- `HAL.MoveRelReor(relative_ypr, speed, wait_time)` - Reorients the TCP by relative angular increments (Yaw,Pitch,Roll in degrees), keeping the position fixed.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.GripperSet(percentage_closure, wait_time)` - Closes (`100`) or opens (`0`) the two-finger gripper to the given closing percentage, adding a final delay in seconds. When it starts closing (`> 5`) automatic attachment is enabled; when it opens (`<= 5`) any attached object is automatically detached.

**Perception**

- `HAL.start_color_filter(color, rmax, rmin, gmax, gmin, bmax, bmin)` - Starts RGB colour filtering. Supported colours: `"red"`, `"green"`, `"blue"`, `"purple"`. RGB values in [0-255].
- `HAL.stop_color_filter(color)` - Stops a running colour filter.
- `HAL.start_shape_filter(color, shape, radius)` - Starts shape detection over the colour-filtered cloud. Supported shapes: `"sphere"`, `"cylinder"`. `radius` in metres.
- `HAL.stop_shape_filter(color, shape)` - Stops the shape filter.

**Object and target queries**

- `HAL.get_object_position(object_name)` → `[x, y, z]` or `None` - Returns the Cartesian position of a known object, or `None` if it is not found.
- `HAL.get_object_info(object_name)` → `(height, width, length, shape, color)` - Returns the metadata of a known object.
- `HAL.get_target_position(target_name)` → `geometry_msgs/Point` - Returns the position of a target (access it as `.x`, `.y`, `.z`).

**Cameras**

- `HAL.getImage(camera="hand")` - Returns an RGB image (numpy array) from the selected camera: `"hand"` (wrist-mounted) or `"base"` (fixed).

**Workspace and scanning**

- `HAL.scan_workspace()` - Moves the robot to a scan pose, triggers environment scanning, returns home, and returns the detected object list.
- `HAL.buildmap()` - Runs a full environment mapping procedure that updates the MoveIt 2 planning scene with the detected obstacles.
- `HAL.custom_scan_sequence(scan_positions)` - Runs a custom multi-pose scanning sequence. `scan_positions` is a list of 6-joint configurations. Returns the merged detected objects.
- `HAL.set_home_position(joint_angles_deg)` / `HAL.get_home_position()` - Set or get the robot home joint configuration.
- `HAL.back_to_home()` - Moves the robot to the home position and opens the gripper.
- `HAL.move_joint_arm(j0, j1, j2, j3, j4, j5)` - Convenience wrapper for absolute joint-space motion.

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI library. Used to display camera images in the browser.

**Robot information**

- `HAL::get_TCP_position();` - Returns the current TCP position as `std::array<double, 3>` [x, y, z] in metres.
- `HAL::get_TCP_orientation();` - Returns the current TCP orientation as `std::array<double, 3>` [yaw, pitch, roll] in degrees.
- `HAL::get_Joint_states();` - Returns the current joint positions as `std::array<double, 6>` in degrees.

**Direct Kinematics**

- `HAL::MoveAbsJ(joints, speed, wait_time);` - Moves the robot to the given angular position for each joint. `joints` is `std::array<double, 6>` in degrees, `speed` in [0,1], `wait_time` in seconds.
- `HAL::MoveSingleJ(joint_number, relative_angle, speed, wait_time);` - Moves a single joint by a relative angular increment. `joint_number` in [1,6], angle in degrees.

**Inverse Kinematics**

- `HAL::MoveJoint(xyz, ypr, speed, wait_time);` - Moves the TCP to an absolute Cartesian pose. `xyz` is `std::array<double, 3>` in metres, `ypr` in degrees.
- `HAL::MoveLinear(xyz, ypr, speed, wait_time);` - Moves the TCP in a linear trajectory to an absolute Cartesian pose. `xyz` in metres, `ypr` in degrees.
- `HAL::MoveRelLinear(xyz, speed, wait_time);` - Moves the TCP by a relative Cartesian increment. `xyz` is `std::array<double, 3>` in metres.
- `HAL::MoveRelReor(ypr, speed, wait_time);` - Reorients the TCP by relative angular increments. `ypr` is `std::array<double, 3>` in degrees.

**Gripper**

- `HAL::GripperSet(relative_closure, wait_time);` - Controls the gripper. `relative_closure` in [0,100] (0 = fully open, 100 = fully closed). When closing (`> 5`) contact-based automatic attachment is enabled; when opening (`<= 5`) the attached object is automatically released.

**Cameras**

- `HAL::getImage(camera);` - Returns a camera frame as `cv::Mat`. `camera` is `"hand"` (wrist-mounted) or `"base"` (fixed). Default: `"hand"`.
- `WebGUI::showImage(image);` - Sends a `cv::Mat` frame to the browser image panel.

**Perception filters**

- `HAL::start_color_filter(color, rmax, rmin, gmax, gmin, bmax, bmin);` - Starts an RGB colour filter. Supported colours: `"red"`, `"green"`, `"blue"`, `"purple"`. RGB values in [0,255].
- `HAL::stop_color_filter(color);` - Stops the colour filter for the given colour.
- `HAL::start_shape_filter(color, shape, radius);` - Starts shape detection on the colour-filtered cloud. Supported shapes: `"sphere"`, `"cylinder"`. `radius` in metres.
- `HAL::stop_shape_filter(color, shape);` - Stops the shape filter.

**Object and target queries**

- `HAL::get_object_position(object_name);` - Returns the position of a known object as `std::array<double, 3>` [x, y, z] in metres. Returns `{0, 0, 0}` if not found.
- `HAL::get_object_info(object_name);` - Returns an `ObjectInfo` struct with fields: `position`, `height`, `width`, `length`, `shape`, `color`. Check `info.shape.empty()` to detect a not-found object.
- `HAL::get_target_position(target_name);` - Returns the position of a target zone as `std::array<double, 3>` in metres.

**Workspace**

- `HAL::scan_workspace();` - Moves to a scan pose, triggers environment scanning, and returns home.
- `HAL::buildmap();` - Runs a full environment mapping procedure.
- `HAL::gripper_percentage_for(diameter, max_open_m);` - Converts an object diameter (m) to gripper closure percentage. `max_open_m` defaults to `0.085` m.

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

Instead of the HAL, you can write your own ROS 2 node (importing only `WebGUI`) and talk straight to the simulation through standard topics and actions. The robot, gripper and perception are provided by the simulator; `WebGUI` exposes the debug image topic used by the browser.

All the interfaces below use the default QoS (reliable, keep-last, depth 10). In particular, publish to `/webgui_image` with this default profile so it stays compatible with the GUI subscriber.

**Cameras** (`sensor_msgs/msg/Image`, BGR8)

- `/hand_camera/image` - Subscribe to receive the wrist-mounted camera image.
- `/base_camera/image` - Subscribe to receive the fixed base camera image.

**Image debugging**

- `/webgui_image` - Publish a `sensor_msgs/msg/Image` here to display it in the browser image panel (the equivalent of `WebGUI.showImage`).

**Arm motion** (IFRA `ros2srrc` action servers)

- `/Move` - `ros2srrc_data/action/Move` action. Send `action: "MoveJ"` with the six target joints in `movej` (degrees) for joint-space motion (equivalent to `MoveAbsJ`).
- `/Robmove` - `ros2srrc_data/action/Robmove` action. Send `type: "PTP"` (point-to-point) or `type: "LIN"` (linear) with the absolute Cartesian goal (`x, y, z` in metres and the `qx, qy, qz, qw` orientation quaternion). Equivalent to `MoveJoint` / `MoveLinear`.

**Gripper**

- `/gripper_controller/follow_joint_trajectory` - `control_msgs/action/FollowJointTrajectory` action controlling the `robotiq_85_left_knuckle_joint` (`0.0` open .. `1.0` closed).
- `/gripper_auto_attach` - Publish `std_msgs/msg/Bool` (`true` while closing, `false` while opening) to enable/disable the contact-based attachment.
- `/graspable_objects` - Publish a comma-separated `std_msgs/msg/String` with the objects that may be attached.

**Perception filters**

- `/start_color_filter` - Publish `pcl_filter_msgs/msg/ColorFilter` to start/stop RGB colour filtering (`color` id, RGB `rmin/rmax/gmin/gmax/bmin/bmax`, `status`).
- `/start_shape_filter` - Publish `pcl_filter_msgs/msg/ShapeFilter` to start/stop shape detection (`color` id, `shape` id, `radius`, `status`).

The numeric ids used by the filter messages are `red=1, green=2, blue=3, purple=4` and `sphere=1, cylinder=2`.

**Robot feedback**

- `/joint_states` - `sensor_msgs/msg/JointState`, the current joint positions (radians).
- The current TCP pose is available from TF (`world` → `tool0`).

#### Object and target positions

In the HAL-based version, `get_object_position()` and `get_target_position()` read these values from the exercise configuration file. A ROS 2-direct node does not have access to that file (and the C++ template does not parse it), so use the known scene positions directly. The robot base is at the origin, so all values below are base-relative, in metres. The object `Z` is the value `get_object_position()` returns: the object centre plus its radius (spheres) or half its height (cylinders).

**Objects** `[x, y, z]`:

| Object | Position |
| ---------------- | ------------------------ |
| `red_sphere`     | `[0.45, -0.25, 0.31]`    |
| `green_sphere`   | `[0.45,  0.09, 0.31]`    |
| `blue_sphere`    | `[0.45,  0.25, 0.31]`    |
| `purple_sphere`  | `[0.45, -0.09, 0.31]`    |
| `red_cylinder`   | `[0.65,  0.09, 0.31]`    |
| `green_cylinder` | `[0.65, -0.09, 0.305]`   |
| `blue_cylinder`  | `[0.65, -0.25, 0.315]`   |
| `purple_cylinder`| `[0.65,  0.25, 0.31]`    |

**Targets** `[x, y, z]` (all at `z = 0.25`):

| Target | Position | Target | Position |
| --------- | ---------------------- | ---------- | ---------------------- |
| `target1` | `[-0.68, -0.18, 0.25]` | `target9`  | `[-0.44, -0.18, 0.25]` |
| `target2` | `[-0.68, -0.06, 0.25]` | `target10` | `[-0.44, -0.06, 0.25]` |
| `target3` | `[-0.68,  0.06, 0.25]` | `target11` | `[-0.44,  0.06, 0.25]` |
| `target4` | `[-0.68,  0.18, 0.25]` | `target12` | `[-0.44,  0.18, 0.25]` |
| `target5` | `[-0.56, -0.18, 0.25]` | `target13` | `[-0.32, -0.18, 0.25]` |
| `target6` | `[-0.56, -0.06, 0.25]` | `target14` | `[-0.32, -0.06, 0.25]` |
| `target7` | `[-0.56,  0.06, 0.25]` | `target15` | `[-0.32,  0.06, 0.25]` |
| `target8` | `[-0.56,  0.18, 0.25]` | `target16` | `[-0.32,  0.18, 0.25]` |

#### Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the browser GUI and its `/webgui_image` bridge.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so guard your own initialization with `if not rclpy.ok(): rclpy.init()`.

#### C++

In order to use direct ROS controls you must include the following lines:

```cpp
#ifndef USER_NODE
#define USER_NODE

#include "rclcpp/rclcpp.hpp"

class UserNode : public rclcpp::Node {
  // Your class
};

#endif
```

You must define `USER_NODE` and a `UserNode` node class. In this mode only `WebGUI` is initialized (the HAL is not), so the arm, gripper and filters are commanded through the topics and actions listed above.

### Argument examples

**Home and intermediate poses (joint space, in degrees)**

```
home      = [0.0, -90.0, 70.0, -70.0, -90.0, 0.0]
pre_pick  = [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
pre_place = [180.0, -90.0, 90.0, -90.0, -90.0, 0.0]
```

**Absolute XYZ poses for `MoveJoint` and `MoveLinear` (in metres)**

```
above_object = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
at_object    = [object_pos[0], object_pos[1], object_pos[2] - 0.025]
```

**TCP orientation (YPR, in degrees)**

```
down = [180.0, 0.0, -90.0]
```

**Colour filter presets (rmax, rmin, gmax, gmin, bmax, bmin)**

```
red    - 255, 100, 40,  0,   40,  0
green  - 40,  0,   255, 100, 40,  0
blue   - 40,  0,   40,  0,   255, 100
purple - 255, 100, 120, 0,   255, 50
```

## Theory

### Object detection

Object detectors are implemented on top of **PCL**. First, a colour filter removes the points outside the chosen RGB range, isolating only the objects of that colour:

{% include gallery id="color_filter" caption="Colour filter output for green (left) and blue (right)." %}

Then a shape segmentation detects the most likely **sphere** or **cylinder** for that colour (given an approximate radius), producing a TF frame and debug topics that let you compute approach and grasp poses:

{% include gallery id="shape_filter" caption="Shape filter output detecting the green and blue cylinders." %}

### Obstacle detection and avoidance

Point clouds captured by the camera mounted on the robot are used to update the MoveIt 2 planning scene (octomap). Build a map by moving the robot to observe the workspace; obstacles are then considered during planning to avoid collisions.

## Hints

Simple hints to help you solve the Machine Vision exercise.

### Where to insert and run the code

In the launched web page, type your code in the text editor and run it by pressing the play button:

```python
import HAL
# Enter sequential code here!

while True:
    # Enter iterative code here!
```

### How should I solve the exercise?

Implement the high-level flow using HAL. A typical sequence is:

```python
# 1) Build an obstacle map
HAL.buildmap()

# 2) Return to a clean start state
HAL.back_to_home()

# 3) Detect one object (example: green cylinder)
HAL.start_color_filter("green", 40, 0, 255, 100, 40, 0)
HAL.start_shape_filter("green", "cylinder", 0.03)
pos = HAL.get_object_position("green_cylinder")
HAL.stop_shape_filter("green", "cylinder")
HAL.stop_color_filter("green")

if pos is not None:
    # 4) Approach, grasp and lift
    tcp_ypr = [180, 0, -90]
    HAL.MoveLinear([pos[0], pos[1], pos[2] + 0.10], tcp_ypr, 0.2, 0.5)
    HAL.MoveLinear([pos[0], pos[1], pos[2]], tcp_ypr, 0.1, 0.0)
    HAL.GripperSet(45, 0.3)  # close (auto-attach)
    HAL.MoveRelLinear([0, 0, 0.10], 0.2, 0.3)

    # 5) Place at a target
    target = HAL.get_target_position("target6")
    HAL.MoveLinear([target.x, target.y, target.z + 0.10], tcp_ypr, 0.2, 0.0)
    HAL.MoveLinear([target.x, target.y, target.z], tcp_ypr, 0.1, 0.0)
    HAL.GripperSet(0, 0.3)  # open (auto-detach)
    HAL.MoveRelLinear([0, 0, 0.10], 0.2, 0.2)
```

If planning fails (`Fail: ABORTED: No motion plan found. No execution attempted.`), the pose is likely unreachable. Adjust the Z-clearance or the orientation and retry.

### How to write `buildmap()`

Move the robot through **several viewpoints** so the wrist camera can observe the surroundings. The point cloud is fused into an octomap that MoveIt 2 uses as collision geometry.

### How to use `get_object_position()`

1. Pick a colour, a shape and an approximate radius.
2. Start the colour filter and tune the RGB limits; verify with the GUI image feeds.
3. Start the shape filter and verify with the debug topics; adjust the radius if needed.
4. Read the position with `HAL.get_object_position(object_name)` and approach with a safe Z-clearance.
5. Stop the filters when done to avoid noisy results.

### How to check filter results

Two image panes are provided in the GUI. Select the topics to view the **colour-filtered** and **shape-filtered** images (like the ones shown in the [Theory](#object-detection) section). For 3D inspection, open RViz and switch the **PointCloud** topic to the filter output. If nothing is detected, the image remains black or the cloud does not update.

### Limitations of the gripper simulation

Grasp simulation can be imperfect and objects may slip. Consider a run successful if the **map is built** and the **desired object is picked** reliably.

### Ignorable ERROR and WARNING messages

- `No p gain specified for pid.`
- Other transient TF/initialization warnings at startup (safe to ignore if they stop repeating once the system settles).

### Object and target lists

**Object list** (four spheres and four cylinders, in red, green, blue and purple):

- `red_sphere`, `green_sphere`, `blue_sphere`, `purple_sphere`
- `red_cylinder`, `green_cylinder`, `blue_cylinder`, `purple_cylinder`

**Target list:** `target1` .. `target16`.

## Videos

{% include youtubePlayer.html id=page.youtubeId %}

## Contributors

- Contributors: [Diego Martín](https://github.com/diegomrt), [José María Cañas](https://github.com/jmplaza) and [Javier Izquierdo](https://github.com/javizqh).

## References

1. IFRA-Cranfield (2023). ROS 2 Sim-to-Real Robot Control. [https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
2. [https://moveit.ros.org/](https://moveit.ros.org/)
3. [https://pointclouds.org/](https://pointclouds.org/)
