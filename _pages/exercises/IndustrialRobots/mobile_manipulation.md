---
permalink: /exercises/IndustrialRobots/mobile_manipulation
title: "Mobile Manipulation"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Mobile Manipulation"
toc_icon: "cog"

youtubeId1: 1XDqUomPsKM
youtubeId2: bsVp3GUh2f8

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/mobile_manipulation/mmo500_robot.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_robot.jpg
    alt: "MMO-500"
    title: "MMO-500"
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg
    alt: "XLeRobot Warehouse"
    title: "XLeRobot Warehouse"
  - url: /assets/images/exercises/mobile_manipulation/myagv_warehouse.jpg
    image_path: /assets/images/exercises/mobile_manipulation/myagv_warehouse.jpg
    alt: "myAGV mechArm Warehouse Delivery"
    title: "myAGV mechArm Warehouse Delivery"

xlerobot:
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_grasp.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_grasp.jpg
    alt: "XLeRobot grasping the yellow box"
    title: "XLeRobot grasping the yellow box"
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_place.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_place.jpg
    alt: "XLeRobot placing the green cylinder"
    title: "XLeRobot placing the green cylinder"

mmo500:
  - url: /assets/images/exercises/mobile_manipulation/mmo500_grasp.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_grasp.jpg
    alt: "MMO-500 grasping the green cylinder"
    title: "MMO-500 grasping the green cylinder"
  - url: /assets/images/exercises/mobile_manipulation/mmo500_place.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_place.jpg
    alt: "MMO-500 placing the blue box"
    title: "MMO-500 placing the blue box"

myagv:
  - url: /assets/images/exercises/mobile_manipulation/myagv_pick.jpg
    image_path: /assets/images/exercises/mobile_manipulation/myagv_pick.jpg
    alt: "myAGV picking the red ball at the dispenser"
    title: "myAGV picking the red ball at the dispenser"
  - url: /assets/images/exercises/mobile_manipulation/myagv_drop.jpg
    image_path: /assets/images/exercises/mobile_manipulation/myagv_drop.jpg
    alt: "myAGV dropping the red ball in the lit box"
    title: "myAGV dropping the red ball in the lit box"
---

## Goal

The goal of this exercise is to practice integrating navigation and manipulation. The student will program a mobile manipulator (a wheeled base, one or two robot arms and their grippers) that has to drive to the objects, pick them up, carry them across a warehouse and leave each one at its destination.

{% include gallery caption="Gallery." %}

The exercise offers three worlds, each one with its own robot and HAL module. Choose the world in the exercise page and read the section of its robot. The exercise runs on **Gazebo Harmonic**.

- [**MMO-500 Warehouse**](#mmo-500-warehouse): the [Neobotix MMO-500](https://www.neobotix-robots.com/), an omnidirectional MPO-500 base with a UR10 arm and a Robotiq 2F-85 gripper, has to take three objects from a conveyor to the tray of their colour.
- [**XLeRobot Warehouse**](#xlerobot-warehouse): the [XLeRobot](https://github.com/Vector-Wangel/XLeRobot), a holonomic base with two 5-joint arms, a robot tray and an RGB-D head camera, has to take three objects from a conveyor to the tray of their colour.
- [**myAGV mechArm Warehouse Delivery**](#myagv-mecharm-warehouse-delivery): the [Elephant Robotics](https://www.elephantrobotics.com/) myAGV, a mecanum base with a mechArm 270 arm, a lidar and a front camera, has to deliver the coloured balls of two dispensers to the delivery box that lights up, while two warehouse workers walk around carrying boxes.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instanciate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise supports the HAL-based approach and ROS 2-direct implementation, both in Python and C++. Each world has its own robot and its own HAL module, and everything specific to a robot is in the section of its world.

| World | Robot | Python | C++ |
| ----- | ----- | ------ | --- |
| [MMO-500 Warehouse](#mmo-500-warehouse) | Neobotix MMO-500 | `import HAL_mmo500 as HAL` | `#include "HAL_mmo500.hpp"` |
| [XLeRobot Warehouse](#xlerobot-warehouse) | XLeRobot | `import HAL_xlerobot as HAL` | `#include "HAL_xlerobot.hpp"` |
| [myAGV mechArm Warehouse Delivery](#myagv-mecharm-warehouse-delivery) | myAGV with mechArm 270 | `import HAL_myagv as HAL` | `#include "HAL_myagv.hpp"` |

The three worlds share `import WebGUI` in Python and `#include "WebGUI.hpp"` in C++.

### Frames

- All the arm goals are expressed in the `base_footprint` frame of the robot: origin on the floor under the base centre, `x` forward, `y` to the left and `z` up. Each robot section has a top view with this frame.
- All the world coordinates are in metres, in the world frame returned by `HAL.getPose3d()`, which gives the exact pose of the robot (without odometry noise). `HAL.worldToBase(x, y, z)` turns a world point into `base_footprint`.

### ROS 2-direct Implementation

Use standard ROS 2 topics and actions for direct communication with the simulation. Each robot runs in its own namespace and its topics and actions are listed in its section. The arm goals of `Robmove` are expressed in `base_footprint`, like the HAL ones.

- `/clock` - Subscribe to this topic to receive the simulation time. Message type: `rosgraph_msgs/msg/Clock`

For image debugging:

- `/webgui_image` - Publish to this topic to set the debug image in the GUI interface. Message type: `sensor_msgs/msg/Image`

### ROS 2-direct setup in Python

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

### ROS 2-direct setup in C++

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

---

## MMO-500 Warehouse
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/mobile_manipulation/mmo500_warehouse.jpg" alt="MMO-500 Warehouse" caption="The MMO-500 at its start pose, with the four conveyors of the warehouse." %}

The [Neobotix MMO-500](https://www.neobotix-robots.com/), an omnidirectional MPO-500 base with a UR10 arm and a Robotiq 2F-85 gripper, has to take three objects from a conveyor to the tray of their colour.

{% include gallery id="mmo500" caption="Grasping the green cylinder (left) and placing the blue box (right)." %}

### The robot

{% include figure image_path="/assets/images/exercises/mobile_manipulation/mmo500_top.png" alt="MMO-500 Warehouse robot top view" caption="Top view of the MMO-500 base with its `base_footprint` frame, the UR10 base and the field of both lasers." %}

- The robot base is 0.80 meters long and 0.57 meters wide. It starts at (0, 0) facing +x.
- The UR10 arm base is at (0.205, 0, 0.802) in `base_footprint`. Arm joints, in order: `ur10_shoulder_pan_joint`, `ur10_shoulder_lift_joint`, `ur10_elbow_joint`, `ur10_wrist_1_joint`, `ur10_wrist_2_joint` and `ur10_wrist_3_joint`. Limits: [-2π, 2π] for every joint except `ur10_elbow_joint`, which is limited to [-π, π].
- The Robotiq 2F-85 gripper is driven by `robotiq_85_left_knuckle_joint`: `0.0` open, `0.8` closed.
- The front laser is at (0.442, 0, 0.372) and the back laser at (-0.442, 0, 0.372), looking backwards. Each one gives 720 values over ±1.48 rad, from 0.1 to 30 m.

### The warehouse

{% include figure image_path="/assets/images/exercises/mobile_manipulation/mmo500_map.png" alt="MMO-500 Warehouse map" caption="Warehouse seen from above, with the trays on conveyors 2, 3 and 4 and the three objects on conveyor 1." %}

- The warehouse is 15 meters long and 15 meters wide, closed by walls at x = ±7.5 and y = ±7.5.
- There are four conveyors of 2.5 x 1 m and 1 m tall: `conveyor1` at (5, 0), `conveyor2` at (0, 5), `conveyor3` at (-5, 0) and `conveyor4` at (0, -5). `conveyor1` and `conveyor3` run along the y axis, `conveyor2` and `conveyor4` along the x axis.
- The trays are 25 x 25 cm with a 3 cm rim.

### Objects and targets

The three objects are located on the east conveyor (`conveyor1`), which is 1 m tall. The pose is the centre of the object. Each one goes to the tray of its colour.

- **blue_box** — Size (l,w,h) = (12,5,7) cm — Pose (x,y,z) = (4.8, 0, 1.035) m
- **yellow_ball** — Size (r) = (4) cm — Pose (x,y,z) = (4.8, -0.35, 1.04) m
- **green_cylinder** — Size (r,h) = (4,10) cm — Pose (x,y,z) = (4.8, 0.35, 1.05) m

The three trays are located on conveyors that are 1 m tall.

- **green_tray** — Pose (x,y) = (0, 4.8) m
- **yellow_tray** — Pose (x,y) = (-4.8, 0) m
- **blue_tray** — Pose (x,y) = (0, -4.8) m

### HAL-based Implementation in Python

- `import HAL_mmo500 as HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL.getPose3d()` - Returns a `Pose3d` object with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (quaternion `[w, x, y, z]`) and `timeStamp` (seconds).
- `HAL.worldToBase(x, y, z)` - Returns the world point `(x, y, z)` expressed in `base_footprint` as a tuple `(x, y, z)`, using the current base pose.
- `HAL.setV(velocity)` - to set the forward speed of the base in m/s.
- `HAL.setVY(velocity)` - to set the sideways speed of the base in m/s (positive to the left).
- `HAL.setW(velocity)` - to set the yaw rate of the base in rad/s (positive counterclockwise).

**Time and joints**

- `HAL.getSimTime()` - Returns the simulation time as an object with `seconds` and `nanoseconds` fields.
- `HAL.sleepSim(seconds)` - Waits for the given number of simulation seconds.
- `HAL.getJointPositions()` - Returns a `dict` with the last reported position in radians of every joint, by joint name.

**Lasers**

- `HAL.getFrontLaserData()` / `HAL.getBackLaserData()` - Returns a `LaserData` object of the front / back laser, with `values` (720 distances in metres over ±1.48 rad), `minAngle`, `maxAngle`, `minRange`, `maxRange` and `timeStamp`. The back laser looks backwards.

**Inverse Kinematics**

- `HAL.moveArm(x, y, z, roll=math.pi, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP")` - Moves the gripper centre (between the finger pads) to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. The default orientation points the gripper down and `yaw` turns the fingers. `speed` is the MoveIt velocity scaling, between 0 and 1. `motion` is `"PTP"` for a point-to-point move or `"LIN"` for a linear trajectory. Returns `True` on success.
- `HAL.getArmPose()` - Returns the current gripper pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) in `base_footprint`, or `None` until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking. They return at once, so wait `duration` with `HAL.sleepSim(duration)` before the next arm command.

- `HAL.setArmJoints(positions, duration=3.0)` - Moves the UR10 to the 6 joint angles in radians, in the order of `HAL.ARM_JOINTS`.
- `HAL.homeArm(duration=3.0)` - Folds the arm over the base.
- `HAL.readyArm(duration=3.0)` - Unfolds the arm in front of the base with the gripper down. Planned moves work better from here than from home.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.setGripper(closed, duration=1.0, target=None)` - `closed = True` closes the gripper and enables automatic attachment, `closed = False` opens it and releases the object. `target` overrides the knuckle angle in radians, from `0.0` (open) to `0.8` (closed). Returns at once like the joint functions.

The objects that can be automatically grasped are: `blue_box`, `yellow_ball` and `green_cylinder`.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showDepthImage(depth)` - Shows a depth array in metres in the left panel as grayscale (closer is brighter, clipped at 4 m).

### HAL-based Implementation in C++

- `#include "HAL_mmo500.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL::get_pose3d();` - Returns a `HAL::Pose3d` struct with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (`std::array<double, 4>` quaternion `[w, x, y, z]`) and `timeStamp` (seconds).
- `HAL::world_to_base(x, y, z);` - Returns the world point `(x, y, z)` expressed in `base_footprint` as a `std::array<double, 3>`, using the current base pose.
- `HAL::set_v(velocity);` - to set the forward speed of the base in m/s.
- `HAL::set_vy(velocity);` - to set the sideways speed of the base in m/s (positive to the left).
- `HAL::set_w(velocity);` - to set the yaw rate of the base in rad/s (positive counterclockwise).

**Time and joints**

- `HAL::get_sim_time();` - Returns the simulation time as a `HAL::SimTime` struct with `seconds` and `nanoseconds` fields.
- `HAL::sleep_sim(seconds);` - Waits for the given number of simulation seconds.
- `HAL::get_joint_positions();` - Returns a `std::map<std::string, double>` with the last reported position in radians of every joint, by joint name.

**Lasers**

- `HAL::get_front_laser_data();` / `HAL::get_back_laser_data();` - Returns a `HAL::LaserData` struct of the front / back laser, with `values` (`std::vector<float>` of 720 distances in metres over ±1.48 rad), `minAngle`, `maxAngle`, `minRange`, `maxRange` and `timeStamp`. The back laser looks backwards.

**Inverse Kinematics**

- `HAL::move_arm(x, y, z, roll, pitch, yaw, speed, motion);` - Moves the gripper centre (between the finger pads) to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. `roll` defaults to `M_PI`, `pitch` and `yaw` to 0.0, which points the gripper down, and `yaw` turns the fingers. `speed` (MoveIt velocity scaling, between 0 and 1) defaults to 0.3 and `motion` to `"PTP"` (point-to-point move), `"LIN"` gives a linear trajectory. Returns `true` on success.
- `HAL::get_arm_pose();` - Returns a `std::optional<HAL::ArmPose>` with the current gripper pose (`x`, `y`, `z`, `qx`, `qy`, `qz`, `qw`, `roll`, `pitch`, `yaw`) in `base_footprint`, empty until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking. They return at once, so wait `duration` with `HAL::sleep_sim(duration);` before the next arm command.

- `HAL::set_arm_joints(positions, duration);` - Moves the UR10 to the 6 joint angles in radians of the `std::array<double, 6>` `positions`, in the order of `HAL::ARM_JOINTS`. `duration` defaults to 3.0 s.
- `HAL::home_arm(duration);` - Folds the arm over the base.
- `HAL::ready_arm(duration);` - Unfolds the arm in front of the base with the gripper down. Planned moves work better from here than from home.

The joint targets are also available as constants: `HAL::ARM_HOME_JOINTS` and `HAL::ARM_READY_JOINTS`.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL::set_gripper(closed, duration, target);` - `closed = true` closes the gripper and enables automatic attachment, `closed = false` opens it and releases the object. `duration` defaults to 1.0 s. The optional `target` overrides the knuckle angle in radians, from `HAL::GRIPPER_OPEN` (`0.0`) to `HAL::GRIPPER_CLOSED` (`0.8`). Returns at once like the joint functions.

The objects that can be automatically grasped are: `blue_box`, `yellow_ball` and `green_cylinder`.

**WebGUI**

- `WebGUI::show_image(image);` - Shows a BGR `cv::Mat` in the right panel.
- `WebGUI::show_depth_image(depth);` - Shows a depth `cv::Mat` in metres in the left panel as grayscale (closer is brighter, invalid pixels are black).

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "HAL_mmo500.hpp"
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

The node setup is described in [ROS 2-direct setup in Python](#ros-2-direct-setup-in-python) and [ROS 2-direct setup in C++](#ros-2-direct-setup-in-c). These are the interfaces of this robot:

- `/mmo500/odom` - Subscribe to this topic to receive the base pose in the world frame. Message type: `nav_msgs/msg/Odometry`

- `/mmo500/cmd_vel` - Publish to this topic to set the base velocities (`linear.x`, `linear.y` and `angular.z`). Message type: `geometry_msgs/msg/Twist`

- `/mmo500/front_laser/scan` and `/mmo500/back_laser/scan` - Subscribe to these topics to receive the front / back laser scans. Message type: `sensor_msgs/msg/LaserScan`

- `/mmo500/joint_states` - Subscribe to this topic to read the position of every joint. Message type: `sensor_msgs/msg/JointState`

- `/mmo500/robot_description` - Subscribe to this topic (QoS **TransientLocal**) to receive the URDF of the robot, with the joint origins and limits of the arm. Message type: `std_msgs/msg/String`

- `/mmo500/Robmove` - Action to move the gripper to a pose planned by MoveIt 2 (`type` is `"PTP"` or `"LIN"`, `speed` is the velocity scaling between 0 and 1, position `x`, `y`, `z` and quaternion `qx`, `qy`, `qz`, `qw`). Action type: `ros2srrc_data/action/Robmove`. Planned moves work better starting from the ready joint pose of [Argument examples](#argument-examples)

- `/mmo500/Robpose` - Subscribe to this topic to receive the current gripper pose. Message type: `ros2srrc_data/msg/Robpose`

- `/mmo500/arm_controller/follow_joint_trajectory` - Action to move the six UR10 joints directly. Action type: `control_msgs/action/FollowJointTrajectory`

- `/mmo500/gripper_controller/follow_joint_trajectory` - Action to move `robotiq_85_left_knuckle_joint`. Action type: `control_msgs/action/FollowJointTrajectory`

- `/gripper_auto_attach` - Publish `true` while closing the gripper to enable the automatic attachment and `false` to release the object. Message type: `std_msgs/msg/Bool`

- `/graspable_objects` - Publish to this topic the comma-separated names of the objects that can be attached (`blue_box,yellow_ball,green_cylinder`). Message type: `std_msgs/msg/String`

### Argument examples

**Joint targets for the MMO-500 (radians, in `HAL.ARM_JOINTS` order)**

```
home  = [1.5708, -2.75, 2.67, -1.5708, -1.5708, 0.0]
ready = [1.1428, -2.1233, 1.9619, -1.4094, -1.5708, -1.9988]
```

**World point to `base_footprint` for `moveArm`**

```
bx, by, bz = HAL.worldToBase(4.8, 0.0, 1.035)
```

**World point to `base_footprint` for `move_arm` in C++**

```cpp
auto [bx, by, bz] = HAL::world_to_base(4.8, 0.0, 1.035);
HAL::move_arm(bx, by, bz + 0.1);
```

---

## XLeRobot Warehouse
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg" alt="XLeRobot Warehouse" caption="The XLeRobot at its start pose, with the four conveyors of the warehouse." %}

The [XLeRobot](https://github.com/Vector-Wangel/XLeRobot), a holonomic base with two 5-joint arms, a robot tray and an RGB-D head camera, has to take three objects from a conveyor to the tray of their colour.

{% include gallery id="xlerobot" caption="Grasping the yellow box (left) and placing the green cylinder (right)." %}

### The robot

{% include figure image_path="/assets/images/exercises/mobile_manipulation/xlerobot_top.png" alt="XLeRobot Warehouse robot top view" caption="Top view of the XLeRobot base with its `base_footprint` frame. The arm named left in the API is on the right side of the robot and the arm named right on its left side." %}

- The robot base is 0.39 meters long and 0.46 meters wide. It starts at (0, 0) facing +x.
- The two arms are mounted at (-0.09, -0.11, 0.765) (arm named left) and (-0.09, 0.11, 0.765) (arm named right) in `base_footprint`.
- Arm joints, in order: `Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch` and `Wrist_Roll`, with suffix `_L` (left arm) or `_R` (right arm). Limits in radians: `Rotation` [-2.16, 2.16], `Pitch` [-0.22, 3.37], `Elbow` [-0.22, 3.14], `Wrist_Pitch` [-1.658, 1.658], `Wrist_Roll` [-2.744, 2.841].
- Jaw joints `Jaw_L` and `Jaw_R`: `-0.3745` closed, `1.7453` fully open. A 5 cm object is held with the jaw at about `0.55`.
- Head joints `head_pan_joint` [-3.2, 3.2] and `head_tilt_joint` [-0.76, 1.45]. The head carries an RGB-D camera of 640x480 with a horizontal field of view of 60º.
- Each wrist carries an RGB camera of 640x480 with a horizontal field of view of 86º.
- The robot tray has one spot for each arm.

### The warehouse

{% include figure image_path="/assets/images/exercises/mobile_manipulation/xlerobot_map.png" alt="XLeRobot Warehouse map" caption="Warehouse seen from above, with the trays on conveyors 2, 3 and 4 and the three objects on conveyor 1." %}

- The warehouse is 15 meters long and 15 meters wide, closed by walls at x = ±7.5 and y = ±7.5.
- There are four conveyors of 2.5 x 1 m and 0.74 m tall: `conveyor1` at (5, 0), `conveyor2` at (0, 5), `conveyor3` at (-5, 0) and `conveyor4` at (0, -5). `conveyor1` and `conveyor3` run along the y axis, `conveyor2` and `conveyor4` along the x axis.
- The trays are 25 x 25 cm with a 3 cm rim.

### Objects and targets

The three objects are located on the east conveyor (`conveyor1`), which is 0.74 m tall. The pose is the centre of the object. Each one goes to the tray of its colour.

- **blue_box** — Size (l,w,h) = (5,5,5) cm — Pose (x,y,z) = (4.55, -0.15, 0.765) m
- **green_cylinder** — Size (r,h) = (2.5,5) cm — Pose (x,y,z) = (4.55, 0.15, 0.765) m
- **yellow_box** — Size (l,w,h) = (5,5,5) cm — Pose (x,y,z) = (4.55, 0.6, 0.765) m

The three trays are located on conveyors that are 0.74 m tall.

- **green_tray** — Pose (x,y) = (0, 4.63) m
- **yellow_tray** — Pose (x,y) = (-4.63, 0) m
- **blue_tray** — Pose (x,y) = (0, -4.63) m

### HAL-based Implementation in Python

- `import HAL_xlerobot as HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL.getPose3d()` - Returns a `Pose3d` object with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (quaternion `[w, x, y, z]`) and `timeStamp` (seconds).
- `HAL.worldToBase(x, y, z)` - Returns the world point `(x, y, z)` expressed in `base_footprint` as a tuple `(x, y, z)`, using the current base pose.
- `HAL.setV(velocity)` - to set the forward speed of the base in m/s.
- `HAL.setVY(velocity)` - to set the sideways speed of the base in m/s (positive to the left).
- `HAL.setW(velocity)` - to set the yaw rate of the base in rad/s (positive counterclockwise).

The HAL limits the base acceleration to 0.4 m/s² (linear) and 0.5 rad/s² (angular), so the base reaches the commanded speed smoothly.

**Time and joints**

- `HAL.getSimTime()` - Returns the simulation time as an object with `seconds` and `nanoseconds` fields.
- `HAL.sleepSim(seconds)` - Waits for the given number of simulation seconds.
- `HAL.getJointPositions()` - Returns a `dict` with the last reported position in radians of every joint, by joint name.

**Cameras**

- `HAL.getImage()` - Returns the head camera image as a BGR `numpy` array of 640x480.
- `HAL.getDepthImage()` - Returns the head camera depth as a `float32` `numpy` array in metres, on the same pixel grid as `getImage()`.
- `HAL.getLeftArmImage()` / `HAL.getRightArmImage()` - Return the image of the left / right wrist camera as a BGR `numpy` array of 640x480.
- `HAL.pixelToCameraPoint(u, v)` - Returns the 3D point `(x, y, z)` in metres of pixel column `u` and row `v` in the head camera optical frame, or `None` when that pixel has no valid depth.
- `HAL.cameraToBase(x, y, z)` - Converts a point from the head camera optical frame to `base_footprint`. Returns a tuple `(x, y, z)` or `None` if the transform is not available yet.
- `HAL.cameraOff(*cameras)` - Lowers the render rate of the given cameras (`"head"`, `"left_arm"`, `"right_arm"`) to 0.2 Hz, or of all of them when none is given. Returns `True` on success.
- `HAL.cameraOn(*cameras)` - Restores the render rate (15 Hz) of the given cameras, or of all of them. Returns `True` on success.
- `HAL.setCameraRate(camera, hz)` - Sets the render rate in Hz of one camera. Returns `True` on success.

**Head**

- `HAL.setHeadJoints(pan, tilt, duration=1.0)` - Moves the head joints to the given angles in radians in `duration` seconds. `pan` in [-3.2, 3.2], `tilt` in [-0.76, 1.45] (positive values look down). Returns at once.

**Grasping**

- `HAL.graspLeft(x, y, z, speed=0.3)` / `HAL.graspRight(x, y, z, speed=0.3)` - Grasps with the left / right arm the object whose centre is at `(x, y, z)` in `base_footprint`: opens the jaw, approaches the object from the front, closes the jaw and retreats. Returns `True` on success.
- `HAL.placeLeft(x, y, z, speed=0.3)` / `HAL.placeRight(x, y, z, speed=0.3)` - Releases the object held by the left / right arm with its centre at `(x, y, z)` in `base_footprint`. Returns `True` on success.

**Robot tray**

- `HAL.dropInLeftTray(speed=0.3)` / `HAL.dropInRightTray(speed=0.3)` - Leaves the object held by the left / right arm in that arm's spot of the robot tray. The object stays fixed to the robot while it drives.
- `HAL.pickFromLeftTray(speed=0.3)` / `HAL.pickFromRightTray(speed=0.3)` - Takes again the object left in the left / right spot of the robot tray.

**Inverse Kinematics**

- `HAL.moveLeftArm(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP")` / `HAL.moveRightArm(...)` - Moves the left / right gripper to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. `speed` is the MoveIt velocity scaling, between 0 and 1. `motion` is `"PTP"` for a point-to-point move or `"LIN"` for a linear trajectory. Returns `True` on success. The arms have 5 joints, so only exactly reachable poses are accepted and MoveIt may return a flipped elbow. The grasping and tray functions solve the arm joints exactly instead.
- `HAL.getLeftArmPose()` / `HAL.getRightArmPose()` - Returns the current gripper pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) in `base_footprint`, or `None` until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking. They return at once, so wait `duration` with `HAL.sleepSim(duration)` before the next arm command.

- `HAL.setLeftArmJoints(positions, duration=2.0)` / `HAL.setRightArmJoints(positions, duration=2.0)` - Moves the arm to the 5 joint angles in radians, in the order of `HAL.LEFT_ARM_JOINTS` / `HAL.RIGHT_ARM_JOINTS`.
- `HAL.readyPose(duration=2.0)` - Both arms forward with the grippers pointing down.
- `HAL.homeLeftArm(duration=2.0)` / `HAL.homeRightArm(duration=2.0)` - Arm to its fully extended zero pose.
- `HAL.leftTrayPose(duration=2.0)` / `HAL.rightTrayPose(duration=2.0)` - Gripper over its spot of the robot tray.

**Gripper**

The grippers grasp and release objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.setLeftGripper(closed, duration=1.0, target=None)` / `HAL.setRightGripper(closed, duration=1.0, target=None)` - `closed = True` closes the jaw and enables automatic attachment, `closed = False` opens it and releases the object. `target` overrides the jaw angle in radians, between `-0.3745` (closed) and `1.7453` (open). Returns at once like the joint functions.

The objects that can be automatically grasped are: `blue_box`, `green_cylinder` and `yellow_box`.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showDepthImage(depth)` - Shows a depth array in metres in the left panel as grayscale (closer is brighter, clipped at 4 m).

### HAL-based Implementation in C++

- `#include "HAL_xlerobot.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL::get_pose3d();` - Returns a `HAL::Pose3d` struct with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (`std::array<double, 4>` quaternion `[w, x, y, z]`) and `timeStamp` (seconds).
- `HAL::world_to_base(x, y, z);` - Returns the world point `(x, y, z)` expressed in `base_footprint` as a `std::array<double, 3>`, using the current base pose.
- `HAL::set_v(velocity);` - to set the forward speed of the base in m/s.
- `HAL::set_vy(velocity);` - to set the sideways speed of the base in m/s (positive to the left).
- `HAL::set_w(velocity);` - to set the yaw rate of the base in rad/s (positive counterclockwise).

The HAL limits the base acceleration to 0.4 m/s² (linear) and 0.5 rad/s² (angular), so the base reaches the commanded speed smoothly.

**Time and joints**

- `HAL::get_sim_time();` - Returns the simulation time as a `HAL::SimTime` struct with `seconds` and `nanoseconds` fields.
- `HAL::sleep_sim(seconds);` - Waits for the given number of simulation seconds.
- `HAL::get_joint_positions();` - Returns a `std::map<std::string, double>` with the last reported position in radians of every joint, by joint name.

**Cameras**

- `HAL::get_image();` - Returns the head camera image as a BGR `cv::Mat` of 640x480.
- `HAL::get_depth_image();` - Returns the head camera depth as a `CV_32FC1` `cv::Mat` in metres, on the same pixel grid as `get_image()`.
- `HAL::get_left_arm_image();` / `HAL::get_right_arm_image();` - Return the image of the left / right wrist camera as a BGR `cv::Mat` of 640x480.
- `HAL::pixel_to_camera_point(u, v);` - Returns a `std::optional<std::array<double, 3>>` with the 3D point in metres of pixel column `u` and row `v` in the head camera optical frame, empty when that pixel has no valid depth.
- `HAL::camera_to_base(x, y, z);` - Converts a point from the head camera optical frame to `base_footprint`. Returns a `std::optional<std::array<double, 3>>`, empty if the transform is not available yet.
- `HAL::camera_off(cameras);` - Lowers the render rate of the cameras in the `std::vector<std::string>` `cameras` (`"head"`, `"left_arm"`, `"right_arm"`) to 0.2 Hz, or of all of them when it is empty (default). Returns `true` on success.
- `HAL::camera_on(cameras);` - Restores the render rate (15 Hz) of the given cameras, or of all of them. Returns `true` on success.
- `HAL::set_camera_rate(camera, hz);` - Sets the render rate in Hz of one camera. Returns `true` on success.

**Head**

- `HAL::set_head_joints(pan, tilt, duration);` - Moves the head joints to the given angles in radians in `duration` seconds (default 1.0). `pan` in [-3.2, 3.2], `tilt` in [-0.76, 1.45] (positive values look down). Returns at once.

**Grasping**

- `HAL::grasp_left(x, y, z, speed);` / `HAL::grasp_right(x, y, z, speed);` - Grasps with the left / right arm the object whose centre is at `(x, y, z)` in `base_footprint`: opens the jaw, approaches the object from the front, closes the jaw and retreats. `speed` defaults to 0.3. Returns `true` on success.
- `HAL::place_left(x, y, z, speed);` / `HAL::place_right(x, y, z, speed);` - Releases the object held by the left / right arm with its centre at `(x, y, z)` in `base_footprint`. Returns `true` on success.

**Robot tray**

- `HAL::drop_in_left_tray(speed);` / `HAL::drop_in_right_tray(speed);` - Leaves the object held by the left / right arm in that arm's spot of the robot tray. The object stays fixed to the robot while it drives.
- `HAL::pick_from_left_tray(speed);` / `HAL::pick_from_right_tray(speed);` - Takes again the object left in the left / right spot of the robot tray.

**Inverse Kinematics**

- `HAL::move_left_arm(x, y, z, roll, pitch, yaw, speed, motion);` / `HAL::move_right_arm(...);` - Moves the left / right gripper to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. `roll`, `pitch` and `yaw` default to 0.0, `speed` (MoveIt velocity scaling, between 0 and 1) to 0.3 and `motion` to `"PTP"` (point-to-point move), `"LIN"` gives a linear trajectory. Returns `true` on success. The arms have 5 joints, so only exactly reachable poses are accepted and MoveIt may return a flipped elbow. The grasping and tray functions solve the arm joints exactly instead.
- `HAL::get_left_arm_pose();` / `HAL::get_right_arm_pose();` - Returns a `std::optional<HAL::ArmPose>` with the current gripper pose (`x`, `y`, `z`, `qx`, `qy`, `qz`, `qw`, `roll`, `pitch`, `yaw`) in `base_footprint`, empty until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking. They return at once, so wait `duration` with `HAL::sleep_sim(duration);` before the next arm command.

- `HAL::set_left_arm_joints(positions, duration);` / `HAL::set_right_arm_joints(positions, duration);` - Moves the arm to the 5 joint angles in radians of the `std::array<double, 5>` `positions`, in the order of `HAL::LEFT_ARM_JOINTS` / `HAL::RIGHT_ARM_JOINTS`. `duration` defaults to 2.0 s.
- `HAL::ready_pose(duration);` - Both arms forward with the grippers pointing down.
- `HAL::home_left_arm(duration);` / `HAL::home_right_arm(duration);` - Arm to its fully extended zero pose.
- `HAL::left_tray_pose(duration);` / `HAL::right_tray_pose(duration);` - Gripper over its spot of the robot tray.

The joint targets are also available as constants: `HAL::READY_LEFT_JOINTS`, `HAL::READY_RIGHT_JOINTS`, `HAL::LEFT_TRAY_JOINTS`, `HAL::RIGHT_TRAY_JOINTS` and `HAL::ARM_HOME_JOINTS`.

**Gripper**

The grippers grasp and release objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL::set_left_gripper(closed, duration, target);` / `HAL::set_right_gripper(closed, duration, target);` - `closed = true` closes the jaw and enables automatic attachment, `closed = false` opens it and releases the object. `duration` defaults to 1.0 s. The optional `target` overrides the jaw angle in radians, between `HAL::GRIPPER_CLOSED` (`-0.3745`) and `HAL::GRIPPER_OPEN` (`1.7453`). Returns at once like the joint functions.

The objects that can be automatically grasped are: `blue_box`, `green_cylinder` and `yellow_box`.

**WebGUI**

- `WebGUI::show_image(image);` - Shows a BGR `cv::Mat` in the right panel.
- `WebGUI::show_depth_image(depth);` - Shows a `HAL::get_depth_image()` depth in metres in the left panel as grayscale (closer is brighter, invalid pixels are black).

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "HAL_xlerobot.hpp"
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

The node setup is described in [ROS 2-direct setup in Python](#ros-2-direct-setup-in-python) and [ROS 2-direct setup in C++](#ros-2-direct-setup-in-c). These are the interfaces of this robot:

- `/logistic_robot/odom` - Subscribe to this topic to receive the base pose in the world frame. Message type: `nav_msgs/msg/Odometry`

- `/logistic_robot/cmd_vel` - Publish to this topic to set the base velocities (`linear.x`, `linear.y` and `angular.z`). Ramp the speed (the HAL uses 0.4 m/s² and 0.5 rad/s²), since sudden changes swing the arms. Message type: `geometry_msgs/msg/Twist`

- `/logistic_robot/joint_states` - Subscribe to this topic to read the position of every joint. Message type: `sensor_msgs/msg/JointState`

- `/logistic_robot/robot_description` - Subscribe to this topic (QoS **TransientLocal**) to receive the URDF of the robot, with the joint origins and limits of the arm. Message type: `std_msgs/msg/String`

- `/logistic_robot/head_camera/image` - Subscribe to this topic to receive the head camera image. Message type: `sensor_msgs/msg/Image`

- `/logistic_robot/head_camera/depth_image` - Subscribe to this topic to receive the head camera depth in metres (32FC1). Message type: `sensor_msgs/msg/Image`

- `/logistic_robot/head_camera/camera_info` - Subscribe to this topic for the head camera intrinsics. Message type: `sensor_msgs/msg/CameraInfo`

- `/logistic_robot/left_arm_camera/image_raw` and `/logistic_robot/right_arm_camera/image_raw` - Subscribe to these topics to receive the wrist camera images. Message type: `sensor_msgs/msg/Image`

- `/tf` and `/tf_static` - The transform from `base_footprint` to `head_camera_depth_frame` converts head camera points to the robot frame. Message type: `tf2_msgs/msg/TFMessage`

- `/logistic_robot/left_robmove/Robmove` and `/logistic_robot/right_robmove/Robmove` - Actions to move the left / right gripper to a pose planned by MoveIt 2 (`type` is `"PTP"` or `"LIN"`, `speed` is the velocity scaling between 0 and 1, position `x`, `y`, `z` and quaternion `qx`, `qy`, `qz`, `qw`). Action type: `ros2srrc_data/action/Robmove`. The arms have 5 joints, so only exactly reachable poses are accepted and MoveIt may return a flipped elbow. Joint goals computed from the arm kinematics of `robot_description` are more reliable.

- `/logistic_robot/left_robpose/Robpose` and `/logistic_robot/right_robpose/Robpose` - Subscribe to these topics to receive the current left / right gripper pose. Message type: `ros2srrc_data/msg/Robpose`

- `/logistic_robot/left_arm_controller/follow_joint_trajectory` and `/logistic_robot/right_arm_controller/follow_joint_trajectory` - Actions to move the arm joints directly, in the order of the arm joints. Action type: `control_msgs/action/FollowJointTrajectory`

- `/logistic_robot/left_gripper_controller/follow_joint_trajectory` and `/logistic_robot/right_gripper_controller/follow_joint_trajectory` - Actions to move the jaw joints `Jaw_L` / `Jaw_R`. Action type: `control_msgs/action/FollowJointTrajectory`

- `/logistic_robot/head_controller/follow_joint_trajectory` - Action to move `head_pan_joint` and `head_tilt_joint`. Action type: `control_msgs/action/FollowJointTrajectory`

- `/xlerobot/left_gripper_attach` and `/xlerobot/right_gripper_attach` - Publish `true` while closing the jaw to enable the automatic attachment and `false` to release the object. Message type: `std_msgs/msg/Bool`

- `/xlerobot/left_gripper_stow` and `/xlerobot/right_gripper_stow` - With the object resting on its spot of the robot tray, publish `true` and then open the jaw to leave it fixed to the robot. To take it again, lower the open jaw onto it and close it with the attach topic set to `true`. The tray joint poses are in [Argument examples](#argument-examples-1). Message type: `std_msgs/msg/Bool`

- `/graspable_objects` - Publish to this topic the comma-separated names of the objects that can be attached (`blue_box,yellow_box,green_cylinder`). Message type: `std_msgs/msg/String`

### Argument examples

**Joint targets for the XLeRobot (radians, in arm joint order)**

```
ready_left  = [-1.600, 1.527, 2.123, 0.063, -1.571]
ready_right = [1.630, 1.525, 2.110, 0.074, -1.571]
```

**Robot tray joint poses for the XLeRobot (radians, in arm joint order).** The tray functions go through `carry` and `above` and lower the object to `drop`.

```
carry_left  = [-1.544, 1.356, 1.66, 0.612, -1.571]
above_left  = [-1.9181, 1.928, 2.1165, 1.2615, -1.5708]
drop_left   = [-1.9181, 1.276, 2.4177, 0.3084, -1.5708]
carry_right = [1.86, 1.332, 1.621, 0.142, -1.571]
above_right = [2.116, 1.6408, 1.8219, 1.5106, -1.5708]
drop_right  = [2.116, 1.1136, 2.099, 0.7062, -1.5708]
```

---

## myAGV mechArm Warehouse Delivery
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/mobile_manipulation/myagv_warehouse.jpg" alt="myAGV mechArm Warehouse Delivery" caption="Part of the delivery warehouse, with the guide tape, its white nodes and the workers carrying boxes." %}

The [Elephant Robotics](https://www.elephantrobotics.com/) myAGV, a mecanum base with a mechArm 270 arm, a lidar and a front camera, has to deliver the coloured balls of two dispensers to the delivery box that lights up, while two warehouse workers walk around carrying boxes.

{% include gallery id="myagv" caption="Picking the red ball at its dispenser (left) and dropping it in the lit box (right)." %}

### The robot

{% include figure image_path="/assets/images/exercises/mobile_manipulation/myagv_top.png" alt="myAGV mechArm Warehouse Delivery robot top view" caption="Top view of the myAGV base with its `base_footprint` frame, the arm base, the sensors and the floor zone the arm can reach." %}

- The robot base is 0.31 meters long and 0.20 meters wide, and its corners stick out up to 0.195 m from its centre. It starts at (1, 0) facing +x.
- The mechArm 270 base is at (0.03, 0, 0.132) in `base_footprint`. Arm joints, in order, with their limits in radians: `joint1_to_base` [-2.7925, 2.7925], `joint2_to_joint1` [-1.3089, 2.0943], `joint3_to_joint2` [-3.0543, 1.1344], `joint4_to_joint3` [-2.7052, 2.7052], `joint5_to_joint4` [-2.0071, 2.0071] and `joint6_to_joint5` [-3.14, 3.14].
- The gripper is driven by `gripper_base_to_gripper_left3`: `0.15` open (about 43 mm), `-0.4` closed (about 22 mm).
- The wrist cannot point straight down close to the base: the arm reaches the floor with the gripper 70º below the horizon (`HAL.GRASP_ROLL = -π + 20º`, `HAL.GRASP_YAW = -π/2`), between about 0.22 and 0.28 m ahead of the base centre.
- The lidar is at (0.065, 0, 0.08), rotated 180º about z. It gives 360 values over [-π, π], from 0.12 to 8 m.
- The front camera is at (0.13, 0, 0.131) looking forward, 640x480 with a horizontal field of view of 60º.

The arm only reaches the floor between about 0.22 and 0.28 m ahead of the base centre, with the gripper 70º below the horizon.
{: .notice--info}

### The warehouse

{% include figure image_path="/assets/images/exercises/mobile_manipulation/myagv_map.png" alt="myAGV mechArm Warehouse Delivery map" caption="Guide tape graph with its nodes, the two ball dispensers on the east wall and the five delivery boxes." %}

- The warehouse has shelves, clutter and a pallet jack, and two workers that walk around picking up and dropping three cargo boxes of 0.3 x 0.4 x 0.28 m. The workers and the boxes have collision.
- The two ball dispensers are on the east wall. The requested ball drops into a cup on the floor (inner radius 7.5 cm, wall 2.5 cm tall) and its centre rests at z = 0.025.
- A guide tape on the floor joins the robot start, the dispensers and the delivery boxes, with a white dot on each node. The nodes and the tape segments between them are:
    - `center` (1.0, 0.0), linked to `south`, `station_4` and `west`.
    - `south` (1.0, -4.0), linked to `center`, `station_3` and `east`.
    - `east` (4.0, -4.0), linked to `south`, `station_5` and `dispensers`.
    - `dispensers` (5.4, -4.0), linked to `east`, `red_turn` and `blue_turn`.
    - `red_turn` (5.4, -2.0), linked to `red_dispenser`. `red_dispenser` (6.411, -2.0).
    - `blue_turn` (5.4, -6.0), linked to `blue_dispenser`. `blue_dispenser` (6.411, -6.0).
    - `west` (-3.6, 0.0), linked to `center`, `station_1` and `station_2`.
    - `station_1` (-3.6, 5.7), `station_2` (-3.6, -3.2), `station_3` (1.0, -8.3), `station_4` (1.0, 5.2) and `station_5` (4.0, 0.5), each one in front of its delivery box.

**How a delivery is counted.** A ball counts as delivered when it lies inside its lit delivery box, the gripper is open and the ball has stayed still for 1 s. The box then flashes green and `getRedTarget()` / `getBlueTarget()` go back to `0`. Only one ball of each colour can be waiting at a time.

### Objects and targets

The balls (radius 1.5 cm) drop into a cup on the floor at the end of each dispenser.

- **red dispenser** — Pose (x,y) = (6.671, -2.0) m
- **blue dispenser** — Pose (x,y) = (6.671, -6.0) m

The five delivery boxes (22 x 22 cm, walls 6 cm tall with their top at z = 0.072) are on the floor.

- **box 1** — Pose (x,y) = (-3.6, 6.0) m
- **box 2** — Pose (x,y) = (-3.6, -3.5) m
- **box 3** — Pose (x,y) = (1.0, -8.6) m
- **box 4** — Pose (x,y) = (1.0, 5.5) m
- **box 5** — Pose (x,y) = (4.0, 0.8) m

### HAL-based Implementation in Python

- `import HAL_myagv as HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL.getPose3d()` - Returns a `Pose3d` object with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (quaternion `[w, x, y, z]`) and `timeStamp` (seconds).
- `HAL.worldToBase(x, y, z)` - Returns the world point `(x, y, z)` expressed in `base_footprint` as a tuple `(x, y, z)`, using the current base pose.
- `HAL.setV(velocity)` - to set the forward speed of the base in m/s.
- `HAL.setVY(velocity)` - to set the sideways speed of the base in m/s (positive to the left).
- `HAL.setW(velocity)` - to set the yaw rate of the base in rad/s (positive counterclockwise).

**Time and joints**

- `HAL.getSimTime()` - Returns the simulation time as an object with `seconds` and `nanoseconds` fields.
- `HAL.sleepSim(seconds)` - Waits for the given number of simulation seconds.
- `HAL.getJointPositions()` - Returns a `dict` with the last reported position in radians of every joint, by joint name.

**Sensors**

- `HAL.getLaserData()` - Returns a `LaserData` object of the 360º lidar, with `values` (360 distances in metres), `minAngle`, `maxAngle`, `minRange`, `maxRange` and `timeStamp`. The lidar is mounted rotated 180º with respect to the base.
- `HAL.getImage()` - Returns the front camera image as a BGR `numpy` array of 640x480.

**Delivery**

- `HAL.spawnRedBall()` / `HAL.spawnBlueBall()` - Drops a red / blue ball into its dispenser and lights a random delivery box with a marker of that colour.
- `HAL.getRedTarget()` / `HAL.getBlueTarget()` - Returns the number (`1` to `5`) of the delivery box where the red / blue ball must go, or `0` when there is no ball of that colour waiting.

**Inverse Kinematics**

- `HAL.moveArm(x, y, z, roll=HAL.GRASP_ROLL, pitch=0.0, yaw=HAL.GRASP_YAW, speed=0.3, motion="PTP")` - Moves the gripper centre to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. The default orientation looks forward, 70º below the horizon. `speed` is the MoveIt velocity scaling, between 0 and 1. `motion` is `"PTP"` for a point-to-point move or `"LIN"` for a linear trajectory. Returns `True` on success.
- `HAL.getArmPose()` - Returns the current gripper pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) in `base_footprint`, or `None` until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking. They return at once, so wait `duration` with `HAL.sleepSim(duration)` before the next arm command.

- `HAL.setArmJoints(positions, duration=3.0)` - Moves the mechArm 270 to the 6 joint angles in radians, in the order of `HAL.ARM_JOINTS`.
- `HAL.homeArm(duration=3.0)` - Folds the arm over the base.
- `HAL.readyArm(duration=3.0)` - Holds the gripper ahead of the base pitched down. Go here before reaching for the floor so the arm does not sweep into it.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.setGripper(closed, duration=1.0, target=None)` - `closed = True` closes the gripper and enables automatic attachment, `closed = False` opens it and releases the ball. `target` overrides the gripper angle in radians, between `-0.4` (closed) and `0.15` (open). Returns at once like the joint functions.

The objects that can be automatically grasped are the red and blue balls.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showDepthImage(depth)` - Shows a depth array in metres in the left panel as grayscale (closer is brighter, clipped at 4 m).

### HAL-based Implementation in C++

- `#include "HAL_myagv.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `#include "WebGUI.hpp"` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL::get_pose3d();` - Returns a `HAL::Pose3d` struct with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (`std::array<double, 4>` quaternion `[w, x, y, z]`) and `timeStamp` (seconds).
- `HAL::world_to_base(x, y, z);` - Returns the world point `(x, y, z)` expressed in `base_footprint` as a `std::array<double, 3>`, using the current base pose.
- `HAL::set_v(velocity);` - to set the forward speed of the base in m/s.
- `HAL::set_vy(velocity);` - to set the sideways speed of the base in m/s (positive to the left).
- `HAL::set_w(velocity);` - to set the yaw rate of the base in rad/s (positive counterclockwise).

**Time and joints**

- `HAL::get_sim_time();` - Returns the simulation time as a `HAL::SimTime` struct with `seconds` and `nanoseconds` fields.
- `HAL::sleep_sim(seconds);` - Waits for the given number of simulation seconds.
- `HAL::get_joint_positions();` - Returns a `std::map<std::string, double>` with the last reported position in radians of every joint, by joint name.

**Sensors**

- `HAL::get_laser_data();` - Returns a `HAL::LaserData` struct of the 360º lidar, with `values` (`std::vector<float>` of 360 distances in metres), `minAngle`, `maxAngle`, `minRange`, `maxRange` and `timeStamp`. The lidar is mounted rotated 180º with respect to the base.
- `HAL::get_image();` - Returns the front camera image as a BGR `cv::Mat` of 640x480.

**Delivery**

- `HAL::spawn_red_ball();` / `HAL::spawn_blue_ball();` - Drops a red / blue ball into its dispenser and lights a random delivery box with a marker of that colour.
- `HAL::get_red_target();` / `HAL::get_blue_target();` - Returns the number (`1` to `5`) of the delivery box where the red / blue ball must go as an `int`, or `0` when there is no ball of that colour waiting.

**Inverse Kinematics**

- `HAL::move_arm(x, y, z, roll, pitch, yaw, speed, motion);` - Moves the gripper centre to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. `roll` defaults to `HAL::GRASP_ROLL`, `pitch` to 0.0 and `yaw` to `HAL::GRASP_YAW`, which looks forward 70º below the horizon. `speed` (MoveIt velocity scaling, between 0 and 1) defaults to 0.3 and `motion` to `"PTP"` (point-to-point move), `"LIN"` gives a linear trajectory. Returns `true` on success.
- `HAL::get_arm_pose();` - Returns a `std::optional<HAL::ArmPose>` with the current gripper pose (`x`, `y`, `z`, `qx`, `qy`, `qz`, `qw`, `roll`, `pitch`, `yaw`) in `base_footprint`, empty until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking. They return at once, so wait `duration` with `HAL::sleep_sim(duration);` before the next arm command.

- `HAL::set_arm_joints(positions, duration);` - Moves the mechArm 270 to the 6 joint angles in radians of the `std::array<double, 6>` `positions`, in the order of `HAL::ARM_JOINTS`. `duration` defaults to 3.0 s.
- `HAL::home_arm(duration);` - Folds the arm over the base.
- `HAL::ready_arm(duration);` - Holds the gripper ahead of the base pitched down. Go here before reaching for the floor so the arm does not sweep into it.

The joint targets are also available as constants: `HAL::ARM_HOME_JOINTS` and `HAL::ARM_READY_JOINTS`.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL::set_gripper(closed, duration, target);` - `closed = true` closes the gripper and enables automatic attachment, `closed = false` opens it and releases the ball. `duration` defaults to 1.0 s. The optional `target` overrides the gripper angle in radians, between `HAL::GRIPPER_CLOSED` (`-0.4`) and `HAL::GRIPPER_OPEN` (`0.15`). Returns at once like the joint functions.

The objects that can be automatically grasped are the red and blue balls.

**WebGUI**

- `WebGUI::show_image(image);` - Shows a BGR `cv::Mat` in the right panel.
- `WebGUI::show_depth_image(depth);` - Shows a depth `cv::Mat` in metres in the left panel as grayscale (closer is brighter, invalid pixels are black).

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "HAL_myagv.hpp"
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

The node setup is described in [ROS 2-direct setup in Python](#ros-2-direct-setup-in-python) and [ROS 2-direct setup in C++](#ros-2-direct-setup-in-c). These are the interfaces of this robot:

- `/myagv_mecharm/odom` - Subscribe to this topic to receive the base pose in the world frame. Message type: `nav_msgs/msg/Odometry`

- `/myagv_mecharm/cmd_vel` - Publish to this topic to set the base velocities (`linear.x`, `linear.y` and `angular.z`). Message type: `geometry_msgs/msg/Twist`

- `/myagv_mecharm/scan` - Subscribe to this topic to receive the lidar scan. Message type: `sensor_msgs/msg/LaserScan`

- `/myagv_mecharm/camera/image_raw` - Subscribe to this topic to receive the front camera image. Message type: `sensor_msgs/msg/Image`

- `/myagv_mecharm/joint_states` - Subscribe to this topic to read the position of every joint. Message type: `sensor_msgs/msg/JointState`

- `/myagv_mecharm/robot_description` - Subscribe to this topic (QoS **TransientLocal**) to receive the URDF of the robot, with the joint origins and limits of the arm. Message type: `std_msgs/msg/String`

- `/myagv_mecharm/Robmove` - Action to move the gripper to a pose planned by MoveIt 2 (`type` is `"PTP"` or `"LIN"`, `speed` is the velocity scaling between 0 and 1, position `x`, `y`, `z` and quaternion `qx`, `qy`, `qz`, `qw`). Action type: `ros2srrc_data/action/Robmove`. Planned moves work better starting from the ready joint pose of [Argument examples](#argument-examples-2)

- `/myagv_mecharm/Robpose` - Subscribe to this topic to receive the current gripper pose. Message type: `ros2srrc_data/msg/Robpose`

- `/myagv_mecharm/arm_controller/follow_joint_trajectory` - Action to move the six mechArm 270 joints directly. Action type: `control_msgs/action/FollowJointTrajectory`

- `/myagv_mecharm/gripper_controller/follow_joint_trajectory` - Action to move `gripper_base_to_gripper_left3`. Action type: `control_msgs/action/FollowJointTrajectory`

- `/gripper_auto_attach` - Publish `true` while closing the gripper to enable the automatic attachment and `false` to release the ball. Message type: `std_msgs/msg/Bool`

- `/graspable_objects` - Publish to this topic the comma-separated names of the objects that can be attached (`red_ball,blue_ball`). Message type: `std_msgs/msg/String`

- `/warehouse_delivery/spawn_red_ball` and `/warehouse_delivery/spawn_blue_ball` - Publish to these topics to drop a red / blue ball into its dispenser. Message type: `std_msgs/msg/Empty`

- `/warehouse_delivery/red_target` and `/warehouse_delivery/blue_target` - Subscribe to these topics (QoS **TransientLocal**, depth 1) to receive the delivery box where the red / blue ball must go, or `0` when there is none. Message type: `std_msgs/msg/Int32`

### Argument examples

**Joint targets for the myAGV mechArm 270 (radians, in `HAL.ARM_JOINTS` order)**

```
home  = [0.0, -0.1, 0.2, 0.0, 1.5, 0.0]
ready = [0.0, 0.236, 0.255, 0.0, 0.739, 0.0]
```

---

## Theory

### Mobile manipulation

A mobile manipulator joins two problems that are usually solved separately: the base navigates in the plane (x, y, yaw) and the arm manipulates objects inside its own workspace, which is small and moves with the base. The object and target positions are known in the world frame, but the arm goals are given in the `base_footprint` frame, so they depend on where the base is.

### Relationship among ROS 2, MoveIt 2, Gazebo and the JdeRobot API

- [ROS](https://www.ros.org/) (Robot Operating System) is a robotics middleware which contains a set of open source libraries for developing robot applications.
- [MoveIt](https://moveit.ros.org/) is an open source Motion Planning framework for industrial robots which integrates motion planning, collision checking, manipulation and 3D perception capabilities.
- [Gazebo](https://gazebosim.org/) is a physics simulator mainly used for robot simulation.
- The HAL API provided by JdeRobot Robotics Academy is built on top of the above tools and the IFRA Cranfield repositories (see References), so you do not need to learn all of them to start simulating mobile manipulation.

## Hints


Simple hints to help you solve the Mobile Manipulation exercise.

### Where to insert and run the code

In the launched web page, type your code in the text editor and run it by pressing the play button. Import the HAL of the robot of the selected world:

```python
import HAL_xlerobot as HAL
# Enter sequential code here!

while True:
    # Enter iterative code here!
```

In C++, include the HAL header of the robot and write your code in `exercise()`, as shown in the C++ part of each robot section.

### Why does the robot sometimes fail to move to a desired pose?

The most likely reason is that your specified pose is unreachable for the robot arm, so MoveIt cannot plan a trajectory from the current pose to the desired pose within the allowed time. You will see a warning like this when that happens:

```bash
Fail: ABORTED: No motion plan found. No execution attempted.
```

### Why does a planned move fail right after a joint move?

The joint functions (`setArmJoints`, `homeArm`, `readyArm`, `setGripper` and the XLeRobot equivalents) return at once. If a planned move starts while the arm is still moving, MoveIt rejects the trajectory because the arm is not where it expected. Wait the `duration` of the joint move with `HAL.sleepSim(duration)` before the next arm command.


## Videos

### MMO-500 Warehouse

{% include youtubePlayer.html id=page.youtubeId2 %}

### XLeRobot Warehouse

{% include youtubePlayer.html id=page.youtubeId1 %}

## References

1. Neobotix ROS 2 simulation. [https://github.com/neobotix/neo_simulation2](https://github.com/neobotix/neo_simulation2)
2. XLeRobot. [https://github.com/Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot)
3. Elephant Robotics myAGV and mechArm. [https://www.elephantrobotics.com/](https://www.elephantrobotics.com/)
4. IFRA-Cranfield (2023). ROS 2 Sim-to-Real Robot Control. [https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
5. [https://moveit.ros.org/](https://moveit.ros.org/)
6. [https://gazebosim.org/](https://gazebosim.org/)
