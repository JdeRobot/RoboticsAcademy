---
permalink: /exercises/IndustrialRobots/pick_place
title: "Pick and Place"

sidebar:
  nav: "docs"

toc: true
toc_label: "Pick and Place exercise"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/pick_place/pick_place_ur_world.png
    image_path: /assets/images/exercises/pick_place/pick_place_ur_world.png
    alt: "Pick and Place"
    title: "Pick and Place"

workspace:
  - url: /assets/images/exercises/pick_place/irb120_workspace.jpg
    image_path: /assets/images/exercises/pick_place/irb120_workspace.jpg
    alt: "IRB120 Workspace"
    title: "IRB120 Workspace"

rpy000:
  - url: /assets/images/exercises/pick_place/rpy000.png
    image_path: /assets/images/exercises/pick_place/rpy000.png
    alt: "rpy000"
    title: "rpy000"

rpy:
  - url: /assets/images/exercises/pick_place/roll90.png
    image_path: /assets/images/exercises/pick_place/roll90.png
    alt: "roll90"
    title: "roll90"
  - url: /assets/images/exercises/pick_place/pitch90.png
    image_path: /assets/images/exercises/pick_place/pitch90.png
    alt: "pitch90"
    title: "pitch90"
  - url: /assets/images/exercises/pick_place/yaw90.png
    image_path: /assets/images/exercises/pick_place/yaw90.png
    alt: "yaw90"
    title: "yaw90"

youtubeId: kJMPz80w9BM
---
## Goal

The goal of this exercise is to learn the underlying infrastructure of Industrial Robot exercises(ROS + MoveIt + our industrial robotics HAL API) and get familiar with the key components needed for more complex exercises by completing the task of pick and place multiple objects and classify them by color or shape.

{% include gallery caption="Gallery." %}

## Frequency API

* `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
* `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

<!-- This exercise now supports ROS 2-direct implementation in addition to the original HAL-based approach. Below you'll find the details for both options. -->

## HAL API for Gazebo 11 (Classic).

To use this HAL API, you need to import the HAL module:

```python
import HAL
```

### Direct Kinematics

* `HAL.MoveAbsJ(absolute_joints, rel_speed, wait_time)`
  * Moves the robot to the given angular position for each joint (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.
* `HAL.MoveSingleJ("jointX", value_in_deg, rel_speed, wait_time)`
  * Moves the joint X [0 to 6] of the robot to the given angle value (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.

### Inverse Kinematics

#### Using absolute poses

* `HAL.MoveJoint(absolute_XYZ, absolute_YPR, rel_speed, wait_time)`
  * Moves the robot Tool Center Point (TCP) to an absolute (X,Y,Z) pose with an absolute orientation (in degrees) of (Yaw,Pitch,Roll), at a given relative speed in the range [0-1], adding a final delay in seconds. The robot will move at constant rotational speeds in each joint, resulting in a non-linear trajectory
* `HAL.MoveLinear (absolute_XYZ, absolute_YPR, rel_speed, wait_time)`
  * Moves the robot TCP to an absolute (X,Y,Z) pose with an absolute orientation (in degrees) of (Yaw,Pitch,Roll) in a linear trajectory, at a given relative speed in the range [0-1], adding a final delay in seconds.

#### Using relative XYZ or YPR increments

* `HAL.MoveRelLinear(increment_XYZ, rel_speed, wait_time)`
  * Moves the robot TCP pose in a linear trajectory by the distances given in increment_XYZ argument, at a given relative speed in the range [0-1], adding a final delay in seconds. The tool orientation is not changed.
* `HAL.MoveRelReor (increment_YPR, rel_speed, wait_time)`
  * Reorients the robot TCP by the angular increments given in increment_YPR argument (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds. The TCP (x,y,z) stay fixed.

### Gripper usage

* `HAL.GripperSet(percentage_closure, wait_time)`
  * Controls the two-finger gripper opening.
    * 0 → fully open  
    * 100 → fully closed  
  * When the gripper is opened (typically ≤ 5%), any attached object is automatically detached.

* `HAL.attach(object_name)`
  * Attaches the specified object to the gripper using the Gazebo LinkAttacher ROS2 service.
  * The object must be one of the predefined simulation objects.

* `HAL.dettach()`
  * Detaches the currently attached object from the gripper using the LinkAttacher service.
  * If no object is attached, the function does nothing.

* `Notes:`
  * Only one object can be attached at a time.
  * The attach/detach operations rely on ROS2 service calls and are not instantaneous.

### Argument examples

#### Example of data targets for MoveAbsJ (angular position for each joint, in deg)

absj_10 = [0, -90, 45, -135, -90, 0]

absj_20 = [-45, -90, 90, -90, -90, 90]

absj_30 = [20, -90, 45, -45, -90, 0]

#### Examples of absolute XYZ poses for MoveJoint and MoveLinear (in meters, from the world frame)

pose_10 = [0.5, 0, 1.3]

pose_20 = [0, 0.5, 1.3]

#### Examples of absolute YPR angular poses or MoveJoint and MoveLinear (in degrees)

YPR_10 = [0, 90, 90]

YPR_20 = [0, 90, 0]

#### Examples of Cartesian increments for relative MoveRelLinear, [Ax,Ay,Az] in meters)

increment_10 = [0.3, 0.1, -0.2]

increment_20 = [-0.4, -0.1, 0.4]

## HAL API for Gazebo Harmonic.

To use this HAL API, you need to import the HAL_Harmonic module:

```python
import HAL_Harmonic
```

### Direct Kinematics

* `HAL.MoveAbsJ(joints_deg, speed, wait_time)`
  * Moves the robot to the given angular position for each joint (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.
* `HAL.MoveSingleJ("jointX", value_in_deg, rel_speed, wait_time)`
  * Moves a single joint by a relative angular increment (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.s

### Inverse Kinematics

#### Using absolute poses

* `HAL.MoveJoint(abs_xyz, abs_ypr, speed, wait_time)`
  * Moves the robot Tool Center Point (TCP) to an absolute (X,Y,Z) pose with an absolute orientation (Roll,Pitch,Yaw in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.
* `HAL.MoveLinear (absolute_XYZ, absolute_YPR, rel_speed, wait_time)`
  * Moves the robot TCP to an absolute (X,Y,Z) pose with an absolute orientation (Roll,Pitch,Yaw in degrees) following a linear trajectory, at a given relative speed in the range [0-1], adding a final delay in seconds.

#### Using relative XYZ or YPR increments

* `HAL.MoveRelLinear(relative_xyz, speed, wait_time)`
  * Moves the robot TCP in Cartesian space by the given relative displacement (in meters), maintaining orientation.
* `HAL.MoveRelReor (relative_rpy, speed, wait_time)`
  * Reorients the robot TCP by relative angular increments (Roll,Pitch,Yaw in degrees), keeping the TCP position fixed.

### Gripper usage

In Gazebo Harmonic, the gripper is controlled through the joint trajectory controller and object attachment is managed through the Gazebo LinkAttacher service.

* `HAL.GripperSet(percentage_closure, wait_time)`
  * Closes the two-finger gripper to the closing percentage given in the first argument, adding a final delay in seconds. A percentage_closure of 100 means fully closed, 0 means fully opened.

* `HAL.attach(object_name)`
  * Attaches the given object_name to the gripper using the Gazebo LinkAttacher service. Precise object names are given below.

* `HAL.dettach()`
  * Dettaches the attached object from the gripper using the Gazebo LinkAttacher service. No argument is needed. When the gripper is fully opened with `HAL.GripperSet(0, wait_time)` an automatic dettach is performed.

<!-- ### ROS 2-direct Implementation

Use standard ROS 2 topics for direct communication with the simulation.

- `/compute_ik` - Service used to compute inverse kinematics for a Cartesian pose.  
  Service type: `moveit_msgs/srv/GetPositionIK`

- `/joint_trajectory_controller/follow_joint_trajectory` - Action used to move the arm joints.  
  Action type: `control_msgs/action/FollowJointTrajectory`

- `/gripper_controller/follow_joint_trajectory` - Action used to control the gripper joint.  
  Action type: `control_msgs/action/FollowJointTrajectory`

- `/ATTACHLINK` - Service used to attach an object to the gripper in simulation.  
  Service type: `linkattacher_msgs/srv/AttachLink`

- `/DETACHLINK` - Service used to detach an object from the gripper in simulation.  
  Service type: `linkattacher_msgs/srv/DetachLink`

- `/move_action` - MoveIt action server available in the system.  
  Action type: `moveit_msgs/action/MoveGroup`

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control. ecution pipeline.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution. -->

### Argument examples

#### Example of data targets for MoveAbsJ (angular position for each joint, in deg)

absj_home = [0, -90, 70, -70, -90, 0]

#### Examples of absolute XYZ poses for MoveJoint (in meters, from the world frame)

aprox_yellow_box = [0.6, 0.3, 0.4]

aprox_yellow_target = [-0.4, -0.45, 0.4]

#### Examples of absolute YPR angular poses for MoveJoint (in degrees)

YPR_pick = [180, 0, -90]

YPR_place = [0, 90, 0]

#### Examples of Cartesian increments for relative MoveRelLinear, [Ax,Ay,Az] in meters

decrease_z_10 = [0, 0, -0.10]

decrease_z_15 = [0, 0, -0.15]

increase_z_20 = [0, 0, 0.20]

increase_z_25 = [0, 0, 0.25]

## Where to insert and run the code?

In the launched webpage, type your code in the text editor and run it pressing the run button:

```python
import HAL
# Enter sequential code here!

while True:
    # Enter iterative code here!
```

### Why does the robot sometimes cannot move to some desired pose?

The most possible reason is that your specified pose is unreachable for the robot arm, so MoveIt cannot plan a trajectory from current pose to desired pose in limited time. You will see such a warning when this problem happened:

```bash
Fail: ABORTED: No motion plan found. No execution attempted.
```

## Object and Target lists, dimensions and poses

### Gazebo 11 (Classic)

**Object list.** The four objects are located on a conveyor that is 1 m tall.
* **yellow_box**
  * Size (l,w,h) = (7,5,10) cm
  * Pose (x,y) = (0.6,0.3) m
* **red_box**
  * Size (l,w,h) = (5,10,8) cm
  * Pose (x,y) = (0.6,-0.3) m
* **blue_ball**
  * Size (r) = (4) cm
  * Pose (x,y) = (0.7, 0.1) m
* **green_cylinder**
  * Size (r,h) = (4,15) cm
  * Pose (x,y) = (0.5, -0.1) m

**Target list.** The four targets are located on a table that is 0.8 m tall.
* **red_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,0.15) m
* **green_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,-0.15) m
* **blue_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,0.45) m
* **yellow_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,-0.45) m

### Gazebo Harmonic

**Object list.** The four objects are located on a conveyor that is 1 m tall.  
Object dimensions and positions are the same as in the Gazebo Classic version.

* **yellow_box**
  * Size (l,w,h) = (7,5,10) cm
  * Pose (x,y) = (0.6,0.3) m
* **red_box**
  * Size (l,w,h) = (5,10,8) cm
  * Pose (x,y) = (0.6,-0.3) m
* **blue_sphere**
  * Size (r) = (4) cm
  * Pose (x,y) = (0.7, 0.1) m
* **green_cylinder**
  * Size (r,h) = (4,15) cm
  * Pose (x,y) = (0.5, -0.1) m

**Target list.** The four targets are located on a table that is 0.8 m tall.  
Target dimensions and positions are identical to the Gazebo Classic version.

* **red_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,0.15) m
* **green_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,-0.15) m
* **blue_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,0.45) m
* **yellow_target**
  * Size (l,w,h) = (36,30,12) cm
  * Pose (x,y) = (-0.6,-0.45) m
  
## Hints

### Relationship among ROS2, MoveIt2, Gazebo, JdeRobot provided API

- [ROS](https://www.ros.org/)(Robot Operating System) is a robotics middleware which contains a set of open source libraries for developing robot applications.
* [MoveIt](https://moveit.ros.org/) is an open source Motion Planning framework for industrial robot which integrates motion planning, collision checking, manipulation, 3D perception capabilities.
* [Rviz](http://wiki.ros.org/rviz) is a 3D visualization tool for ROS. Many ROS topics can be visualized in Rviz, including the planning scene of MoveIt move group, but it does not contain any physics simulation capability.
* [Gazebo](http://gazebosim.org/) is a physics simulator mainly use for robot simulation.
* The HAL API provided by JdeRobot Robotics Academy is based on the above tools and the IFRA Cranfield repositories (see Acknowledgements), so the user don't need to learn all of them to start simulation of industrial robot manipulation.

## Videos

### Demonstration video of the solution

{% include youtubePlayer.html id=page.youtubeId %}

## Acknowledgements

This exercise uses the excellent work of the IFRA Cranfield Group. Check out their repos at:
IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: <https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl>.
