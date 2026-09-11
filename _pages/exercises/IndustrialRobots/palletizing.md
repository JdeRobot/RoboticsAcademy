---
permalink: /exercises/IndustrialRobots/palletizing
title: "Palletizing"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Palletizing"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->
---

## Goal

The goal of this exercise is to program a robot arm that picks up boxes arriving on a conveyor and stacks them on a pallet.

A feeder spawns boxes one at a time and carries them down the conveyor. Once a box stops at the pickup point, the exercise announces it and hands you its size, mass and observed pose. You pick it up with the arm's suction cup, choose an in-bounds spot on the pallet, place it there, and tell the feeder you are done so the next box can start.

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

**Direct Kinematics**

- `HAL.MoveAbsJ(absolute_joints, speed, wait_time)` - Moves the robot to the given angular position for each joint (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds. `absolute_joints` is a list of 6 joint angles.
- `HAL.MoveSingleJ(joint_number, relative_angle, speed, wait_time)` - Moves the joint `joint_number` [1 to 6] by a relative angular increment (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.

**Inverse Kinematics**

- `HAL.MoveJoint(absolute_XYZ, absolute_YPR, speed, wait_time)` - Moves the robot Tool Center Point (TCP) to an absolute (X,Y,Z) pose (in metres) with an absolute orientation (Yaw,Pitch,Roll in degrees). The robot moves point-to-point, resulting in a non-linear trajectory.
- `HAL.MoveLinear(absolute_XYZ, absolute_YPR, speed, wait_time)` - Moves the robot TCP to an absolute (X,Y,Z) pose (in metres) with an absolute orientation (Yaw,Pitch,Roll in degrees) following a linear trajectory.
- `HAL.MoveRelLinear(increment_XYZ, speed, wait_time)` - Moves the robot TCP in a linear trajectory by the Cartesian displacement given in `increment_XYZ` (in metres). The tool orientation is not changed. Blocks until the joints settle, on top of `wait_time`.
- `HAL.MoveRelReor(increment_YPR, speed, wait_time)` - Reorients the robot TCP by the angular increments given in `increment_YPR` (Yaw,Pitch,Roll in degrees). The TCP position stays fixed.

**Suction gripper**

The gripper is a suction cup that grasps and releases boxes automatically through a contact-based attachment system, so no manual attach/detach calls are required.

- `HAL.SuctionSet(on, wait_time)` - `on = True` energizes the suction cup, so any graspable box touching it gets attached. `on = False` de-energizes it, releasing any attached box. `wait_time` in seconds.

**Box feeder**

- `HAL.WaitForBox()` - Blocks until a box is stopped at the pickup point, returns its feeder name (`str`).
- `HAL.GetBoxInfo(name, timeout=5.0)` - Returns a `dict` with the box's semantic task data: `name`, `sku`, `size` (`[length, width, height]` in metres) and `mass` (kg).
- `HAL.GetPickupPose(name, timeout=5.0)` - Returns a `dict` with the box's observed top-center pose in the robot's `base_link` frame: `frame` (always `"base_link"`), `center` (`[x, y, z]` in metres) and `top_z` (the Z of the box's top face, `center[2] + height / 2`).
- `HAL.GetPalletInfo(timeout=5.0)` - Returns a `dict` with the pallet's placement area in the `base_link` frame: `frame`, `size`, `usable_size`, `center` (`[x, y, z]`) and `top_z` (the Z of the pallet's deck surface).
- `HAL.BoxDone(name)` - Acknowledges that the box is clear of the conveyor, so the feeder can spawn and start moving the next one.

#### C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).

**Direct Kinematics**

- `HAL::MoveAbsJ(joints, speed, wait_time);` - Moves the robot to the given angular position for each joint. `joints` is `std::array<double, 6>` in degrees, `speed` in [0,1], `wait_time` in seconds.
- `HAL::MoveSingleJ(joint_number, relative_angle, speed, wait_time);` - Moves a single joint by a relative angular increment. `joint_number` in [1,6], angle in degrees.

**Inverse Kinematics**

- `HAL::MoveJoint(xyz, ypr, speed, wait_time);` - Moves the TCP to an absolute Cartesian pose. `xyz` is `std::array<double, 3>` in metres, `ypr` in degrees.
- `HAL::MoveLinear(xyz, ypr, speed, wait_time);` - Moves the TCP in a linear trajectory to an absolute Cartesian pose. `xyz` in metres, `ypr` in degrees.
- `HAL::MoveRelLinear(xyz, speed, wait_time);` - Moves the TCP by a relative Cartesian increment. `xyz` is `std::array<double, 3>` in metres. Blocks until the joints settle, on top of `wait_time`.
- `HAL::MoveRelReor(ypr, speed, wait_time);` - Reorients the TCP by relative angular increments. `ypr` is `std::array<double, 3>` in degrees.

**Suction gripper**

- `HAL::SuctionSet(on, wait_time);` - `on = true` energizes the suction cup, `on = false` de-energizes it. `wait_time` in seconds.

**Box feeder**

- `HAL::WaitForBox();` - Blocks until a box is stopped at the pickup point, returns its feeder name (`std::string`).
- `HAL::GetBoxInfo(name, timeout);` - Returns a `nlohmann::json` object with the box's semantic task data: `name`, `sku`, `size` (`[length, width, height]` in metres) and `mass` (kg). `timeout` defaults to 5.0 s.
- `HAL::GetPickupPose(name, timeout);` - Returns a `nlohmann::json` object with the box's observed top-center pose in the robot's `base_link` frame: `frame`, `center` (`[x, y, z]`) and `top_z`. `timeout` defaults to 5.0 s.
- `HAL::GetPalletInfo(timeout);` - Returns a `nlohmann::json` object with the pallet's placement area: `frame`, `size`, `usable_size`, `center` and `top_z`. `timeout` defaults to 5.0 s.
- `HAL::BoxDone(name);` - Acknowledges that the box is clear of the conveyor, so the feeder can spawn the next one.

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "HAL.hpp"
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

Use standard ROS 2 topics and actions for direct communication with the simulation.

- `/Move` - Action used for `MoveAbsJ`, `MoveSingleJ` and `MoveRelLinear`. Action type: `ros2srrc_data/action/Move`

- `/Robmove` - Action used for `MoveJoint` and `MoveLinear`. Action type: `ros2srrc_data/action/Robmove`

- `/joint_states` - Subscribe to this topic to read joint velocities, used to detect that a `MoveRelLinear` motion has settled. Message type: `sensor_msgs/msg/JointState`

- `/gripper_auto_attach` - Publish to this topic to energize (`true`) or de-energize (`false`) the suction cup. Message type: `std_msgs/msg/Bool`

- `/graspable_objects` - Publish to this topic to tell the gripper plugin which object names are graspable (substring match, the feeder generates names containing `box`). Message type: `std_msgs/msg/String`

For the box feeder:

- `/box_ready` - Subscribe to this topic for the feeder name of the box currently stopped at the pickup point. Message type: `std_msgs/msg/String`

- `/box_info` - Subscribe to this topic for the box's semantic task data and observed pickup pose, as a JSON string. Message type: `std_msgs/msg/String`

- `/pallet_info` - Subscribe to this topic for the pallet's placement area, as a JSON string. Message type: `std_msgs/msg/String`

- `/box_done` - Publish the box's feeder name to this topic once it is clear of the conveyor, so the feeder can spawn the next one. Message type: `std_msgs/msg/String`

#### Python

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

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

Simple hints to help you solve the Palletizing exercise.

### Where to insert and run the code

In the launched web page, type your code in the text editor and run it by pressing the play button:

```python
import HAL
# Enter sequential code here!

while True:
    # Enter iterative code here!
```

### Why does the robot sometimes fail to move to a desired pose?

The most likely reason is that your specified pose is unreachable for the robot arm, so MoveIt cannot plan a trajectory from the current pose to the desired pose within the allowed time. You will see a warning like this when that happens:

```bash
Fail: ABORTED: No motion plan found. No execution attempted.
```

### Where do I grab and place the box?

Aim the descent at `pickup["center"][0]`, `pickup["center"][1]` and `pickup["top_z"]` to touch the top face of the box with the suction cup. When placing, target the pallet at your chosen (x, y) within `pallet["usable_size"]` of `pallet["center"]`, at `pallet["top_z"] + stacked_height`.

## Contributors

- Contributors: [Ashwani1330](https://github.com/Ashwani1330) and [Javier Izquierdo](https://github.com/javizqh).

## References

1. IFRA-Cranfield (2023). ROS 2 Sim-to-Real Robot Control. [https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
2. [https://moveit.ros.org/](https://moveit.ros.org/)
3. [https://gazebosim.org/](https://gazebosim.org/)
