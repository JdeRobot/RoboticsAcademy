---
permalink: /exercises/IndustrialRobots/pick_place
title: "Pick and Place"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Pick and Place"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/pick_place/pick_place_ur_world.png
    image_path: /assets/images/exercises/pick_place/pick_place_ur_world.png
    alt: "UR5 Pick and Place world"
    title: "UR5 Pick and Place world"
  - url: /assets/images/exercises/pick_place/dobot_suction_cup_world.jpg
    image_path: /assets/images/exercises/pick_place/dobot_suction_cup_world.jpg
    alt: "Dobot Magician with the suction cup"
    title: "Dobot Magician with the suction cup"
  - url: /assets/images/exercises/pick_place/dobot_real.jpg
    image_path: /assets/images/exercises/pick_place/dobot_real.jpg
    alt: "Real Dobot Magician"
    title: "Real Dobot Magician"

rpy000:
  - url: /assets/images/exercises/pick_place/rpy000.png
    image_path: /assets/images/exercises/pick_place/rpy000.png
    alt: "TCP frame at YPR = (0, 0, 0)"
    title: "TCP frame at YPR = (0, 0, 0)"

rpy:
  - url: /assets/images/exercises/pick_place/roll90.png
    image_path: /assets/images/exercises/pick_place/roll90.png
    alt: "Roll = 90"
    title: "Roll = 90"
  - url: /assets/images/exercises/pick_place/pitch90.png
    image_path: /assets/images/exercises/pick_place/pitch90.png
    alt: "Pitch = 90"
    title: "Pitch = 90"
  - url: /assets/images/exercises/pick_place/yaw90.png
    image_path: /assets/images/exercises/pick_place/yaw90.png
    alt: "Yaw = 90"
    title: "Yaw = 90"

dobot:
  - url: /assets/images/exercises/pick_place/dobot_gripper_world.jpg
    image_path: /assets/images/exercises/pick_place/dobot_gripper_world.jpg
    alt: "Dobot Pick And Place World"
    title: "Dobot Pick And Place World"
  - url: /assets/images/exercises/pick_place/dobot_suction_cup_world.jpg
    image_path: /assets/images/exercises/pick_place/dobot_suction_cup_world.jpg
    alt: "Dobot Suction Cup Pick And Place World"
    title: "Dobot Suction Cup Pick And Place World"


youtubeId: kJMPz80w9BM
youtubeId2: iZGmwDLkJz8
---

## Goal

The goal of this exercise is to learn the underlying infrastructure of the Industrial Robot exercises (ROS 2 + MoveIt 2 + our industrial robotics HAL API) and to get familiar with the key components needed for more complex exercises.

{% include gallery caption="Gallery." %}

The student will program a robot arm to pick and place several objects and classify them by colour on the corresponding targets. The exercise runs on **Gazebo Harmonic**, and the same code also moves a real UR5 / UR5e or a real Dobot Magician.

The exercise offers five worlds. Choose the world in the exercise page and read the section of its robot.

- [**UR5**](#ur5): a UR5 arm with a Robotiq 2F-85 gripper picks four objects from a conveyor and leaves them on the trays of their colour.
  - **Pick And Place World**, in simulation.
  - **Ur5 Physical**, the real UR5 or UR5e, see [Running on the real robots](#running-on-the-real-robots).
- [**Dobot Magician**](#dobot-magician): a desktop 4-joint arm picks three cubes from the table and leaves them on the mats of their colour.
  - **Dobot Pick And Place World**, in simulation with the parallel gripper.
  - **Dobot Suction Cup Pick And Place World**, in simulation with the suction cup.
  - **Dobot Magician Physical**, the real Dobot with the suction cup, see [Running on the real robots](#running-on-the-real-robots).

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

### C++

- `#include "Frequency.hpp"` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency freq = Frequency();` - to instanciate the Frequency class.
- `freq.tick(ideal_rate);` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

The robot is controlled through the HAL-based approach, in Python and C++. Each arm has its own HAL module, and the simulated and the real robot of the same arm share it.

| World | Robot | Python | C++ |
| ----- | ----- | ------ | --- |
| Pick And Place World, Ur5 Physical | [UR5](#ur5) | `import HAL` | `#include "HAL.hpp"` |
| Dobot Pick And Place World, Dobot Suction Cup Pick And Place World, Dobot Magician Physical | [Dobot Magician](#dobot-magician) | `import HAL_dobot as HAL` | `#include "HAL_dobot.hpp"` |

---

## UR5
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/pick_place/pick_place_world.png" alt="UR5 Pick And Place World" caption="The UR5 between the conveyor with the objects and the table with the targets." %}

A UR5 arm with a Robotiq 2F-85 gripper has to pick four objects from a conveyor and classify them by colour on the corresponding target trays.

### HAL-based Implementation in Python

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo or the real robot).

**Direct Kinematics**

- `HAL.MoveAbsJ(absolute_joints, speed, wait_time)` - Moves the robot to the given angular position for each joint (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds. `absolute_joints` is a list of 6 joint angles.
- `HAL.MoveSingleJ(joint_number, relative_angle, speed, wait_time)` - Moves the joint `joint_number` [1 to 6] by a relative angular increment (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.

**Inverse Kinematics**

- `HAL.MoveJoint(absolute_XYZ, absolute_YPR, speed, wait_time)` - Moves the robot Tool Center Point (TCP) to an absolute (X,Y,Z) pose (in metres) with an absolute orientation (Yaw,Pitch,Roll in degrees). The robot moves point-to-point, resulting in a non-linear trajectory.
- `HAL.MoveLinear(absolute_XYZ, absolute_YPR, speed, wait_time)` - Moves the robot TCP to an absolute (X,Y,Z) pose (in metres) with an absolute orientation (Yaw,Pitch,Roll in degrees) following a linear trajectory.
- `HAL.MoveRelLinear(increment_XYZ, speed, wait_time)` - Moves the robot TCP in a linear trajectory by the Cartesian displacement given in `increment_XYZ` (in metres). The tool orientation is not changed.
- `HAL.MoveRelReor(increment_YPR, speed, wait_time)` - Reorients the robot TCP by the angular increments given in `increment_YPR` (Yaw,Pitch,Roll in degrees). The TCP position stays fixed.

**Robot state**

- `HAL.get_Joint_states()` - Returns the current joint angles in degrees, as a list of 6 values in the order `MoveAbsJ` takes them.
- `HAL.get_TCP_pose()` - Returns the current TCP pose as a tuple `(xyz, ypr)`, in the same frame, units and angle order `MoveLinear` takes, so it can be passed back to it.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.GripperSet(percentage_closure, wait_time)` - Closes (`100`) or opens (`0`) the two-finger gripper to the given closing percentage, adding a final delay in seconds. When it starts closing (`> 5`) automatic attachment is enabled; when it opens (`<= 5`) any attached object is automatically detached.

The objects that can be automatically grasped are: `blue_ball`, `green_cylinder`, `yellow_box` and `red_box`.

### HAL-based Implementation in C++

- `#include "HAL.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo or the real robot).

**Direct Kinematics**

- `HAL::MoveAbsJ(joints, speed, wait_time);` - Moves the robot to the given angular position for each joint. `joints` is `std::array<double, 6>` in degrees, `speed` in [0,1], `wait_time` in seconds.
- `HAL::MoveSingleJ(joint_number, relative_angle, speed, wait_time);` - Moves a single joint by a relative angular increment. `joint_number` in [1,6], angle in degrees.

**Inverse Kinematics**

- `HAL::MoveJoint(xyz, ypr, speed, wait_time);` - Moves the TCP to an absolute Cartesian pose. `xyz` is `std::array<double, 3>` in metres, `ypr` in degrees.
- `HAL::MoveLinear(xyz, ypr, speed, wait_time);` - Moves the TCP in a linear trajectory to an absolute Cartesian pose. `xyz` in metres, `ypr` in degrees.
- `HAL::MoveRelLinear(xyz, speed, wait_time);` - Moves the TCP by a relative Cartesian increment. `xyz` is `std::array<double, 3>` in metres.
- `HAL::MoveRelReor(ypr, speed, wait_time);` - Reorients the TCP by relative angular increments. `ypr` is `std::array<double, 3>` in degrees.

**Robot state**

- `HAL::get_Joint_states();` - Returns the current joint angles in degrees as `std::array<double, 6>`, in the order `MoveAbsJ` takes them.
- `HAL::get_TCP_position();` - Returns the current TCP position in metres as `std::array<double, 3>`, in the frame `MoveLinear` takes.
- `HAL::get_TCP_orientation();` - Returns the current TCP orientation in degrees as `std::array<double, 3>`, in the angle order `MoveLinear` takes.

**Gripper**

- `HAL::GripperSet(relative_closure, wait_time);` - Controls the gripper. `relative_closure` in [0,100] (0 = fully open, 100 = fully closed). When closing (`> 5`) contact-based automatic attachment is enabled; when opening (`<= 5`) the attached object is automatically released.

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

### Argument examples

**Data targets for `MoveAbsJ` (joint angles, in degrees)**

```
absj_home = [0, -90, 70, -70, -90, 0]
```

**Absolute XYZ poses for `MoveJoint` and `MoveLinear` (in metres, world frame)**

```
aprox_yellow_box    = [0.6, 0.3, 0.4]
aprox_yellow_target = [-0.4, -0.45, 0.4]
```

**Absolute YPR orientations for `MoveJoint` and `MoveLinear` (in degrees)**

```
YPR_pick  = [180, 0, -90]
YPR_place = [0, 90, 0]
```

**Cartesian increments for `MoveRelLinear`, [Ax, Ay, Az] in metres**

```
decrease_z_10 = [0, 0, -0.10]
increase_z_20 = [0, 0,  0.20]
```

### Objects and targets

**Object list.** The four objects are located on a conveyor that is 1 m tall.

- **yellow_box** — Size (l,w,h) = (7,5,10) cm — Pose (x,y) = (0.6, 0.3) m
- **red_box** — Size (l,w,h) = (5,10,8) cm — Pose (x,y) = (0.6, -0.3) m
- **blue_ball** — Size (r) = (4) cm — Pose (x,y) = (0.7, 0.1) m
- **green_cylinder** — Size (r,h) = (4,15) cm — Pose (x,y) = (0.5, -0.1) m

**Target list.** The four targets are located on a table that is 0.8 m tall.

- **red_target** — Pose (x,y) = (-0.4, 0.15) m
- **green_target** — Pose (x,y) = (-0.4, -0.15) m
- **blue_target** — Pose (x,y) = (-0.4, 0.45) m
- **yellow_target** — Pose (x,y) = (-0.4, -0.45) m

---

## Dobot Magician
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include gallery id="dobot" caption="Dobot Pick And Place World with the parallel gripper (left) and Dobot Suction Cup Pick And Place World (right)." %}

The [Dobot Magician](https://www.dobot-robots.com/products/education/magician.html), a desktop 4-joint arm, has to pick three cubes from the table and leave each one on the mat of its colour. It comes with two tools: a parallel gripper and a suction cup. The real robot runs with the suction cup.

### The robot

- The arm base is at the origin of the world, on the table, with `x` forward and `y` to the left.
- Arm joints, in order: `joint_1` (base, [-125, 125] degrees), `joint_2` (rear arm, [-5, 90] degrees), `joint_3` (forearm, [-15, 70] degrees) and `joint_4` (tool rotation, [-150, 150] degrees). The tool always points down.
- The parallel gripper is driven by `joint_prismatic_l`: `-0.018` m open, `0.0` closed.

### HAL-based Implementation in Python

- `import HAL_dobot as HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo or the real robot).

**Direct Kinematics**

- `HAL.MoveAbsJ(absolute_joints, speed, wait_time)` - Moves the robot to the given angular position for each joint (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds. `absolute_joints` is a list of 4 joint angles.
- `HAL.MoveSingleJ(joint_number, relative_angle, speed, wait_time)` - Moves the joint `joint_number` [1 to 4] by a relative angular increment (in degrees), at a given relative speed in the range [0-1], adding a final delay in seconds.

**Inverse Kinematics**

- `HAL.MoveJoint(absolute_XYZ, absolute_YPR, speed, wait_time)` - Moves the robot TCP to an absolute (X,Y,Z) pose (in metres) with an absolute orientation (Yaw,Pitch,Roll in degrees). The robot moves point-to-point, resulting in a non-linear trajectory.
- `HAL.MoveLinear(absolute_XYZ, absolute_YPR, speed, wait_time)` - Moves the robot TCP to an absolute (X,Y,Z) pose (in metres) with an absolute orientation (Yaw,Pitch,Roll in degrees) following a linear trajectory.
- `HAL.MoveRelLinear(increment_XYZ, speed, wait_time)` - Moves the robot TCP in a linear trajectory by the Cartesian displacement given in `increment_XYZ` (in metres). The tool orientation is not changed.
- `HAL.MoveRelReor(increment_YPR, speed, wait_time)` - Reorients the robot TCP by the angular increments given in `increment_YPR` (Yaw,Pitch,Roll in degrees). The TCP position stays fixed.

**Robot state**

- `HAL.get_Joint_states()` - Returns the current joint angles in degrees, as a list of 4 values in the order `MoveAbsJ` takes them.
- `HAL.get_TCP_pose()` - Returns the current TCP pose as a tuple `(xyz, ypr)`, in the same frame, units and angle order `MoveLinear` takes, so it can be passed back to it.

**Gripper and suction cup**

Both tools grasp and release objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**. Use the function of the tool of the selected world.

- `HAL.GripperSet(percentage_closure, wait_time)` - Closes (`100`) or opens (`0`) the parallel gripper to the given closing percentage, adding a final delay in seconds. When it starts closing (`> 5`) automatic attachment is enabled; when it opens (`<= 5`) any attached object is automatically detached.
- `HAL.SuctionSet(on, wait_time)` - Turns the suction cup on (`True`) or off (`False`), adding a final delay in seconds. When it is on, any object touching the cup is attached; when it is off, the held object is released.

The objects that can be automatically grasped are: `red_cube`, `green_cube` and `blue_cube`.

### HAL-based Implementation in C++

- `#include "HAL_dobot.hpp"` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo or the real robot).

**Direct Kinematics**

- `HAL::MoveAbsJ(joints, speed, wait_time);` - Moves the robot to the given angular position for each joint. `joints` is `std::array<double, 4>` in degrees, `speed` in [0,1], `wait_time` in seconds.
- `HAL::MoveSingleJ(joint_number, relative_angle, speed, wait_time);` - Moves a single joint by a relative angular increment. `joint_number` in [1,4], angle in degrees.

**Inverse Kinematics**

- `HAL::MoveJoint(xyz, ypr, speed, wait_time);` - Moves the TCP to an absolute Cartesian pose. `xyz` is `std::array<double, 3>` in metres, `ypr` in degrees.
- `HAL::MoveLinear(xyz, ypr, speed, wait_time);` - Moves the TCP in a linear trajectory to an absolute Cartesian pose. `xyz` in metres, `ypr` in degrees.
- `HAL::MoveRelLinear(xyz, speed, wait_time);` - Moves the TCP by a relative Cartesian increment. `xyz` is `std::array<double, 3>` in metres.
- `HAL::MoveRelReor(ypr, speed, wait_time);` - Reorients the TCP by relative angular increments. `ypr` is `std::array<double, 3>` in degrees.

**Robot state**

- `HAL::get_Joint_states();` - Returns the current joint angles in degrees as `std::array<double, 4>`, in the order `MoveAbsJ` takes them.
- `HAL::get_TCP_position();` - Returns the current TCP position in metres as `std::array<double, 3>`, in the frame `MoveLinear` takes.
- `HAL::get_TCP_orientation();` - Returns the current TCP orientation in degrees as `std::array<double, 3>`, in the angle order `MoveLinear` takes.

**Gripper and suction cup**

- `HAL::GripperSet(relative_closure, wait_time);` - Controls the parallel gripper. `relative_closure` in [0,100] (0 = fully open, 100 = fully closed). When closing (`> 5`) contact-based automatic attachment is enabled; when opening (`<= 5`) the attached object is automatically released.
- `HAL::SuctionSet(on, wait_time);` - Controls the suction cup. `on = true` grips any object touching the cup, `on = false` releases it.

In order to use the HAL-based controls you must include the following lines:

```cpp
#include "HAL_dobot.hpp"
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

### Argument examples

**Data targets for `MoveAbsJ` (joint angles, in degrees)**

```
absj_home = [0, 17, 17, 0]
```

`absj_home` is the resting pose of the arm.

### Objects and targets

**Dobot Pick And Place World.** The three cubes and the three mats lie on the table, at the height of the arm base.

- **red_cube** — Size (l,w,h) = (2.5,2.5,2.5) cm — Pose (x,y) = (0.2, -0.08) m
- **green_cube** — Size (l,w,h) = (2.5,2.5,2.5) cm — Pose (x,y) = (0.2, 0) m
- **blue_cube** — Size (l,w,h) = (2.5,2.5,2.5) cm — Pose (x,y) = (0.2, 0.08) m
- **red_mat** — Size (l,w) = (6,6) cm — Pose (x,y) = (0.115, -0.171) m
- **green_mat** — Size (l,w) = (6,6) cm — Pose (x,y) = (0.05, -0.2) m
- **blue_mat** — Size (l,w) = (6,6) cm — Pose (x,y) = (-0.022, -0.205) m

**Dobot Suction Cup Pick And Place World and Dobot Magician Physical.** The real robot uses this same layout, marked on the table with a measuring tape in the simulated world.

- **red_cube** — Size (l,w,h) = (4.5,4.5,4.5) cm — Pose (x,y) = (0.2, 0.1) m
- **green_cube** — Size (l,w,h) = (4.5,4.5,4.5) cm — Pose (x,y) = (0.2, 0) m
- **blue_cube** — Size (l,w,h) = (4.5,4.5,4.5) cm — Pose (x,y) = (0.2, -0.1) m
- **red_mat** — Size (l,w) = (7,7) cm — Pose (x,y) = (0.2, -0.18) m
- **green_mat** — Size (l,w) = (7,7) cm — Pose (x,y) = (0.12, -0.18) m
- **blue_mat** — Size (l,w) = (7,7) cm — Pose (x,y) = (0.04, -0.18) m

---

## Theory

### TCP orientation (Yaw, Pitch, Roll)

The `MoveJoint` and `MoveLinear` functions receive the TCP orientation as a `[Yaw, Pitch, Roll]` list, in degrees, expressed in the world frame. At `[0, 0, 0]` the tool frame is aligned with the world frame:

{% include gallery id="rpy000" caption="TCP frame at YPR = (0, 0, 0)." %}

A single 90º rotation on each axis (yaw, pitch or roll) reorients the tool as follows:

{% include gallery id="rpy" caption="Effect of a 90º rotation on each axis: Roll (left), Pitch (centre), Yaw (right)." %}

### Relationship among ROS 2, MoveIt 2, Gazebo and the JdeRobot API

- [ROS](https://www.ros.org/) (Robot Operating System) is a robotics middleware which contains a set of open source libraries for developing robot applications.
- [MoveIt](https://moveit.ros.org/) is an open source Motion Planning framework for industrial robots which integrates motion planning, collision checking, manipulation and 3D perception capabilities.
- [RViz](https://github.com/ros2/rviz) is a 3D visualization tool for ROS. Many ROS topics can be visualized in RViz, including the planning scene of the MoveIt move group, but it does not contain any physics simulation capability.
- [Gazebo](https://gazebosim.org/) is a physics simulator mainly used for robot simulation.
- The HAL API provided by JdeRobot Robotics Academy is built on top of the above tools and the IFRA Cranfield repositories (see References), so you do not need to learn all of them to start simulating industrial robot manipulation.
- With a real robot, Gazebo is replaced by the robot driver, which offers the same ROS 2 interfaces as the simulated arm, so MoveIt and the HAL run unchanged.

## Hints

Simple hints to help you solve the Pick and Place exercise.

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

## Videos

### UR5

{% include youtubePlayer.html id=page.youtubeId %}

### Dobot Magician

{% include youtubePlayer.html id=page.youtubeId2 %}

## Running on the real robots

The worlds **Ur5 Physical** and **Dobot Magician Physical** move a real arm instead of a simulated one. No simulator runs: the robot driver offers the same ROS 2 interfaces as the simulated arm, so MoveIt and the HAL run unchanged. Both drivers are part of the Robotics Backend and are started by the exercise when the world is launched.

The steps are the usual ones of the [Unibotics user guide](https://unibotics.org/user_guide): launch the Robotics Backend on your computer, open the exercise in [unibotics.org](https://unibotics.org) and connect. The difference is that the Robotics Backend is launched with extra options for the robot. The Robotics Backend runs inside a Docker container and the robot is outside it, so the container has to be given a way to talk to the hardware, and each robot does it in its own way:

- The **UR5 / UR5e** is reached over Ethernet and connects back to the computer, so the container needs the robot address and the ports the robot connects to.
- The **Dobot Magician** is reached over USB, so the container needs access to the USB devices of the computer.

Each robot section gives the full command to copy. It is the launch command without GPU acceleration of the user guide, since no simulator runs with a real robot, and it works on any computer.

### UR5 / UR5e

{% include figure image_path="/assets/images/exercises/pick_place/ur5e_real.jpg" alt="Real UR5e" caption="The real UR5e at its home pose." %}

Works with a UR5 (CB3) and a UR5e (e-Series). The arm runs on the official `ur_robot_driver`. The Robotiq 2F-85 gripper is driven through its URCap when the robot has it. The robot connects back to the computer on the ports 50001 to 50004, so the computer needs a fixed address on the robot network.

**1. Cable.** With the robot powered off, open the control box and plug an Ethernet cable from the computer into the port on the lower edge of the white panel, under the network icon.

{% include figure image_path="/assets/images/exercises/pick_place/ur5e_controller_ethernet.jpg" alt="Ethernet port inside the UR5e control box" caption="Ethernet port inside the control box." %}

**2. Robot address.** Power the robot on with the button of the pendant. Open the menu at the top right corner, then Settings > System > Network, and write down the robot address and the subnet mask, for example `192.168.0.4` and `255.255.255.0`. If the network method is DHCP, choose Static Address, type an address and press Apply.

{% include figure image_path="/assets/images/exercises/pick_place/ur5e_pendant_network.jpg" alt="Robot address on the pendant" caption="Robot address on the pendant (the pendant of the photo is in Spanish)." %}

**3. Computer address.** Give the computer a fixed address in the same network, with the same subnet mask and an empty gateway, for example `192.168.0.50`. Each computer connected to the robot needs a different address.

- **Linux (Ubuntu):** Settings > Network > Wired > gear icon > IPv4 tab > Manual. Type the address and the netmask, leave the gateway empty and press Apply. Turn the wired connection off and on.
- **Windows:** Settings > Network & internet > Ethernet > IP assignment > Edit > Manual. Turn IPv4 on, type the IP address and the subnet mask, leave the gateway empty and press Save.

{% include figure image_path="/assets/images/exercises/pick_place/ur5e_pc_network.png" alt="Fixed address of the computer on Ubuntu" caption="Fixed address of the computer on Ubuntu." %}

Check the connection with the robot address, it has to answer:

```bash
ping 192.168.0.4
```

**4. Pendant.** In this order:

1. In the Initialize screen press **ON**, wait, and press **START** with the same button to release the brakes. The robot is ready when "Robot operational" turns green.
2. Press **Exit** to leave the Initialize screen.
3. On a UR5e, only the first time, enable remote control in the menu at the top right corner > Settings > System > Remote Control.
4. On a UR5e, tap the **Local** icon at the top right corner and choose **Remote Control**. The icon only responds outside the Initialize and Settings screens.

{% include figure image_path="/assets/images/exercises/pick_place/ur5e_pendant_init.jpg" alt="Initialize screen of the pendant" caption="Initialize screen of the pendant and the Local icon (the pendant of the photo is in Spanish)." %}

**5. Launch the Robotics Backend.** Replace the three addresses and the robot type with yours:

- `UR_TYPE`: `ur5` for a CB3, `ur5e` for an e-Series.
- `UR_ROBOT_IP`: robot address, from step 2.
- `UR_REVERSE_IP`: computer address, from step 3.
- `UR_GRIPPER`: `none` without gripper, `auto` with a Robotiq gripper.

```bash
docker run --rm -it --name robotics-backend -p 6080-6090:6080-6090 -p 1108:1108 -p 7163:7163 -p 50001-50004:50001-50004 -e UR_TYPE=ur5e -e UR_ROBOT_IP=192.168.0.4 -e UR_REVERSE_IP=192.168.0.50 -e UR_GRIPPER=none jderobot/robotics-backend:latest
```

The full list of options is in the [ur5_bridge README](https://github.com/JdeRobot/RoboticsInfrastructure/blob/humble-devel/Industrial/ur5_bridge/README.md).

**6. Open Pick and Place** in [unibotics.org](https://unibotics.org), connect to the Robotics Backend and choose the **Ur5 Physical** world.

**7. Speed.** In Remote Control the pendant locks its speed slider. Set it from a second terminal, start low. `0.2` is 20 %:

```bash
docker exec -it robotics-backend bash -c "source /opt/ros/humble/setup.bash && ros2 service call /io_and_status_controller/set_speed_slider ur_msgs/srv/SetSpeedSliderFraction '{speed_slider_fraction: 0.2}'"
```

**8. Run your code** with `import HAL`, as in simulation. The coordinates are in the robot base frame: `z = 0` is the surface the robot is mounted on.

**Safety**

- MoveIt does not know about the table or anything around the robot, there is no collision model.
- The coordinates of the simulation belong to another table, do not run a simulation program on the real robot.
- Prefer absolute targets above a minimum height over chains of relative moves, a failed step in a chain can end up against the table.
- Start with a low speed and keep a hand near the emergency stop.

**Possible errors**

| Symptom | Fix |
| ------- | --- |
| `ping` gets no answer | Check that the cable is in the port of step 1 and that the computer address and subnet mask of step 3 are in the network of the robot |
| The arm does not move and the pendant shows error `C210A0` | The robot is in Local mode. Switch it to Remote Control and launch the world again |
| The Remote Control icon does not respond | Leave the Initialize or Settings screen first |
| The pendant speed slider cannot be changed | Expected in Remote Control, set the speed with the command of step 7 |
| The arm moves too slowly to notice | Raise the `speed` of the HAL call and the speed slider |
| `ModuleNotFoundError: No module named 'HAL'` with an empty console | The file is inside a folder of the editor, create it at the top level |
| The arm does not move after stopping and running the code again | Launch the world again |

### Dobot Magician

{% include figure image_path="/assets/images/exercises/pick_place/dobot_real.jpg" alt="Real Dobot Magician" caption="The real Dobot Magician with the suction cup, the cubes and the mats." %}

The real Dobot runs with the suction cup, the parallel gripper is only available in simulation. Its driver talks to the robot firmware over the USB serial port, so the container needs access to the USB devices of the computer. It only works on Linux, Docker Desktop on Windows and macOS does not pass USB devices through.

**1. Suction cup.** Mount the suction cup on the arm and connect its air tube to the pump box. Connect the two cables of the pump box to the upper connectors of the **Peripheral Interface** of the base: the red power cable to **SW1** and the green signal cable to **GP1**. In each colour the upper connector is number 1 and the lower one number 2. With the cables in other connectors the arm moves but the pump never turns on.

{% include figure image_path="/assets/images/exercises/pick_place/dobot_pump_ports.jpg" alt="SW1 and GP1 connectors of the Dobot base" caption="In this photo the cables are in GP2 and SW2 (red), which is wrong. They go in GP1 and SW1 (blue), the upper green and red connectors of the Peripheral Interface." %}

**2. Table.** Lay the cubes and the mats as in the [Dobot Suction Cup Pick And Place World](#objects-and-targets-1), measuring from the centre of the arm base.

**3. Power and USB.** Power the Dobot on and plug its USB cable into the computer. Check that the computer sees it:

```bash
ls /dev/ttyUSB*
```

**4. Launch the Robotics Backend.** The options `-v /dev:/host_dev:ro` and `--device-cgroup-rule` give the container access to the USB devices. The cable can also be plugged in after launching it:

```bash
docker run --rm -it --name robotics-backend -p 6080-6090:6080-6090 -p 1108:1108 -p 7163:7163 -v /dev:/host_dev:ro --device-cgroup-rule='c 188:* rmw' --device-cgroup-rule='c 166:* rmw' jderobot/robotics-backend:latest
```

The devices of the computer go to `/host_dev` and never to `/dev`, so the terminals of the computer do not show up inside the container.

**5. Open Pick and Place** in [unibotics.org](https://unibotics.org), connect to the Robotics Backend and choose the **Dobot Magician Physical** world.

After each power on the arm homes once. It first raises the rear arm and the forearm, so the cup does not drag over the table, and then turns the base. Wait until it stops before running your code.

**6. Run your code** with `import HAL_dobot as HAL` and `HAL.SuctionSet`, as in the Dobot Suction Cup Pick And Place World. The same program moves the simulated and the real robot.

**Possible errors**

| Symptom | Fix |
| ------- | --- |
| The arm moves but the pump does not turn on | The pump cables are not in SW1 and GP1, see step 1 |
| The pump runs but the cube falls | Check that the air tube is connected to the suction cup |
| The Dobot is not found | Check the USB cable and `ls /dev/ttyUSB*` on the computer, and the USB options of the launch command |

## Contributors

- Contributors: [Diego Martín](https://github.com/diegomrt), [José María Cañas](https://github.com/jmplaza) and [Javier Izquierdo](https://github.com/javizqh).

## References

1. IFRA-Cranfield (2023). ROS 2 Sim-to-Real Robot Control. [https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
2. [https://moveit.ros.org/](https://moveit.ros.org/)
3. [https://gazebosim.org/](https://gazebosim.org/)
4. [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
5. [Dobot Magician](https://www.dobot-robots.com/products/education/magician.html)
