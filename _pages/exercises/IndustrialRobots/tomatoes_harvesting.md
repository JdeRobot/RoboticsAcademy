---
permalink: /exercises/IndustrialRobots/tomatoes_harvesting
title: "Tomatoes Harvesting"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Tomatoes Harvesting"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/tomatoes_harvesting/harvest_approach.jpg
    image_path: /assets/images/exercises/tomatoes_harvesting/harvest_approach.jpg
    alt: "Harvest head approaching a truss"
    title: "Harvest head approaching a truss"
  - url: /assets/images/exercises/tomatoes_harvesting/harvest_grip.jpg
    image_path: /assets/images/exercises/tomatoes_harvesting/harvest_grip.jpg
    alt: "Tomato held between the fingers"
    title: "Tomato held between the fingers"
  - url: /assets/images/exercises/tomatoes_harvesting/harvest_crate.jpg
    image_path: /assets/images/exercises/tomatoes_harvesting/harvest_crate.jpg
    alt: "Tomato released over the crate"
    title: "Tomato released over the crate"

rules:
  - url: /assets/images/exercises/tomatoes_harvesting/tomato_states.png
    image_path: /assets/images/exercises/tomatoes_harvesting/tomato_states.png
    alt: "Life of a tomato"
    title: "Life of a tomato"
  - url: /assets/images/exercises/tomatoes_harvesting/force_window.png
    image_path: /assets/images/exercises/tomatoes_harvesting/force_window.png
    alt: "Grip force window"
    title: "Grip force window"
---

## Goal

The goal of this exercise is to harvest ripe tomatoes in a greenhouse with a mobile manipulator. The robot has to drive along the aisles, find the ripe tomatoes with the camera of its harvest head, pick them one by one without crushing them or tearing their truss, and leave them in the crate it carries. Green tomatoes have to stay on the plant.

{% include figure image_path="/assets/images/exercises/tomatoes_harvesting/greenhouse.jpg" alt="Tomato greenhouse" caption="The MMO-500 Harvester entering the default greenhouse, with its two numbered double rows." %}

The robot is the [Neobotix MMO-500](https://www.neobotix-robots.com/), an omnidirectional MPO-500 base with a UR10 arm. Instead of a gripper it carries a harvest head with three fingers that close around a tomato and spin to snap its stem, and an RGB-D camera that looks along the fingers. The exercise runs on **Gazebo Harmonic** and offers three worlds:

- [**Tomato Greenhouse**](#tomato-greenhouse) (default): two double rows of 6 m with mostly ripe fruit.
- [**Tomato Greenhouse Big**](#tomato-greenhouse-big): four double rows of 10 m with more fruit and more green tomatoes.
- [**Tomato Harvest Bench**](#tomato-harvest-bench): a single plant line with three fixed trusses next to the robot, to practise one harvest at a time.

{% include gallery caption="Approaching a truss (left), holding a tomato between the fingers (centre) and releasing it over the crate (right)." %}

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

This exercise supports the HAL-based approach and the ROS 2-direct implementation, both in Python.

### Frames

- All the arm goals are expressed in the `base_footprint` frame of the robot: origin on the floor under the base centre, `x` forward, `y` to the left and `z` up.
- The arm moves the **grasp point**, the centre of the space between the three fingers. The harvest head is always kept level, so its orientation is a single angle, the **heading**: the direction the fingers point to, measured around the vertical like the base yaw. `0` points forward, `π/2` to the left and `-π/2` (the default) to the right.
- World coordinates are in metres in the frame of `HAL.getPose3d()`, which gives the exact pose of the robot (without odometry noise). A world point `(wx, wy)` seen from a base at `(x, y, yaw)` is at `bx = cos(yaw)·(wx - x) + sin(yaw)·(wy - y)` and `by = -sin(yaw)·(wx - x) + cos(yaw)·(wy - y)` in `base_footprint`.

### HAL-based Implementation in Python

- `import HAL` - to import the HAL (Hardware Abstraction Layer) library class. This class contains the functions that send and receive information to and from the Hardware (Gazebo).
- `import WebGUI` - to import the WebGUI (Web Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

**Base**

- `HAL.getPose3d()` - Returns a `Pose3d` object with the base pose in the world frame: `x`, `y`, `z` (metres), `yaw`, `pitch`, `roll` (radians), `q` (quaternion) and `timeStamp` (seconds).
- `HAL.setVX(velocity)` - to set the forward speed of the base in m/s, limited to ±0.5.
- `HAL.setVY(velocity)` - to set the sideways speed of the base in m/s (positive to the left), limited to ±0.5.
- `HAL.setW(velocity)` - to set the yaw rate of the base in rad/s (positive counterclockwise), limited to ±0.8.

**Lasers**

- `HAL.getFrontLaserData()` / `HAL.getBackLaserData()` - Returns a `LaserData` object of the front / back laser, with `values` (720 distances in metres over ±1.48 rad), `minAngle`, `maxAngle`, `minRange`, `maxRange` and `timeStamp`. The back laser looks backwards.

**Arm**

- `HAL.moveArm(x, y, z, heading=-math.pi/2)` - Moves the grasp point to `(x, y, z)` in `base_footprint` with the fingers pointing towards `heading`, through a trajectory planned by MoveIt 2. If the plan fails it unfolds the arm in front of the base and tries once more. Returns `True` once the arm is there and `False` if the point cannot be reached.
- `HAL.getArmPose()` - Returns an `ArmPose` object with the current grasp point `x`, `y`, `z` in `base_footprint` and its `heading`, or `None` until the arm reports its first pose.
- `HAL.homeArm()` - Folds the arm over the base, the pose the robot starts in.

**Harvest head**

- `HAL.grip(force)` - Closes the fingers until they squeeze with `force` newtons and returns the force reached. It keeps closing from where the fingers are, so a second call with a higher force tightens the grip without letting go first. Returns `0.0` when the fingers closed completely without touching anything.
- `HAL.twist(turns)` - Spins the fingers this many turns, two turns per second. Negative turns spin the other way. The head can wind up to five turns each way from its rest position. Returns `True` once the turns are done.
- `HAL.release()` - Opens the fingers. The head winds back to its rest position at the same time.

**Wrist camera**

- `HAL.getImage()` - Returns the 640 x 480 BGR image of the wrist camera.
- `HAL.getDepthImage()` - Returns the 640 x 480 depth image of the wrist camera as a float array in metres, `inf` where nothing is in range (0.1 to 6 m). The depth is only rendered while it is being asked for, so the first call after a few seconds without using it waits for a new frame.
- `HAL.pixelToPoint(u, v)` - Returns the point seen at pixel column `u` and row `v` as `[x, y, z]` in `base_footprint`, using the depth around that pixel and the current pose of the head. Returns `None` when that pixel has no depth.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showLeftImage(image)` - Shows a BGR image in the left panel.

The WebGUI also shows the [score](#score) of the run in its top right corner.

### ROS 2-direct Implementation

Use standard ROS 2 topics and actions for direct communication with the simulation. The robot runs in the `/mmo500` namespace.

- `/clock` - Subscribe to this topic to receive the simulation time. Message type: `rosgraph_msgs/msg/Clock`

- `/mmo500/odom` - Subscribe to this topic to receive the base pose in the world frame. Message type: `nav_msgs/msg/Odometry`

- `/mmo500/cmd_vel` - Publish to this topic to set the base velocities (`linear.x`, `linear.y` and `angular.z`). Message type: `geometry_msgs/msg/Twist`

- `/mmo500/front_laser/scan` and `/mmo500/back_laser/scan` - Subscribe to these topics to receive the front / back laser scans. Message type: `sensor_msgs/msg/LaserScan`

- `/mmo500/joint_states` - Subscribe to this topic to read the position of every joint. Message type: `sensor_msgs/msg/JointState`

- `/mmo500/harvest_camera/image`, `/mmo500/harvest_camera/depth_image` and `/mmo500/harvest_camera/camera_info` - Subscribe to these topics to receive the colour image, the depth image in metres (`32FC1`) and the intrinsics of the wrist camera. They are only rendered while someone subscribes to them. Message types: `sensor_msgs/msg/Image` and `sensor_msgs/msg/CameraInfo`

- `/mmo500/Robmove` - Action to move the grasp point to a pose planned by MoveIt 2 (`type` `"PTP"` or `"LIN"`, `speed` between 0 and 1, position `x`, `y`, `z` in `base_footprint` and quaternion `qx`, `qy`, `qz`, `qw` of a frame with `z` along the fingers and `x` up). Action type: `ros2srrc_data/action/Robmove`

- `/mmo500/Robpose` - Subscribe to this topic to receive the current pose of the grasp point. Message type: `ros2srrc_data/msg/Robpose`

- `/mmo500/arm_controller/follow_joint_trajectory` - Action to move the six UR10 joints directly. Action type: `control_msgs/action/FollowJointTrajectory`

- `/mmo500/gripper_controller/joint_trajectory` and `/mmo500/gripper_controller/follow_joint_trajectory` - Topic and action to move `harvest_finger_1_joint`, from `0.0` (open) to `0.20` rad (closed). Message type: `trajectory_msgs/msg/JointTrajectory`. Action type: `control_msgs/action/FollowJointTrajectory`

- `/mmo500/twist_controller/follow_joint_trajectory` - Action to spin `harvest_twist_joint`, within ±10π rad. Action type: `control_msgs/action/FollowJointTrajectory`

- `/tomato_greenhouse/grip_force` - Subscribe to this topic to read the force in newtons the fingers apply on a tomato. Message type: `std_msgs/msg/Float32`

- `/tomato_greenhouse/score` - Subscribe to this topic (QoS **TransientLocal**) to receive the [score](#score) as a JSON string. Message type: `std_msgs/msg/String`

For image debugging:

- `/webgui/image_debug_left` and `/webgui/image_debug_right` - Publish to these topics to set the left / right debug image in the GUI interface. Message type: `sensor_msgs/msg/Image`

**Note**: Ensure this import is included in your script to access the Web GUI functionalities.

`import WebGUI` - to enable the Web GUI for visualizing camera images.

To have frequency control you need to use standard ROS 2 mechanisms to manage loop timing:

- `rclpy.spin()` - Event-driven execution using callbacks.
- `rclpy.spin_once()` - Single-step processing, often with custom timers.
- `rclpy.Rate()` - Loop-based frequency control.

**Note**
`WebGUI` already initializes `rclpy` internally, so this should be taken into account when building a direct ROS 2 solution.

### Argument examples

**Grasp point in front of the base and to its right, fingers pointing right**

```python
HAL.moveArm(0.4, -0.6, 1.2)
```

**Same point with the fingers pointing forward**

```python
HAL.moveArm(0.4, -0.6, 1.2, heading=0.0)
```

**A pixel of the wrist camera to a point of the base frame**

```python
point = HAL.pixelToPoint(320, 240)
if point is not None:
    x, y, z = point
```

---

## The MMO-500 Harvester
{: style="font-size: 1.9em; margin-top: 1.5em"}

### The robot

{% include figure image_path="/assets/images/exercises/tomatoes_harvesting/robot_top.png" alt="MMO-500 Harvester top view" caption="Top view of the MMO-500 Harvester with its `base_footprint` frame, the crate, the UR10 base, the field of both lasers and the heading of the harvest head." %}

- The base is 0.80 metres long and 0.57 metres wide and moves in any direction: forward, sideways and turning, at the same time.
- The UR10 arm base is at (0.205, 0, 0.802) in `base_footprint`. The arm reaches about 1.3 m around it.
- The crate is mounted over the front of the base, centred at (0.56, 0). Its inside is 0.37 x 0.57 m and 0.14 m deep, with the floor at z = 0.415 m. Everything that falls inside it travels with the robot.
- The front laser is at (0.442, 0, 0.372) and the back laser at (-0.442, 0, 0.372), looking backwards.

### The harvest head

{% include figure image_path="/assets/images/exercises/tomatoes_harvesting/harvest_head.png" alt="Harvest head" caption="The harvest head seen from the side, with the grasp point and the wrist camera." %}

- Three fingers, 120° apart, close around the grasp point. A tomato is between the fingers when its centre is less than 2.5 cm from the finger axis and less than 3 cm from the grasp point along it.
- The fingers spin together around their axis to twist the tomato off its stem.
- The RGB-D camera sits 0.185 m behind the grasp point and 0.045 m above the finger axis, looking along the fingers with its optical axis tilted 0.1 rad towards them. A tomato at the grasp point therefore appears somewhat below the centre of the image. The field of view is 1.5 rad horizontally, the image 640 x 480 at 15 Hz.

### Harvesting a tomato

{% include gallery id="rules" caption="What can happen to a tomato (left) and what a grip does to it depending on its ripeness (right)." %}

- **Grip.** The fingers only grip a tomato that was already between them while they were open. Closing on a tomato that is not centred, or closing first and then approaching, only bumps into it.
- **Force.** A grip below 1 N is too loose and the fingers slip around the fruit. Every tomato bursts above a force that goes from 2.5 N for a ripe one to 5 N for a green one.
- **Twist.** With a firm grip, twisting turns the tomato on its stem. Each stem snaps after between one and two turns, and from then on the tomato is in the hand.
- **Pull.** Moving the head more than 4 cm while holding a tomato that is still on its stem tears the whole truss, and all its fruit falls.
- **Release.** Opening the fingers before the stem snaps leaves the tomato on the plant. Opening them after drops the tomato: it is harvested if it falls in the crate and lost if it falls anywhere else.
- **Ripeness.** A tomato is green below half its ripeness, from green through orange to deep red. Green tomatoes in the crate are counted as a mistake.

### Score

The WebGUI shows the score of the run in its top right corner, and it is also published in `/tomato_greenhouse/score`.

- **Ripe harvested**: ripe tomatoes in the crate over the ripe tomatoes the plants had at the start, and its percentage.
- **Precision**: ripe tomatoes in the crate over every tomato taken off the plant, well or badly.
- **Green**, **Crushed**, **Dropped** and **Broken trusses**: the mistakes of the run.

The score starts again when the world is reset.

---

## Tomato Greenhouse
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/tomatoes_harvesting/greenhouse_map.png" alt="Tomato Greenhouse map" caption="The default greenhouse seen from above, with its two double rows and its three aisles." %}

- The greenhouse is 12 metres long and 7.2 metres wide, closed by glass walls at x = ±6 and y = ±3.6, with a door on the east wall.
- There are two numbered double rows of plants along the x axis, from x = -3 to x = 3. The plant lines are at y = ±0.9 and y = ±1.5, on gutters 0.26 m wide and 0.44 m tall.
- The aisles run at y = 0, with plants on both sides, and at y = ±2.4, with plants on one side. The aisle at y = 0 is about 1.5 m wide between gutters.
- The robot starts at (4.6, 0) facing -x, in the free ground east of the rows.

### Tomatoes

- About 78 tomatoes hang on 24 trusses spread over all the plant lines. Most of them are ripe and only a few are green.
- Each truss carries three or four tomatoes in a line parallel to the row, 10 cm apart, and hangs towards the aisle, 3 to 17 cm away from its plant line.
- The tomatoes hang between 0.75 and 1.57 m above the floor, and their radius is between 2 and 4 cm.
- Their position and ripeness change every time the world starts or is reset.

---

## Tomato Greenhouse Big
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/tomatoes_harvesting/greenhouse_big_map.png" alt="Tomato Greenhouse Big map" caption="The big greenhouse seen from above, with its four double rows and its five aisles." %}

- The greenhouse is 17 metres long and 12 metres wide, closed by glass walls at x = ±8.5 and y = ±6, with a door on the east wall.
- There are four numbered double rows of plants along the x axis, from x = -5 to x = 5. The plant lines are at y = ±0.9, ±1.5, ±3.3 and ±3.9.
- The aisles run at y = 0 and y = ±2.4, with plants on both sides, and at y = ±4.8, with plants on one side.
- The robot starts at (7.0, 0) facing -x.
- About 195 tomatoes hang on 60 trusses, and around one in six of them is green. They follow the same rules as in the default greenhouse.

---

## Tomato Harvest Bench
{: style="font-size: 1.9em; margin-top: 1.5em"}

{% include figure image_path="/assets/images/exercises/tomatoes_harvesting/bench_map.png" alt="Tomato Harvest Bench map" caption="The bench seen from above, with the robot, its crate and the three trusses on its right." %}

- A single plant line runs along the x axis at y = -0.9, to the right of the robot, which starts at (0, 0) facing +x.
- Three trusses of three tomatoes start at x = 0.15, 0.65 and 1.15. Their tomatoes hang about 1 m above the floor, 10 cm apart, at y ≈ -0.8.
- The trusses never change, and the last tomato of the third one is green.

---

## Theory

### Robotic harvesting

Harvesting fruit is one of the hardest tasks for a robot. The fruit hides among the leaves, every plant is different, and the fruit is delicate: a robot has to find it, reach it without hitting the plant and detach it without damaging it. Commercial harvesters solve the detachment with soft grippers that hold the fruit and twist it, because a fruit stem breaks easily by rotation at its joint, while pulling transmits the force to the whole truss.

### Ripeness from colour

Tomatoes change from green to red as they ripen, so colour is the usual cue to tell ripe fruit. Colour segmentation works better in the HSV colour space than in BGR, because the hue separates the colour from the lighting. In OpenCV the hue goes from 0 to 180, and red sits at both ends of that range.

### From a pixel to a point

A camera projects a 3D point onto a pixel and loses its distance. A depth camera gives that distance back for every pixel, so a pixel `(u, v)` with depth `d` can be turned back into a 3D point with the pinhole model: `X = (u - cx)·d / fx`, `Y = (v - cy)·d / fy`, `Z = d`, where `fx`, `fy`, `cx` and `cy` are the intrinsics of the camera. That point is in the frame of the camera, and the pose of the camera turns it into the base frame. `HAL.pixelToPoint` does both steps.

### Relationship among ROS 2, MoveIt 2, Gazebo and the JdeRobot API

- [ROS](https://www.ros.org/) (Robot Operating System) is a robotics middleware which contains a set of open source libraries for developing robot applications.
- [MoveIt](https://moveit.ros.org/) is an open source Motion Planning framework for industrial robots which integrates motion planning, collision checking, manipulation and 3D perception capabilities.
- [Gazebo](https://gazebosim.org/) is a physics simulator mainly used for robot simulation.
- The HAL API provided by JdeRobot Robotics Academy is built on top of the above tools and the IFRA Cranfield repositories (see References), so you do not need to learn all of them to start harvesting.

## Hints

Simple hints to help you solve the Tomatoes Harvesting exercise.

### Where to insert and run the code

In the launched web page, type your code in the text editor and run it by pressing the play button.

```python
import HAL
import WebGUI
# Enter sequential code here!

while True:
    # Enter iterative code here!
```

### Why does the arm fail to move to a desired pose?

The most likely reason is that the point is out of the reach of the arm with the requested heading, so MoveIt cannot plan a trajectory to it. `HAL.moveArm` returns `False` and you will see this message:

```bash
Arm movement FAILED, the point may be out of reach
```

### Why does `HAL.grip` return 0.0?

The fingers closed completely without touching a tomato. The tomato must be between the open fingers before closing them: its centre less than 2.5 cm from the finger axis and less than 3 cm from the grasp point.

### Why did the tomato burst?

The grip went over the force that tomato can stand, which is lower the riper it is. See [Harvesting a tomato](#harvesting-a-tomato).

### Why did the whole truss fall?

The head moved more than 4 cm while it held a tomato that was still attached to its stem. A tomato has to come off its stem before the arm carries it away.

### Why does the first depth image take a while?

The depth image is only rendered while it is being used. After a few seconds without asking for it, the next call to `HAL.getDepthImage` or `HAL.pixelToPoint` waits until the camera sends a new frame.

## References

1. Neobotix ROS 2 simulation. [https://github.com/neobotix/neo_simulation2](https://github.com/neobotix/neo_simulation2)
2. IFRA-Cranfield (2023). ROS 2 Sim-to-Real Robot Control. [https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
3. [https://moveit.ros.org/](https://moveit.ros.org/)
4. [https://gazebosim.org/](https://gazebosim.org/)
5. [https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html](https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html)
