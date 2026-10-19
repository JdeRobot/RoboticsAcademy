---
permalink: /exercises/IndustrialRobots/mobile_manipulation
title: "Mobile Manipulation"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Mobile Manipulation"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg
    alt: "XLeRobot Warehouse"
    title: "XLeRobot Warehouse"
  - url: /assets/images/exercises/mobile_manipulation/mmo500_robot.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_robot.jpg
    alt: "MMO-500"
    title: "MMO-500"
  - url: /assets/images/exercises/mobile_manipulation/myagv_warehouse.jpg
    image_path: /assets/images/exercises/mobile_manipulation/myagv_warehouse.jpg
    alt: "myAGV mechArm Warehouse Delivery"
    title: "myAGV mechArm Warehouse Delivery"

xlerobot:
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_warehouse.jpg
    alt: "XLeRobot Warehouse"
    title: "XLeRobot Warehouse"
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_grasp.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_grasp.jpg
    alt: "XLeRobot grasping the yellow box"
    title: "XLeRobot grasping the yellow box"
  - url: /assets/images/exercises/mobile_manipulation/xlerobot_place.jpg
    image_path: /assets/images/exercises/mobile_manipulation/xlerobot_place.jpg
    alt: "XLeRobot placing the green cylinder"
    title: "XLeRobot placing the green cylinder"

mmo500:
  - url: /assets/images/exercises/mobile_manipulation/mmo500_warehouse.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_warehouse.jpg
    alt: "MMO-500 Warehouse"
    title: "MMO-500 Warehouse"
  - url: /assets/images/exercises/mobile_manipulation/mmo500_grasp.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_grasp.jpg
    alt: "MMO-500 grasping the green cylinder"
    title: "MMO-500 grasping the green cylinder"
  - url: /assets/images/exercises/mobile_manipulation/mmo500_place.jpg
    image_path: /assets/images/exercises/mobile_manipulation/mmo500_place.jpg
    alt: "MMO-500 placing the blue box"
    title: "MMO-500 placing the blue box"

myagv:
  - url: /assets/images/exercises/mobile_manipulation/myagv_robot.jpg
    image_path: /assets/images/exercises/mobile_manipulation/myagv_robot.jpg
    alt: "myAGV with mechArm 270"
    title: "myAGV with mechArm 270"
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

The exercise offers three worlds, each one with its own robot and HAL module. The exercise runs on **Gazebo Harmonic**.

- **XLeRobot Warehouse**: the [XLeRobot](https://github.com/Vector-Wangel/XLeRobot), a holonomic base with two 5-joint arms, a robot tray and an RGB-D head camera, has to take three objects from a conveyor to the tray of their colour.
- **MMO-500 Warehouse**: the [Neobotix MMO-500](https://www.neobotix-robots.com/), an omnidirectional MPO-500 base with a UR10 arm and a Robotiq 2F-85 gripper, has to take three objects from a conveyor to the tray of their colour.
- **myAGV mechArm Warehouse Delivery**: the [Elephant Robotics](https://www.elephantrobotics.com/) myAGV, a mecanum base with a mechArm 270 arm, a lidar and a front camera, has to deliver the coloured balls of two dispensers to the delivery box that lights up, while two warehouse workers walk around carrying boxes.

## Frequency API

### Python

- `import Frequency` - to import the Frequency library class. This class contains the tick function to regulate the execution rate.
- `Frequency.tick(ideal_rate)` - regulates the execution rate to the number of Hz specified. Defaults to 50 Hz.

## Robot API

The robot is controlled through the HAL-based approach. Each world has its own robot and its own HAL module. Below you'll find the details for each of them.

### HAL-based Implementation

All the arm goals are expressed in the `base_footprint` frame of the robot: origin on the floor under the base centre, `x` forward, `y` to the left and `z` up.

#### XLeRobot

{% include gallery id="xlerobot" caption="XLeRobot Warehouse." %}

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

- `HAL.setHeadJoints(pan, tilt, duration=1.0)` - Moves the head joints to the given angles in radians in `duration` seconds. `pan` in [-3.2, 3.2], `tilt` in [-0.76, 1.45] (positive values look down).

**Grasping**

- `HAL.graspLeft(x, y, z, speed=0.3)` / `HAL.graspRight(x, y, z, speed=0.3)` - Grasps with the left / right arm the object whose centre is at `(x, y, z)` in `base_footprint`: opens the jaw, approaches the object from the front, closes the jaw and retreats. Returns `True` on success.
- `HAL.placeLeft(x, y, z, speed=0.3)` / `HAL.placeRight(x, y, z, speed=0.3)` - Releases the object held by the left / right arm with its centre at `(x, y, z)` in `base_footprint`. Returns `True` on success.

**Robot tray**

- `HAL.dropInLeftTray(speed=0.3)` / `HAL.dropInRightTray(speed=0.3)` - Leaves the object held by the left / right arm in that arm's spot of the robot tray. The object stays fixed to the robot while it drives.
- `HAL.pickFromLeftTray(speed=0.3)` / `HAL.pickFromRightTray(speed=0.3)` - Takes again the object left in the left / right spot of the robot tray.

**Inverse Kinematics**

- `HAL.moveLeftArm(x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP")` / `HAL.moveRightArm(...)` - Moves the left / right gripper to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. `motion` is `"PTP"` for a point-to-point move or `"LIN"` for a linear trajectory. Returns `True` on success. The arms have 5 joints, so only exactly reachable poses are accepted.
- `HAL.getLeftArmPose()` / `HAL.getRightArmPose()` - Returns the current gripper pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) in `base_footprint`, or `None` until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking.

- `HAL.setLeftArmJoints(positions, duration=2.0)` / `HAL.setRightArmJoints(positions, duration=2.0)` - Moves the arm to the 5 joint angles in radians, in the order of `HAL.LEFT_ARM_JOINTS` / `HAL.RIGHT_ARM_JOINTS`.
- `HAL.readyPose(duration=2.0)` - Both arms forward with the grippers pointing down.
- `HAL.homeLeftArm(duration=2.0)` / `HAL.homeRightArm(duration=2.0)` - Arm to its fully extended zero pose.
- `HAL.leftTrayPose(duration=2.0)` / `HAL.rightTrayPose(duration=2.0)` - Gripper over its spot of the robot tray.

**Gripper**

The grippers grasp and release objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.setLeftGripper(closed, duration=1.0, target=None)` / `HAL.setRightGripper(closed, duration=1.0, target=None)` - `closed = True` closes the jaw and enables automatic attachment, `closed = False` opens it and releases the object. `target` overrides the jaw angle in radians, between `-0.3745` (closed) and `1.7453` (open).

The objects that can be automatically grasped are: `blue_box`, `green_cylinder` and `yellow_box`.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showDepthImage(depth)` - Shows a depth array in metres in the left panel as grayscale (closer is brighter, clipped at 4 m).

#### MMO-500

{% include gallery id="mmo500" caption="MMO-500 Warehouse." %}

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

- `HAL.moveArm(x, y, z, roll=math.pi, pitch=0.0, yaw=0.0, speed=0.3, motion="PTP")` - Moves the gripper centre (between the finger pads) to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. The default orientation points the gripper down and `yaw` turns the fingers. `motion` is `"PTP"` for a point-to-point move or `"LIN"` for a linear trajectory. Returns `True` on success.
- `HAL.getArmPose()` - Returns the current gripper pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) in `base_footprint`, or `None` until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking.

- `HAL.setArmJoints(positions, duration=3.0)` - Moves the UR10 to the 6 joint angles in radians, in the order of `HAL.ARM_JOINTS`.
- `HAL.homeArm(duration=3.0)` - Folds the arm over the base.
- `HAL.readyArm(duration=3.0)` - Unfolds the arm in front of the base with the gripper down.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.setGripper(closed, duration=1.0, target=None)` - `closed = True` closes the gripper and enables automatic attachment, `closed = False` opens it and releases the object. `target` overrides the knuckle angle in radians, from `0.0` (open) to `0.8` (closed).

The objects that can be automatically grasped are: `blue_box`, `yellow_ball` and `green_cylinder`.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showDepthImage(depth)` - Shows a depth array in metres in the left panel as grayscale (closer is brighter, clipped at 4 m).

#### myAGV mechArm 270

{% include gallery id="myagv" caption="myAGV mechArm Warehouse Delivery." %}

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

- `HAL.moveArm(x, y, z, roll=HAL.GRASP_ROLL, pitch=0.0, yaw=HAL.GRASP_YAW, speed=0.3, motion="PTP")` - Moves the gripper centre to a pose in `base_footprint` (metres and radians) planned by MoveIt 2. The default orientation looks forward, 70º below the horizon. `motion` is `"PTP"` for a point-to-point move or `"LIN"` for a linear trajectory. Returns `True` on success.
- `HAL.getArmPose()` - Returns the current gripper pose (`x`, `y`, `z`, `roll`, `pitch`, `yaw`) in `base_footprint`, or `None` until the first message arrives.

**Direct Kinematics**

These functions move the joints without planning or collision checking.

- `HAL.setArmJoints(positions, duration=3.0)` - Moves the mechArm 270 to the 6 joint angles in radians, in the order of `HAL.ARM_JOINTS`.
- `HAL.homeArm(duration=3.0)` - Folds the arm over the base.
- `HAL.readyArm(duration=3.0)` - Holds the gripper ahead of the base pitched down.

**Gripper**

The gripper grasps and releases objects automatically through a contact-based attachment system, so **no manual attach/detach calls are required**.

- `HAL.setGripper(closed, duration=1.0, target=None)` - `closed = True` closes the gripper and enables automatic attachment, `closed = False` opens it and releases the ball. `target` overrides the gripper angle in radians, between `-0.4` (closed) and `0.15` (open).

The objects that can be automatically grasped are the red and blue balls.

**WebGUI**

- `WebGUI.showImage(image)` - Shows a BGR image in the right panel.
- `WebGUI.showDepthImage(depth)` - Shows a depth array in metres in the left panel as grayscale (closer is brighter, clipped at 4 m).

### Argument examples

**Joint targets for the XLeRobot (radians, in arm joint order)**

```
ready_left  = [-1.600, 1.527, 2.123, 0.063, -1.571]
ready_right = [1.630, 1.525, 2.110, 0.074, -1.571]
```

**Joint targets for the MMO-500 (radians, in `HAL.ARM_JOINTS` order)**

```
home  = [1.5708, -2.75, 2.67, -1.5708, -1.5708, 0.0]
ready = [1.1428, -2.1233, 1.9619, -1.4094, -1.5708, -1.9988]
```

**Joint targets for the myAGV mechArm 270 (radians, in `HAL.ARM_JOINTS` order)**

```
home  = [0.0, -0.1, 0.2, 0.0, 1.5, 0.0]
ready = [0.0, 0.236, 0.255, 0.0, 0.739, 0.0]
```

**World point to `base_footprint` for `moveArm`**

```
bx, by, bz = HAL.worldToBase(4.8, 0.0, 1.035)
```

## Supporting information

All the coordinates are in metres, in the world frame returned by `HAL.getPose3d()`, which gives the exact pose of the robot (without odometry noise). The objects and targets of each world are listed in [Object and target lists](#object-and-target-lists).

### XLeRobot Warehouse

#### XLeRobot:

- The robot base is 0.39 meters long and 0.46 meters wide. It starts at (0, 0) facing +x.
- The two arms are mounted at (-0.09, -0.11, 0.765) (arm named left) and (-0.09, 0.11, 0.765) (arm named right) in `base_footprint`.
- Arm joints, in order: `Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch` and `Wrist_Roll`, with suffix `_L` (left arm) or `_R` (right arm). Limits in radians: `Rotation` [-2.16, 2.16], `Pitch` [-0.22, 3.37], `Elbow` [-0.22, 3.14], `Wrist_Pitch` [-1.658, 1.658], `Wrist_Roll` [-2.744, 2.841].
- Jaw joints `Jaw_L` and `Jaw_R`: `-0.3745` closed, `1.7453` fully open. A 5 cm object is held with the jaw at about `0.55`.
- Head joints `head_pan_joint` [-3.2, 3.2] and `head_tilt_joint` [-0.76, 1.45]. The head carries an RGB-D camera of 640x480 with a horizontal field of view of 60º.
- Each wrist carries an RGB camera of 640x480 with a horizontal field of view of 86º.
- The robot tray has one spot for each arm.

#### Warehouse:

- The warehouse is 15 meters long and 15 meters wide, closed by walls at x = ±7.5 and y = ±7.5.
- There are four conveyors of 2.5 x 1 m and 0.74 m tall: `conveyor1` at (5, 0), `conveyor2` at (0, 5), `conveyor3` at (-5, 0) and `conveyor4` at (0, -5). `conveyor1` and `conveyor3` run along the y axis, `conveyor2` and `conveyor4` along the x axis.
- The trays are 25 x 25 cm with a 3 cm rim.

### MMO-500 Warehouse

#### MMO-500:

- The robot base is 0.80 meters long and 0.57 meters wide. It starts at (0, 0) facing +x.
- The UR10 arm base is at (0.205, 0, 0.802) in `base_footprint`. Arm joints, in order: `ur10_shoulder_pan_joint`, `ur10_shoulder_lift_joint`, `ur10_elbow_joint`, `ur10_wrist_1_joint`, `ur10_wrist_2_joint` and `ur10_wrist_3_joint`. Limits: [-2π, 2π] for every joint except `ur10_elbow_joint`, which is limited to [-π, π].
- The Robotiq 2F-85 gripper is driven by `robotiq_85_left_knuckle_joint`: `0.0` open, `0.8` closed.
- The front laser is at (0.442, 0, 0.372) and the back laser at (-0.442, 0, 0.372), looking backwards. Each one gives 720 values over ±1.48 rad, from 0.1 to 30 m.

#### Warehouse:

- The warehouse is 15 meters long and 15 meters wide, closed by walls at x = ±7.5 and y = ±7.5.
- There are four conveyors of 2.5 x 1 m and 1 m tall: `conveyor1` at (5, 0), `conveyor2` at (0, 5), `conveyor3` at (-5, 0) and `conveyor4` at (0, -5). `conveyor1` and `conveyor3` run along the y axis, `conveyor2` and `conveyor4` along the x axis.
- The trays are 25 x 25 cm with a 3 cm rim.

### myAGV mechArm Warehouse Delivery

#### myAGV mechArm 270:

- The robot base is 0.31 meters long and 0.20 meters wide, and its corners stick out up to 0.195 m from its centre. It starts at (1, 0) facing +x.
- The mechArm 270 base is at (0.03, 0, 0.132) in `base_footprint`. Arm joints, in order, with their limits in radians: `joint1_to_base` [-2.7925, 2.7925], `joint2_to_joint1` [-1.3089, 2.0943], `joint3_to_joint2` [-3.0543, 1.1344], `joint4_to_joint3` [-2.7052, 2.7052], `joint5_to_joint4` [-2.0071, 2.0071] and `joint6_to_joint5` [-3.14, 3.14].
- The gripper is driven by `gripper_base_to_gripper_left3`: `0.15` open (about 43 mm), `-0.4` closed (about 22 mm).
- The wrist cannot point straight down close to the base: the arm reaches the floor with the gripper 70º below the horizon (`HAL.GRASP_ROLL = -π + 20º`, `HAL.GRASP_YAW = -π/2`), between about 0.22 and 0.28 m ahead of the base centre.
- The lidar is at (0.065, 0, 0.08), rotated 180º about z. It gives 360 values over [-π, π], from 0.12 to 8 m.
- The front camera is at (0.13, 0, 0.131) looking forward, 640x480 with a horizontal field of view of 60º.

#### Warehouse:

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

### Why does the robot sometimes fail to move to a desired pose?

The most likely reason is that your specified pose is unreachable for the robot arm, so MoveIt cannot plan a trajectory from the current pose to the desired pose within the allowed time. You will see a warning like this when that happens:

```bash
Fail: ABORTED: No motion plan found. No execution attempted.
```

### Which arm of the XLeRobot is which?

The arm named left is mounted at y = -0.11 m in `base_footprint` and the arm named right at y = +0.11 m.

### How is a delivery counted?

In the myAGV world, a ball counts as delivered when it lies inside its lit delivery box, the gripper is open and the ball has stayed still for 1 s. The box then flashes green and `getRedTarget()` / `getBlueTarget()` go back to `0`. Only one ball of each colour can be waiting at a time.

### Object and target lists

**XLeRobot Warehouse.** The three objects are located on the east conveyor (`conveyor1`), which is 0.74 m tall. The pose is the centre of the object. Each one goes to the tray of its colour.

- **blue_box** — Size (l,w,h) = (5,5,5) cm — Pose (x,y,z) = (4.55, -0.15, 0.765) m
- **green_cylinder** — Size (r,h) = (2.5,5) cm — Pose (x,y,z) = (4.55, 0.15, 0.765) m
- **yellow_box** — Size (l,w,h) = (5,5,5) cm — Pose (x,y,z) = (4.55, 0.6, 0.765) m

The three trays are located on conveyors that are 0.74 m tall.

- **green_tray** — Pose (x,y) = (0, 4.63) m
- **yellow_tray** — Pose (x,y) = (-4.63, 0) m
- **blue_tray** — Pose (x,y) = (0, -4.63) m

**MMO-500 Warehouse.** The three objects are located on the east conveyor (`conveyor1`), which is 1 m tall. The pose is the centre of the object. Each one goes to the tray of its colour.

- **blue_box** — Size (l,w,h) = (12,5,7) cm — Pose (x,y,z) = (4.8, 0, 1.035) m
- **yellow_ball** — Size (r) = (4) cm — Pose (x,y,z) = (4.8, -0.35, 1.04) m
- **green_cylinder** — Size (r,h) = (4,10) cm — Pose (x,y,z) = (4.8, 0.35, 1.05) m

The three trays are located on conveyors that are 1 m tall.

- **green_tray** — Pose (x,y) = (0, 4.8) m
- **yellow_tray** — Pose (x,y) = (-4.8, 0) m
- **blue_tray** — Pose (x,y) = (0, -4.8) m

**myAGV mechArm Warehouse Delivery.** The balls (radius 1.5 cm) drop into a cup on the floor at the end of each dispenser.

- **red dispenser** — Pose (x,y) = (6.671, -2.0) m
- **blue dispenser** — Pose (x,y) = (6.671, -6.0) m

The five delivery boxes (22 x 22 cm, walls 6 cm tall with their top at z = 0.072) are on the floor.

- **box 1** — Pose (x,y) = (-3.6, 6.0) m
- **box 2** — Pose (x,y) = (-3.6, -3.5) m
- **box 3** — Pose (x,y) = (1.0, -8.6) m
- **box 4** — Pose (x,y) = (1.0, 5.5) m
- **box 5** — Pose (x,y) = (4.0, 0.8) m

## References

1. XLeRobot. [https://github.com/Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot)
2. Neobotix ROS 2 simulation. [https://github.com/neobotix/neo_simulation2](https://github.com/neobotix/neo_simulation2)
3. Elephant Robotics myAGV and mechArm. [https://www.elephantrobotics.com/](https://www.elephantrobotics.com/)
4. IFRA-Cranfield (2023). ROS 2 Sim-to-Real Robot Control. [https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
5. [https://moveit.ros.org/](https://moveit.ros.org/)
6. [https://gazebosim.org/](https://gazebosim.org/)
