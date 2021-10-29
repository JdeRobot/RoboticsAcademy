---
permalink: /exercises/IndustrialRobots/machine_vision
title: "Machine Vision"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Machine Vision"
toc_icon: "cog"

gallery:
  - url: /assets/images/exercises/machine_vision/world_secondexercise.png
    image_path: /assets/images/exercises/machine_vision/world_secondexercise.png
    alt: "Machine Vision"
    title: "Gazebo World"
  - url: /assets/images/exercises/machine_vision/rviz_secondexercise.png
    image_path: /assets/images/exercises/machine_vision/rviz_secondexercise.png
    alt: "Machine Vision"
    title: "Rviz"

robot_teleop:
  - url: /assets/images/exercises/machine_vision/newGUI_secondexercise.png
    image_path: /assets/images/exercises/machine_vision/newGUI_secondexercise.png
    alt: "Robot Teleoperator"
    title: "Robot Teleoperator"

green_cylinder_filter:
  - url: /assets/images/exercises/machine_vision/green_filter_image.png
    image_path: /assets/images/exercises/machine_vision/green_filter_image.png
    alt: "green_filter_image"
    title: "green_filter_image"
  - url: /assets/images/exercises/machine_vision/green_cylinder_image.png
    image_path: /assets/images/exercises/machine_vision/green_cylinder_image.png
    alt: "green_cylinder_image"
    title: "green_cylinder_image"

green_cylinder_filter:
  - url: /assets/images/exercises/machine_vision/green_filter_image.png
    image_path: /assets/images/exercises/machine_vision/green_filter_image.png
    alt: "green_filter_image"
    title: "green_filter_image"
  - url: /assets/images/exercises/machine_vision/green_cylinder_image.png
    image_path: /assets/images/exercises/machine_vision/green_cylinder_image.png
    alt: "green_cylinder_image"
    title: "green_cylinder_image"

yellow_cylinder_filter:
  - url: /assets/images/exercises/machine_vision/yellow_filter.png
    image_path: /assets/images/exercises/machine_vision/yellow_filter.png
    alt: "yellow_filter"
    title: "yellow_filter"
  - url: /assets/images/exercises/machine_vision/yellow_cylinder_filter.png
    image_path: /assets/images/exercises/machine_vision/yellow_cylinder_filter.png
    alt: "yellow_cylinder_filter"
    title: "yellow_cylinder_filter"

youtubeId: LHq4ZA2lGxQ
---

## Versions to run the exercise

Currently, there are 2 versions for running this exercise:

- ROSNode Templates
- Web Templates(In Development)

The instructions for both of them are provided as follows.
## Goal

The goal of this exercise is to learn how to use vision to assist industrial robot by detecting known objects and unknown obstacles. You will need to complete pick and place task using a robot arm and vacuum gripper. Two kinect camera, one fixed to the world and another one fixed to the robot end effector, will be provided. The shape, size and color of the objects are known, but the pose of them and the situation of obstacles in surrounding environment need to be found using two cameras.

{% include gallery caption="Gallery." %}

## Instructions for web-templates

This is the preferred way for running the exercise once it is finished.

### Installation 

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

	```bash
  docker pull jderobot/robotics-academy:2.4.2
  ```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

### Enable GPU Acceleration
- For Linux machines with NVIDIA GPUs, acceleration can be enabled by using NVIDIA proprietary drivers, installing  [VirtualGL](https://virtualgl.org/) and executing the following docker run command:
  ```bash
  docker run --rm -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:2.4.2 ./start.sh
  ```


- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background ([hardware accelerated version](#enable-gpu-acceleration))

	```bash
  docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:2.4.2 ./start.sh
  ```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Click the connect button and wait for some time until an alert appears with the message `Connection Established` and button displays connected. 

- The exercise can be used after the alert.

**Where to insert the code?**

In the launched webpage, type your code in the text editor,

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!


while True:
    # Enter iterative code!
```

### Using the Interface

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Robot. Stop button stops the code that is currently running on the Robot. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation(primarily, the position of the robot).

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as recal time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student can use the `print()` command in the Editor. 

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.back_to_home()` - Command the robot arm and gripper to move back to the home pose.
* `HAL.pickup()` - to set the linear speed
* `HAL.place()` - to set the angular velocity
* `HAL.move_pose_arm(pose_goal)` - Command the robot with Pose message to make its end effector frame move to the desired pose with inverse kinematics.
* `HAL.move_joint_hand(joint_value)` - Command the gripper joint to move to desired joint value.
* `HAL.start_color_filter(color, rmax, rmin, gmax, gmin, bmax, bmin)` - Start color filter for the given color with RGB range for this color. Two messages, $(color)_filter(pointcloud) and $(color)_filtered_image (RGB image), will be published until stopping with stop_color_filter function. Range of RGB value is [0, 255].
* `HAL.stop_color_filter(color)` - Stop the color filter of the given color.
* `HAL.start_shape_filter(color, shape, radius)` - Start the shape filter to detect one set of pointcloud that can represent the input shape and has a smaller than input radius size from color filtered pointcloud. If the filter can detect a possible shape from pointcloud, two messages, color_shape(pointcloud) and color_shape_image(depth image), will be published until stopping with stop_shape_filter function. A frame called color_shape will be added in tf tree. The origin of the frame is the center of predicted sphere center of a point in the center axis of predicted cylinder. The unit of radius is meter(m).
* `HAL.stop_shape_filter(color, shape)` - Stop shape filter.
* `HAL.get_object_position(color, shape)` - If a frame named with color_shape exists in the tf tree, output is the transformation from this frame to world frame. If not, output is None.
* `HAL.get_object_position(object_name)` -  If a frame named with color_shape exists in tf tree, output is the transformation from this frame to world frame. If not, output is None.
* `HAL.get_target_position(target_name)` - Return the position of target where we are going to place the objects.

## Instructions for ROSNode Templates
## Installation
1. Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.
2. Install PCL and octomap related packages
```bash
sudo apt-get install ros-melodic-octomap-ros
sudo apt-get install ros-melodic-octomap-msgs
sudo apt-get install ros-melodic-octomap-rviz-plugins
sudo apt install libpcl-dev
sudo apt-get install ros-melodic-perception-pcl
```
3. Install Industrial Robot package
If you have done this part in other Industrial Robots exercises, you can skip to next section.
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
```
after pull request to the industrial robot repo
```bash
git clone https://github.com/JdeRobot/IndustrialRobotics.git -b melodic_devel
cd ..
```
Update ROS dependencies
```bash
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro melodic
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
```
Build workspace
```bash
catkin build
```
Export environment variables
```bash
echo 'source '$PWD'/devel/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:'$PWD'/src/IndustrialRobotics/assets/models' >> ~/.bashrc
source ~/.bashrc
```

## How to run the exercise
TO launch the exercise, open a terminal windows, navigate to `machine_vision` folder inside `exercises` folder and execute following command:
```bash
roslaunch machine_vision.launch 
```
Three different windows will pop up:
- **Gazebo simulator**: A warehouse environment with a industrial robot(robot arm and gripper), multiple objects, two kinect cameras, a conveyor, a table and a box will be shown in Gazebo.
- **Rviz**: Robot arm, cameras, model of objects, the pointcloud of fixed camera and the octomap build from the pointcloud of camera on robot will be shown in Rviz.
- **Industrial robot teleoperator**: A GUI which provides following functionalities:
    - A Forward Kinematics teleoperator providing sliders to change the angle of each joint of the robot arm and the gripper. The limits of each joints are shown in the two sides of the slider. The true angle of the joints are shown in the text box beside the sliders.
    - An Inverse Kinematics teleoperator allowing users to set the desired end effector pose. 
        - Plan button can plan the trajectory to the desired pose
        - Execute button can make the robot execute the planned trajectory
        - Plan & Execute button is the integration of last two buttons
        - Stop button allows users to stop the robot
        - Back to home button can make the robot move back to home pose
    - Two update button can update the value of current robot state
    - Code Manager part
        - Four buttons to start, stop, pause and restart the main code you program
        - One button to launch Rviz
        - One button to Respawn all objects in Gazebo and Rviz
        - An info box showing the status of the main code("start" or "stop")
    - Camera viewer
        - Two combo boxes to choose image topics to show in below image windows
        - Two image windows to show chosen image topics

{% include gallery id="robot_teleop" caption="Robot teleoperator GUI" %}

Then open a new terminal window, navigate to `machine_vision` folder inside `exercises` folder and execute following command:
```bash
python MyAlgorithm.py
```
You can start running the algorithm with the start button when you see `You can start your algorithm with GUI` in the terminal.

## How should I solve the exercise
To solve the exercise, you must edit the MyAlgorithm.py file and insert control logic in myalgorithm() function. Before writing the main logic, you should implement build_map() function to detect obstacles in surrounding environments and get_object_position() function to detect the position of the objects.
```python
def build_map(self):
    ############## Insert your code here ###############
    self.pick_place.send_message("Building map")

    ####################################################

def get_object_position(self, object_name):
    ############## Insert your code here ###############

    return position
    ####################################################

def myalgorithm(self):
	############## Insert your code here ###############
    self.build_map()

    # Move the robot back to home as a start
    self.pick_place.back_to_home()
	
    # insert following two lines where you want to pause or stop the algorithm 
    # with the stop button in GUI
    while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
        if not self.stopevent.isSet():
            return

    ##### A brief example to pick and place object "green_cylinder" #####

    # get object position and pick it up
    # parameters HEIGHT_OFFSET need to be tuned according to the object height
    # Height offset between robot base and conveyor is 0.2m
    object_name = "green_cylinder"

    position = self.get_object_position(object_nam)
    self.pick_place.pickup(object_name, position)

    # setup stop signal detector
    while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
        if not self.stopevent.isSet():
            return

    # choose target position and place the object
    target_name = "target6"
    position = self.pick_place.get_target_position(target_name)
    self.pick_place.place(object_name, position)

    ####################################################


```  
Multiple APIs can be used to implement your algorithm. They are provided in Pick_Place class, so you should always add "self.pick_place." as a prefix to following introduced APIs in your algorithm.

## API
### Detect Object
* `start_color_filter(color, rmax, rmin, gmax, gmin, bmax, bmin)` - Start color filter for given `color` with RGB range for this `color`. Two messages, `$(color)_filter`(pointcloud) and `$(color)_filtered_image`(RGB image), will be published until stopping with `stop_color_filter` function. Range of RGB value is [0, 255]
* `stop_color_filter(color)` - Stop the color filter for given `color`.
* `start_shape_filter(color, shape, radius)` - Start the shape filter to detect one set of pointcloud that can represent the input `shape` and has a smaller than input `radius` size from color filtered pointcloud. If the filter can detect a possible shape from pointcloud, two messages, `$(color)_$(shape)`(pointcloud) and `$(color)_$(shape)_image`(depth image), will be published until stopping with `stop_shape_filter` function. A frame called `$(color)_$(shape)` will be added in tf tree. The origin of the frame is the center of predicted sphere center of a point in the center axis of predicted cylinder. The unit of radius is meter(m)
* `stop_shape_filter(color, shape)` - Stop the shape filter.
* `get_object_position(color, shape)` - If a frame named with `$(color)_$(shape)` exists in tf tree, output is the transformation from this frame to world frame. If not, output is None.

### Environment Information
* `get_object_list()` - Return the name list of all objects.
* `get_object_info(object_name)` - Return the height, width, length, shape, color of the object in order.
* `get_target_list()` - Return the name list of all targets.
* `get_target_position(target_name)` - Return the position of target where we are going to place the objects.

### Convert Pose Message
* `pose2msg(roll, pitch, yaw, x, y, z)` - Convert pose to Pose message. The unit of roll, pitch, yaw is radian.
* `pose2msg_deg(roll, pitch, yaw, x, y, z)` - Convert pose to Pose message. The unit of roll, pitch, yaw is degree.
* `msg2pose(pose)` - Convert Pose message to pose, return roll, pitch, yaw, x, y, z in order. The unit of roll, pitch, yaw is radian.
* `msg2pose_deg(pose)` - Convert Pose message to pose, return roll, pitch, yaw, x, y, z in order. The unit of roll, pitch, yaw is degree.

### Basic Robot Movement
* `move_pose_arm(pose_goal)` - Command the robot with Pose message to make its end effector frame move to the desired pose with inverse kinematics.
* `move_joint_arm(joint_0,joint_1,joint_2,joint_3,joint_4,joint_5)` - Command the robot joints to move to desired joints value
* `back_to_home()` - Command the robot arm and gripper to move back to the home pose.
* `gripper_grasp()` - Turn on the vacuum gripper.
* `gripper_release()` - Turn off the vacuum gripper.

### Pick and Place
* `pickup(object_name, position[, distance])` - Command the industrial robot to pick up the object by moving the end effector to given position. Distance is the distance the robot arm will move in z axis before and after grasping objects.
* `place(object_name, position[, distance])` - Command the industrial robot to place the object by moving the end effector to given position. Distance is the distance the robot arm will move in z axis before and after placing objects.

## Theory

### Object Detection
In this exercise, object detectors are implemented based on [Point Cloud Library(PCL)](https://pointclouds.org/). It is an opensource point cloud processing library. There are multiple methods to detect the pose of known objects with vision. The method we choose here is fully based on pointcloud.

Assume that we only know the color, size and shape(sphere or cylinder) of the object we want to pick. Firstly, a color filter will filter out all points that are not inside chosen RGB range. Secondly, a shape segmentation will be done to find points that have the highest possibility to be a part of chosen shape and size. Some parameters about this possible shape will also be provided by the segmentation filter. With these information, we are able to find the position of sphere or cylinder.

### Obstacle Detection and Avoidance
In `Pick and Place` exercise, we have introduced that objects in planning scene will be considered as obstacles when doing motion planning to avoid collision. Obstacles are known and added into planning scene in that exercise, but what if we have no information about the surrounding environment? Obstacle detection will be necessary. 

Obstacle detection can be done using many different sensors. What we use in this exercise is kinect camera which provides rgb image, depth image and pointcloud. Pointcloud from camera is what we finally use to build the obstacle map. Pointcloud contains the position of points and points can represent obstacles. If we merging these points and representing each of them with small cube, we will build a simplified map with obstacles in planning scene.

## Hints

### How to write buildmap() function:
The pointcloud from the camera fixed to the robot will be automatically monitored `occupancy_map_monitor/PointCloudOctomapUpdater` to update the planning scene, so what you have to do is move the robot around and make sure that the camera fixed to the robot can take image of all surrounding environment inside robot workspace.

### How to write get_object_position() function:
1. Get the color, shape and size of the object.
2. Choose RGB range for the color filter and start it. Check the color filter result with image viewer or Rviz and tune parameters to get better result.
3. Choose the radius for the shape filter and start it. Check the shape filter result and tune parameters to get better result.
4. Check the position output from `get_object_position` API. 
5. Return the position of detected object when getting a relatively stable object position.

**Note:** Please remember to stop the color filter and shape filter after getting the object position. If too many filters are running at the same time, you might not be able to get a stable result.

### How to check the result of color filter and shape filter?
Two image windows are provided in GUI. You can choose corresponding topic to check the result. All filtering process are done with pointcloud. The image is the projection of the pointcloud. From the image side, the expected result for color filter is that only the part with your chosen RGB value will be shown with original color in the image. All filtered out parts are black. The expected for shape filter is that only the points related to the detected shape will be shown with non-black color in the image. If nothing is detected, the whole image is black or you won't see the image change from the previously chosen image. Here are the results(topic: "green_filtered_image" and "green_cylinder_image" of detecting green cylinder.

{% include gallery id="green_cylinder_filter" caption="green cylinder filter image" %}

To view 3D pointcloud, you need to launch Rviz and change the topic of Poindcloud to the topic you want to check. If nothing is detected, you will see nothing new in Rviz after choosing the topic. Here are the pointcloud results(topic: "yellow_filter" and "yellow_cylinder") of detecting yellow cylinder.

{% include gallery id="yellow_cylinder_filter" caption="yellow cylinder filter pointcloud image" %}

### Limitation of vacuum gripper
Because the simulation of vacuum gripper is not perfect in this exercise, the robot sometimes cannot plan a path to place the object to target position or may drop the object on its way to target. Please take successfully building map and picking up desired objects as the sign of success.

### Ignorable ERROR and WARNING
- `No p gain specified for pid.`
- `gazebo_ros_vacuum_gripper: already status is 'off'`

Following three errors can be ignored in the beginning, but if it keep spawning when you run your algorithm or move the robot with GUI, please relaunch the exercise.
- `The complete state of the robot is not yet known.  Missing gripper_joint, gripper_joint1, gripper_joint2, gripper_joint3, gripper_joint4, gripper_joint5, gripper_joint6, gripper_joint7, gripper_joint8`
- `Transform error: "vacuum_gripper5" passed to lookupTransform argument source_frame does not exist.`
- `Transform cache was not updated. Self-filtering may fail.`

### Object and Target lists:
**Object list:**
- red_ball
- green_ball
- blue_ball
- yellow_ball
- red_cylinder
- green_cylinder
- blue_cylinder
- yellow_cylinder

**Target list:** target_ID, ID are integers from 1 to 16

## Demonstration video of the solution

{% include youtubePlayer.html id=page.youtubeId %}
