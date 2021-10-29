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
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse.png
    alt: "Warehouse"
    title: "Warehouse"
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse_teaser.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse_teaser.png
    alt: "MobileRobot"
    title: "MobileRobot"
  - url: /assets/images/exercises/amazon_warehouse/amazon_warehouse_2.png
    image_path: /assets/images/exercises/amazon_warehouse/amazon_warehouse_2.png
    alt: "Amazon"
    title: "Amazon"

theory:
  - url: /assets/images/exercises/amazon_warehouse/navstack.png
    image_path: /assets/images/exercises/amazon_warehouse/navstack.png
    alt: "Navigation Stack"
    title: "Navigation Stack"
  - url: /assets/images/exercises/amazon_warehouse/action.png
    image_path: /assets/images/exercises/amazon_warehouse/action.png
    alt: "Action Server and Client"
    title: "Action Server and Client"

youtubeId: T1-6Y4ulEnQ
---

## Goal

The objective of this practice is to implement the autonomous robot navigation and pick-and-place logic in warehouse.

{% include gallery caption="Gallery" %}

The students program an Amazon robot to navigate to the seleceted pallet, pick it, and deliver to the required zone. After completion of the task, go the charging area, and wait for the next tasks.

## Installation
Install the [General Infrastructure](https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure) of the JdeRobot Robotics Academy.

Add following packages if you don't have them already:

```bash
sudo wget https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml -P /opt/ros/kinetic/share/hector_pose_estimation/
sudo apt-get install ros-melodic-kobuki-msgs
sudo apt-get install ros-melodic-yocs-cmd-vel-mux
sudo apt-get install ros-melodic-navigation
```

## How to run your solution?

Run Gazebo simulator:

```bash
cd launch
ROS_HOME=`pwd` roslaunch amazonrobot_1_warehouse.launch 
```

Navigate to exercise folder and run the practice and the user interface: 

```bash
python2 amazonWarehouse.py amazonMap.conf amazonConf.yml
```

To simplify the closure of the environment, just close the Amazon window (s). *`Ctrl + C` will give problems.*

## How to do the practice
To carry out the practice, you must edit the `MyAlgorithm.py` file and insert the control logic into it.

### Where to insert the code
MyAlgorithm.py

```python
    def execute(self):
        # Add your code here
        print("Running")

        # LIFT PALLET
        self.liftDropExecute()

        # TO DO
```

## Robot API
* `self.client.sendGoalToClient(x, y)` - to send goal to the move_base client
* `self.client.getResultFromClient()` - to send get result from client. None if not reached the goal
* `self.grid.getDestiny()` - to get destination of the robot after double click on GUI
* `self.path.getPath()` - to obtain the path to the current goal
* `self.liftDropExecute()` - to lift or drop pallet
* `clearCostmaps()` - to clear costmaps after moving with pallet

## Theory

ROS Navigation Stack forms the backbone of this exercise. The navigation stack is quite simple on a conceptual level. It takes information from odometry and sensor streams and outputs velocity commands to robot controller. The most difficult part of ROS Navigation Stack is configuring it to work with the robot, which has been taken care of, leaving the student with only using the Simple Action Client. Let's cover all these concepts one by one:

### Navigation Stack

At its core, the navigation stack system allows a ROS enabled robot to move about the world to a specified goal position efficiently, without hitting the obstacles on its way. It integrates information from the map, localization system, sensors and odometry to plan a good path from the current position to the goal position, and then follows it to the best of robot's ability.

The basic steps in its working are:

1. A navigation goal is sent to the navigation stack. This is done using an action call with a goal of type *MoveBaseGoal*, which specifies a goal pose (position and orientation) in some coordinate frame (commonly called the *map* frame).

2. The nav stack uses a path planning algorithm in the *global planner* to plan the shortest path from the current location to the goal, using the map. Global Path Planning is covered in another [exercise](https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/). More about it over there!

3. This path is passed to the *local planner*, which tries to drive the robot along the path. The local planner uses information from the sensors in order to avoid obstacles that appear in front of the robot but that are not in the map, such as people. **If the local planner gets stuck and cannot make progress, it can ask the global planner to make a new plan and then attempt to follow that.** Local Path Planning is covered in another [exercise](https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/obstacle_avoidance). More about it over there!

4. When the robot gets close to the goal pose, the action terminates and we're done.


### ROS Actions

ROS system consists of a number of independent nodes that comprise a graph. In order for something useful to take place, the nodes have to communicate with each other. This is accomplished by use of **topics**. A topic is a name for a stream of messages with a defined type. Topics implement a *publish/subscribe* communication mechanism to exchange data in the distributed system. Before nodes start to transmit data over topics, they must first announce, or *advertise*, both the topic name and the types of messages that are going to be sent. Then they can start to send, or *publish* the actual data on the topic. Nodes that want to receive messages on a topic can *subscribe* to the topic.

ROS Services provide a way to execute synchronous remote procedure calls; calling a function in one node that executes in another node. The server(which provides the service) specifies a callback to deal with the service request, and advertises the service. The client(which calls the service) then accesses the service through a local proxy.

ROS Actions provide a way to execute asynchronous remote procedure calls. Similar to the request and response of a service, an action uses a *goal* to initiate a behaviour and sends the *result* when the behaviour is complete. But the action further uses *feedback* to provide updates on the behaviour's progress toward the goal and also allows for goals to be cancelled. 

Hence, services are handy for simple get/set instructions like querying status and managing configuration, they don't work well when we need to initiate a long running task like navigation. This is exactly where actions can be used.

{% include gallery id="theory" caption="Theory" %}

## Hints
Simple hints provided to help you solve the amazon_warehosue exercise.

The main problem in this exercise is to correctly identify the goal and send it to the action server.

### Locations

* The locations of the pallets to be stored are already present in the `pallets_coords.yaml` file
* The boundaries for *Pick-up room* are: `(x > 310) and (y > 125) and (y < 185)`
* The boundaries for *Charging Point* are : `(y > 255) and (x < 315) and (x > 85)`

## Demonstrative video of completed solution

{% include youtubePlayer.html id=page.youtubeId %}

## References

Programming Robots with ROS, by Morgan Quigley, Brian Gerkey & William D.Smart

[The ROS Wiki](http://wiki.ros.org/navigation)
