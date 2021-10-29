---
permalink: /exercises/MobileRobots/vacuum_cleaner_loc
title: "Localized Vacuum Cleaner"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Localized Vacuum Cleaner"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
    image_path: /assets/images/exercises/vacuum_cleaner_loc/vacuum_cleaner.png
    alt: "Vacuum"

youtubeId1: I967nzeSSZg
youtubeId2: mT5PkgtDLDg
---
## Versions to run the exercise

- Web Templates(Current Release)

## Goal

The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum cleaner by making use of the location of the robot. The robot is equipped with a map and knows it's current location in it. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

<img src="/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/vacuum_cleaner.png" width="100%" height="60%">
{% include gallery caption="Vacuum cleaner" %}

## Instructions for Web Templates
This is the preferred way for running the exercise.

### Installation 
- Clone the Robotics Academy repository on your local machine

	```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
	```

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

	```bash
docker pull jderobot/robotics-academy:3.1.6
	```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

### Enable GPU Acceleration
- For Linux machines with NVIDIA GPUs, acceleration can be enabled by using NVIDIA proprietary drivers, installing  [VirtualGL](https://virtualgl.org/) and executing the following docker run command:
  ```bash
  docker run --rm -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:3.1.6 ./start.sh
  ```


- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background ([hardware accelerated version](#enable-gpu-acceleration))

	```bash
  docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:3.1.6 ./start.sh
  ```

- On the local machine navigate to 127.0.0.1:8000/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

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

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Simulation Button**: Opens a VNC with GZClient.

* **Console Button**: This shows the error messages related to the student’s code that is sent. The student can also use it to visualize the output of the `print()` function.

* **Grid Button**: Toggles the visualization of the navigation matrix. It must be sent using the API in order to visualize it.

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.

* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity

* `HAL.getPose3d().x` - to get the X coordinate of the robot
* `HAL.getPose3d().y` - to get the Y coordinate of the robot
* `HAL.getPose3d().yaw` - to get the orientation of the robot
* `HAL.getBumperData().state` - To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* `HAL.getBumperData().bumper` - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its left and 2 if the collision is at its right.
* `HAL.getLaserData()` - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
* `GUI.showNumpy(mat)` - Displays the numpy matrix sent. It supports 3 different colors: 0 - grey, 1 - green, 2 - yellow, 3 - red. The tool button **Grid** must be turned on in order to visualize the grid.

```python
# Example of how to visualize a matrix
nav_mat = np.zeros((20, 20), int) # grey color
nav_mat[0, 0] = 1 # green color
nav_mat[1, 1] = 2 # yellow color
nav_mat[2, 2] = 3 # red color
GUI.showNumpy(nav_mat)
```

For this example, it is necessary to ensure that the vacuum cleaner covers the highest possible percentage of the house. The application of the automatic evaluator (referee) will measure the percentage traveled, and based on this percentage, will perform the qualification of the solution algorithm.

### Types conversion

**Laser**

```python
laser_data = HAL.getLaserData()

def parse_laser_data(laser_data):
    laser = []
    for i in range(180):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser
```

```python
def laser_vector(laser):
    laser_vectorized = []
    for d,a in laser:
        # (4.2.1) laser into GUI reference system
        x = d * math.cos(a) * -1
        y = d * math.sin(a) * -1
        v = (x,y)
        laser_vectorized += [v]

    laser_mean = np.mean(laser_vectorized, axis=0)
    return laser_mean
```

## Demonstration video

{% include youtubePlayer.html id=page.youtubeId2 %}

*This solution is an illustration for the Web Templates*

<br/>

## Theory
As the problem may have multiple solutions, only the theory behind the reference solution has been covered. 

### Conversion From 3D to 2D
**Robot Localization** is the process of determining, where robot is located with respect to it's environment. Localization is a an important resource to us in solving this exercise. Localization can be accomplished in any way possible, be it Monte Carlo, Particle Filter, or even Offline Algorithms. Since, we have a map available to us, offline localization is the best way to move forward. Offline Localization will involve converting from a 3D environment scan to a 2D map. There are again numerous ways to do it, but the technique used in exercise is using **transformation matrices**.

#### Transformation Matrices
In simple terms, transformation is an invertible function that maps a set _X_ to itself. Geometrically, it moves a point to some other location in some space. Algebraically, all the transformations can be mapped using matrix representation. In order to apply transformation on a point, we multiply the point with the specific transformation matrix to get the new location. Some important transformations are:

- **Translation**

Translation of Euclidean Space(2D or 3D world) moves every point by a fixed distance in the same direction

- **Rotation**

Rotation spins the object around a fixed point, known as center of rotation.

- **Scaling**

Scaling enlarges or diminishes objects, by a certain given scale factor.

- **Shear**

Shear rotates one axis so that the axes are no longer perpendicular.

In order to apply multiple transformations all at the same time, we use the concept of **Transformation Matrix** which enables us to multiply a single matrix for all the operations at once!

![Transformation Matrix](https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcRj8-LHyAz5s62zEx9fnQg_KLZX08E_rbdfHQ52kQZwh3BvPqMl)

*Transformation Matrix*

In our case we need to map a 3D Point in gazebo, to a 2D matrix map of our house. The equation used in the exercise was:

![Coordinate to Pixel]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/coord2pix.png)

*Coordinate to Pixel Conversion Equation*

In order to carry out the inverse operation of 3D to 2D, we can simply multiply, the pixel vector with the inverse of the transformation matrix to get the gazebo vector. The inverse of the matrix exists because the **mapping is invertible** and we **do not care about the z coordinate of the environment**, implying that each point in gazebo corresponds to a single point in the map.

### Coverage and Decomposition
After the robot is localized in it's environment, we can employ decomposition techniques in our algorithm, to deal with the actual coverage of the surroundings. There are lot of [decomposition techniques](https://www.cs.cmu.edu/~motionplanning/lecture/Chap6-CellDecomp_howie.pdf) available for our use. The Decomposition Algorithm, decomposes the map into separate segments, which our robot can cover one by one. Decomposition can be directly related to **Graph Theory**, where the segments are taken as nodes and the edges connecting nodes depict that the adjacent segments share a common boundary. The robot can path plan to the nearest node and then start the sweeping again! Most of the details regarding decomposition would be implementation

![Decomposition and Graph Theory]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/adj_graph.png)

*Adjacency Graph*

### Travelling between segments
Once, a certain segment has been swept. In order to reach the next segment(preferably nearest one) there are again a multitude of [path planning algorithms](http://correll.cs.colorado.edu/?p=965). The algorithm used in the reference solution is the **Visibility Algorithm**. As the name suggests, visibility of the target is the basic building block. So, let's define visibility first off all!

If a straight line exists between two points, and the line does not pass over obstacles, the two points are said to be visible to each other. Mathematically speaking, the two points under consideration must satisfy a common equation.

![Equation of a Line](https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcRmpTMXyRw9cDm_0BWW6FTuBdXrBnozMP2uLGrwOQBev5hRwPK9) 

*Equation of a Line*

Starting from the destination cell, the robot can map it's path one cell at a time, while keeping visibility as a reference, until it reaches the current cell, the robot is present in. Once, the plan has been decided the robot can follow that path and reach it's destination.

![Error of Obstacles]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/error.png)

*Visibility Error*

But, sometimes due to close proximity to corner, the robot may collide with it. To avoid the occurrence of any such event, we may also consider dilation techniques, since the map used is a binary image. **Erosion** is a **Morphology Function** that erodes away the boundaries of foreground object(white is the object in foreground). Refer to this [link](https://homepages.inf.ed.ac.uk/rbf/HIPR2/erode.htm) for more information on erosion.

![Morphological Operations](https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcTzn6m8Kkpb9OUy-mv70GpKRmsd3hySZBJZH8n5y-OLO4jBq9mW)

*Morphological Operations*

## Hints
Simple hints provided to help you solve the vacuum_cleaner_loc exercise. Please note that the **full solution has not been provided.** Also, the hints are more related to the reference solution, since multiple solutions are possible for this exercise.

### Grid Representation
Due to the involvement of Localization in this exercise, we are given a map of the surroundings of the robot, which it needs to cover. The map is present in the directory path `resources/images/`. But since, we are using path planning and decomposition algorithms. These are very easy to implement on a grid like structure. Although any data structure, representing adjacency graph would suffice. See the [illustrations](#Illustrations) for what happens without using any data structure(a brute force approach). To implement grids on images, we need to apply basic image processing, which can be accomplished easily using the `opencv` library.

The first step would be to apply erosion function on the map image. This is done in order to ensure some degree of distance between our robot and obstacle during runtime.

The second task is to come up with a **transformation matrix**, that would convert our 3d gazebo positions into 2d map representations. A degree of rotation has to applied on the Y axis and add a translation of 0.6 on the X axis and -1 on the Y axis can be used to get the origin of coordinates (0, 0) of the image in the upper left hand corner.

Once we get **specific positions** on our map. Using the dimensions of our vacuum cleaner, we can generate grid cells around those points. The size of vacuum cleaner can be taken as 16x16 pixel units. Take it as an exercise to calculate it on your own!

### Obstacles and Color Coding
Taking the map as a grayscale image, we can apply **color coding** to the map as well. It is convenient to separate **Obstacles**, **Virtual Obstacles**, **Return Points** and **Critical Points**

In the context of this exercise,
- **Obstacles**: The obstacles in our map that our robot cannot go over
- **Virtual Obstacles**: The grid cell that our robot has covered.
- **Return Points**: Starting points of the decomposed cells
- **Critical Points**: The points where our robot has stuck between obstacles and virtual obstacles. It is from critical points, that our robot has to start moving towards the next return point.

### Checking for Points
Defining different points simplifies the algorithm as well!

- **Return Points**
Return Points can be checked while the robot is in zig-zag motion. Empty cells near the robot can be classified as return points. The list of Return Points has to be kept dynamic in order to insert and pop the points whenever required.

- **Critical Points**
Critical Points can be classified whenever our robot is stuck around Obstacles or Virtual Obstacles and cannot move any further.

- **Checking for Arrival of Cell**
Considering various offset errors in the real world(consider simulation to be real world as well!). Arrival of the robot in a particular cell should be considered within a margin of error, otherwise the robot may start oscillating, in the search of a cell on which it can never arrive.

- **Efficient Turning and Straight Motion**
One trick to adjust the speed and direction of motion is to keep the next 3 cells of the robot which are in it's direction of motion under consideration.

- **All 3 cells are free** Make the robot move at maximum speed
- **2 cells are free**  Make the robot move at a fast speed
- **1 cell is free** Along with small linear motion forward, start applying rotation as well
- **0 cells are free** Stop the robot and apply only rotation

As a final note, quite a lot of tips and tricks regarding implementation have been discussed in this page. This is a tough exercise, which may take **quite a lot of time** to solve. The main objective of the exercise is to cover a **significant area of the house**, without taking time into consideration.

## Illustrations
![Grid Based]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/grid.gif)

*Grid Based Approach*

![PID Based]({{ site.url }}/RoboticsAcademy/assets/images/exercises/vacuum_cleaner_loc/pid.gif)

*PID Based(without grid) Approach*

## Contributors

- Contributors: [Vanessa Fernandez](https://github.com/vmartinezf), [Jose María Cañas](https://gsyc.urjc.es/jmplaza/), [Carlos Awadallah](https://github.com/cawadall), [Nacho Arranz](https://github.com/igarag).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna).

## References
The major credit for this coverage algorithm goes to [Jose María Cañas](https://github.com/jmplaza) and his student [Irene Lope](https://github.com/ilope236).

[1](https://onlinelibrary.wiley.com/doi/full/10.1002/047134608X.W8318)
[2](https://en.wikipedia.org/wiki/Transformation_(function))
[3](https://www.cs.cmu.edu/~motionplanning/lecture/Chap6-CellDecomp_howie.pdf)
[4](http://correll.cs.colorado.edu/?p=965)
[5](https://homepages.inf.ed.ac.uk/rbf/HIPR2/dilate.htm)
[6](https://gsyc.urjc.es/jmplaza/students/tfg-Robotics_Academy-irene_lope-2018.pdf)

One of the best [video series on Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab)
