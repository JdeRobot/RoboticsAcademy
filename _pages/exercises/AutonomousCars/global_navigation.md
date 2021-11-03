---
permalink: /exercises/AutonomousCars/global_navigation/
title: "Global Navigation using TeleTaxi"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Global Navigation"
toc_icon: "cog"

<!--- layout: archive --->

<!--- classes: wide --->

gallery:
  - url: /assets/images/exercises/global_navigation/global_navigation.png
    image_path: /assets/images/exercises/global_navigation/global_navigation.png
    alt: "City View."
    title: "City View."
  - url: /assets/images/exercises/global_navigation/global_navigation_teaser.png
    image_path: /assets/images/exercises/global_navigation/global_navigation_teaser.png
    alt: "GUI."
    title: "GUI."
  - url: /assets/images/exercises/global_navigation/taxi.png
    image_path: /assets/images/exercises/global_navigation/taxi.png
    alt: "Model."
    title: "Model."

gifs:
  - url: /assets/images/exercises/global_navigation/miss.gif
    image_path: /assets/images/exercises/global_navigation/miss.gif
    alt: "examples"
    title: "examples"

  - url: /assets/images/exercises/global_navigation/two_step.gif
    image_path: /assets/images/exercises/global_navigation/two_step.gif
    alt: "examples"
    title: "examples"

potential:
  - url: /assets/images/exercises/global_navigation/potential_drop.png
    image_path: /assets/images/exercises/global_navigation/potential_drop.png
    alt: "Potential Drop"
    title: "Potential Drop"

  - url: /assets/images/exercises/global_navigation/potential_well.png
    image_path: /assets/images/exercises/global_navigation/potential_well.png
    alt: "Potential Well"
    title: "Potential Well"

  - url: /assets/images/exercises/global_navigation/potential_wall.png
    image_path: /assets/images/exercises/global_navigation/potential_wall.png
    alt: "Potential Wall"
    title: "Potential Wall"

  - url: /assets/images/exercises/global_navigation/total_potential.png
    image_path: /assets/images/exercises/global_navigation/total_potential.png
    alt: "Total Potential"
    title: "Total Potential"

Probability_Roadmap:
  - url: /assets/images/exercises/global_navigation/pr1.png
    image_path: /assets/images/exercises/global_navigation/pr1.png
    alt: "Step1"
    title: "Step1"

  - url: /assets/images/exercises/global_navigation/pr2.png
    image_path: /assets/images/exercises/global_navigation/pr2.png
    alt: "Step2"
    title: "Step2"

  - url: /assets/images/exercises/global_navigation/pr3.png
    image_path: /assets/images/exercises/global_navigation/pr3.png
    alt: "Step3"
    title: "Step3"

  - url: /assets/images/exercises/global_navigation/pr4.png
    image_path: /assets/images/exercises/global_navigation/pr4.png
    alt: "Step4"
    title: "Step4"

TreeBasedPlanner:
  - url: /assets/images/exercises/global_navigation/tr1.png
    image_path: /assets/images/exercises/global_navigation/tr1.png
    alt: "Step1"
    title: "Step1"

  - url: /assets/images/exercises/global_navigation/tr2.png
    image_path: /assets/images/exercises/global_navigation/tr2.png
    alt: "Step2"
    title: "Step2"

  - url: /assets/images/exercises/global_navigation/tr3.png
    image_path: /assets/images/exercises/global_navigation/tr3.png
    alt: "Step3"
    title: "Step3"

youtubeId1: zUtK6seVL5g
youtubeId2: q6G6BHqljP4
youtubeId3: itTbU4uLwfE
youtubeId4: zcS4X-ZO68U
youtubeId5: lO9Ru2mNAR4

---
## Versions to run the exercise

- Web Templates(Current Release)

## Goal

The objective of this practice is to implement the logic of a Gradient Path Planning (GPP) algorithm. Global navigation through GPP, consists of:

{% include gallery caption="Gallery" %}

* Selected a destination, the GPP algorithm is responsible for finding the shortest path to it, avoiding, in the case of this practice, everything that is not road.

* Once the path has been selected, the logic necessary to follow this path and reach the objective must be implemented in the robot.

With this, it is possible for the robot to go to the marked destination autonomously and following the shortest path.

The solution can integrate one or more of the following levels of difficulty, as well as any other one that occurs to you:

* Reach the goal.

* Optimize the way to find the shortest path.

* Arrive as quickly as possible to the destination.

## Instructions for Web Template
This is the preferred way for running the exercise.

### Installation 
- Clone the Robotics Academy repository on your local machine

```bash
git clone https://github.com/JdeRobot/RoboticsAcademy
```

- Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

- Pull the current distribution of Robotics Academy Docker Image

```bash
docker pull jderobot/robotics-academy:3.1.3
```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

#### Enable GPU Acceleration (For advanced users)
- For Linux machines with NVIDIA GPUs, acceleration can be enabled by using NVIDIA proprietary drivers and executing the following docker run command:
```bash
docker run --rm -it --device /dev/dri -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:3.1.3 ./start-3.1.sh
```

- For Windows machines, GPU acceleration to Docker is an experimental approach and can be implemented as per instructions given [here](https://www.docker.com/blog/wsl-2-gpu-support-is-here/)

### How to perform the exercise?
- Start a new docker container of the image and keep it running in the background

```bash
docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy:3.1.3 ./start-3.1.sh
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

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student can use the `print()` command in the Editor. 

## Robot API

* `from HAL import HAL` - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
* `from GUI import GUI` - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `from MAP import MAP` - This class contains functions that interact with data related to the map and Gazebo world. 
* `HAL.motors.sendV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity
* `HAL.getPose()` - returns x,y and theta components in the world
* `GUI.showNumpy(numpy)` - shows Gradient Path Planning field on the web-template. It represents the values of the field that have been assigned to the grid. Accepts as input a two-dimensional numpy array whose values can range from 0 to 255 (grayscale).   
* `GUI.showPath(array)` - shows ideal path on the map and the parameter should be 2D array
* `GUI.targetPose` - returns x,y coordinates of chosen destionation in the world 
* `MAP.getMap()` - Map image opencv data in opencv data
* `MAP.robotPose()` - returns x,y coordinates of robot on the map
* `MAP.setGridVal(x, y, val)` - sets the value val to the indicated position on the map
* `MAP.getGridVal(x, y)` - returns the value in that grid position on the map
* `MAP.gridToWorld(x, y)` - returns translated x,y coordinates in the world
* `MAP.worldToGrid(x, y)` - returns translated x,y coordinates on the map 

### API
* `sensor.getRobotX()` - to obtain the position of the robot
* `sensor.getRobotY()` - to obtain the position of the robot
* `sensor.getRobotTheta()` - to obtain the orientation of the robot with respect to the map
* `vel.setV()` - to set the linear speed
* `vel.setW()` - to set the angular velocity


### Own API
This component, is related both to the world and the map. To simplify this, we 
have a grid object with the following functions:

* `grid.getMap()` - returns the image of the map that is being displayed. 
The image returned will be a 3-channel image with values 0 or 255, 
where 0 represents the obstacles and 255 the road. Although the image has 3 
channels, for this practice it will be useful to use only one.
* `grid.getDestiny()` - returns the selected destination through the GUI as 
a tuple (x, y).
* `grid.getPose()` - returns the position with respect to the map, not with 
respect to the world, also as a tuple (x, y).
* `gird.showGrid()` - creates a window in which represents the values ​​of the 
field that have been assigned to the grid. The smaller values ​​will have a color 
closer to black, and will become clearer as larger values ​​are involved. For the 
representation, a copy of the grid is made and its values ​​are normalized so that 
they are between 0 and 1, and it is represented later with cv2.imshow().

The Grid class also provides a grid, on which to point the distance to the 
destination. The values ​​of this grid are float type. The size and positions of 
the grid match that of the image. To interact with it:
* `grid.getVal(x, y)` - returns the value in that grid position.
* `grid.setVal(x, y, val)` - sets the value val to the indicated position.

This class also offers another grid on which the path should be pointed once found. 
Points with value 0 will be ignored, higher values ​​will be considered path. 
The functions to interact are:
* `grid.setPathVal(x, y, val)` - sets the value val to the indicated position.
* `grid.getPathVal(x, y)` - returns the value of the indicated position.
* `grid.setPathFinded()` - establishes that the path has been found to start painting.

Finally, the grid object also offers functions to move from world coordinates to 
map coordinates and vice versa:
* `gridToWorld(gridX, gridY)` - receives the x and y components of the coordinates 
of the map and returns a tuple with the equivalent coordinates in the world: (worldX, worldY)
* `worldToGrid(worldX, worldY)` - receives the x and y components of the world 
coordinates and returns a tuple with the equivalent coordinates in the map: (gridX, gridY)
    
**REMEMBER**: Once running, double click in any point of the map to set destination, then click on "Generate Path" and finally click "GO" to see the result.

### Example video for web template

{% include youtubePlayer.html id=page.youtubeId5 %}

## Theory

Motion Planning is a term used in robotics to find a sequence of valid configurations that moves the robot from source to destination. Motion Planning algorithms find themselves in a variety of settings, be it industrial manipulators, mobile robots, artificial intelligence, animations or study of biological molecules.

There are mainly 2 methods to solve the exercise, **Gradient Path Planning**, **Sampling Based Path Planning**

### Gradient Path Planning
One such method for Motion Planning is Gradient Path Planning. GPP works on the principle of potential fields. The obstacles in the path serve as potential wall to the path planner, and the target serve as potential well. By combining all the potential walls and wells, a path is constructed as a downward slope. The robot follows that path to reach it's destination.

{% include gallery id="potential" caption="Various illustrations based on potential" %}

Gradient Path Planning can be implemented using Brushfire Algorithm or Wave Front Algorithm. Next section explains the working of Wave Front Algorithm.

#### Wave Front Algorithm

Wave Front Algorithm is BFS based approach to build a path from source to destination. The algorithm works by assigning weights to a grid of cells. Given the source and target, the algorithm starts from the target node and moves outwards like a ripple, while progressively assigning weights to the neighboring cells.

![Assigning Weights]({{ site.url }}/RoboticsAcademy/assets/images/exercises/global_navigation/weights.png)

*Assigning Weights*

As for obstacles, additional weights are added to the cells that are close to obstacles. Intuitively, the weights represent the superposition of waves that are reflected from the walls of obstacles.

![Superposition of Waves]({{ site.url }}/RoboticsAcademy/assets/images/exercises/global_navigation/superposition.gif)

*Superposition of Waves*

The algorithm stops upon reaching the source. To navigate through the generated path, the robot follows the path indicated by decreasing weights (a downhill drive). A grayscale image representation quite clearly depicts the path the robot might follow!

![GrayScale Representation]({{ site.url }}/RoboticsAcademy/assets/images/exercises/global_navigation/grayscale.png)

*Gray Scale Representation*

### Sampling Based Path Planning

Sampling based Path Planning employs sampling of the state space of the robot in order to quickly and effectively plan paths, even with differential constraints or those with many degrees of freedom. Some of the algorithms under this class are:

#### Probabilistic Roadmap

These methods work by randomly sampling points in the **workspace**. Once the desired number of samples are obtained, the roadmap is constructed by connecting the random samples to form edges. On the resulting graph formed, any shortest path algorithm (A*, Dijkstra, BFS) is applied to get our resulting path.

{% include gallery id="Probability_Roadmap" caption="Probabilistic Roadmap" %}

#### Tree Based Planner

Tree Based Planners are very similar to Probabilistic Roadmaps, except for the fact that there are no cycles involved in tree based planners. There are a variety of tree based planners, like RRT, EST, SBL and KPIECE. These algorithms work heuristically, working from the root node, a tree (a graph without cycles) is constructed.

{% include gallery id="TreeBasedPlanner" caption="Tree Based Planner" %}

## Hints

Simple hints provided to help you solve the global navigation exercise. The hints are focused on solving the exercise through GPP algorithm.

### Path Planning

Since, the map has already been divided into grids, we can directly work on the grid! Path Planning involves a BFS based search starting from the destination. The psuedo code for the algorithm is:

- **Step1**:	insert Target Node into priority queue
- **Step2**:	c = pop node from priority queue
- **Step3**:	if c == start node	End
- **Step4**:	if c == obstacle	Save to another list and goto Step2
- **Step5**:	assign weight to neighbors of c if **previously unassigned**
- **Step6**:	insert neighbors of c to priority queue
- **Step7**:	goto Step2

Assignment of weights to the cells is arbitrary. Generally, diagonally neighboring cells are assigned a greater weight than the other neighbors. Apart from all the assignment, the additional weight of cells close to obstacles is extremely important, as it is the one that is going to help avoid collisions!

**Important Points to Remember**

- You may use `self.grid.getMap()` to know whether an obstacle is present at (i, j) coordinate of the map. Also, in order to work with this grid, we have to invert our usage of coordinates. Implying, (i, j) can be accessed using (j, i).

- In order to assign those extra weights, we may take the obstacle points we saved earlier, and add extra values to the neighbors of the obstacle cell afterwards.

### Path Navigation

The next step involves Path Navigation. We can start by defining the low level implementation like linear speed and angular speed. Using the orientation of the robot, distance and direction of the target, linear speed and angular speeds can be assigned to the robot.

For navigating effectively, we assign local goal points which eventually lead to the final destination. These local goal points can be selected by choosing 2 points, which occur one after the another **recursively**.

Also, make sure that the selection is done from a big enough radius, otherwise during navigation the robot may pass the grid and reach a **local minima**.

{% include gallery id="gifs" caption="Choose a large radius to avoid missing on turns (left) - Select the next point conditioned on the first point we select, and then plan accordingly (right)" %}

All in all, the exercise is a little on the tough side. But spending time with this exercise, all the bugs, issues and hints provided may make sense!

## Demonstrative Video

Global Navigation with GPP:

{% include youtubePlayer.html id=page.youtubeId1 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId2 %}

<br/>

Global Navigation teletaxi with OMPL:

{% include youtubePlayer.html id=page.youtubeId3 %}

<br/>

{% include youtubePlayer.html id=page.youtubeId4 %}

## Contributors

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Francisco Rivas](https://github.com/chanfr), [Francisco Pérez](https://github.com/fqez), [Jose María Cañas](https://github.com/jmplaza), [Nacho Arranz](https://github.com/igarag), [Nay Oo Lwin](https://github.com/NayOoLwin5).
- Maintained by [Sakshay Mahna](https://github.com/SakshayMahna).

## References
[1](https://en.wikipedia.org/wiki/Motion_planning)
[2](http://ompl.kavrakilab.org/OMPL_Primer.pdf)
[3](https://gsyc.urjc.es/jmplaza/students/tfg-Robotics_Academy-vanessa_fernandez-2017.pdf)
[4](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

