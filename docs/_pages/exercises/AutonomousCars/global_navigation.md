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

---

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


## Installation 

To launch the infrastructure of this practice, first set up the gazebo sources, then launch the simulator with the appropriate scenario:

```bash
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

or add them directly to your bashrc to run automatically whenever you open a terminal:

```bash
echo 'source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh' >> ~/.bashrc
```

```bash
echo 'source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh' >> ~/.bashrc
```

```bash
source ~/.bashrc
```

## How to do the practice
To carry out the practice, you must edit the file MyAlgorithm.py and insert all the functionality in it.

### Where to insert the code
There are two parts in which the code should be inserted:

- The part related to finding the shortest path must be located inside the 
generatePath function, which is executed only when the button is pressed in 
the GUI MyAlgorithm.py

```python
def generatePath(self):
	print "LOOKING FOR SHORTER PATH"
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        # TODO
```

- The code that will allow the robot to reach the destination will be placed in 
the execute function, which is executed periodically. 
MyAlgorithm.py

```python
def execute(self):
        # Add your code here
        print "GOING TO DESTINATION"
```

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

## How to run your solution

Navigate to the global_navigation directory

```bash
cd exercises/global_navigation
```

Launch Gazebo with the tele_taxi world through the command 

```bash
roslaunch ./launch/tele_taxi.launch
```

Execute the practice's component indicating the configuration file for the map:
```bash
python2 globalNavigation.py taxiMap.conf
```
    
**REMEMBER**: Once running, double click in any point of the map to set destination, then click on "Generate Path" and finally click "GO" to see the result.

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

## Contributers

- Contributors: [Alberto Martín](https://github.com/almartinflorido), [Francisco Rivas](https://github.com/chanfr), [Francisco Pérez](https://github.com/fqez), [Jose María Cañas](https://github.com/jmplaza), [Nacho Arranz](https://github.com/igarag).
- Maintaied by [Sakshay Mahna](https://github.com/SakshayMahna).

## References
[1](https://en.wikipedia.org/wiki/Motion_planning)
[2](http://ompl.kavrakilab.org/OMPL_Primer.pdf)
[3](https://gsyc.urjc.es/jmplaza/students/tfg-Robotics_Academy-vanessa_fernandez-2017.pdf)
[4](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)

