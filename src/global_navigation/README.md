 # Global Navigation
* [Go to English instructions (not planned)](#english)
* [Ir a las instrucciones en Español](#spanish)

<a name="spanish"/>
# Global Navigation Practice

The objective of this practice is to implement the logic of a Gradient Path Planning (GPP) algorithm.

Global navigation through GPP, consists of:

- Selected a destination, the GPP algorithm is responsible for finding the shortest path to it, avoiding, in the case of this practice, everything that is not road.
- Once the path has been selected, the logic necessary to follow this path and reach the objective must be implemented in the robot.

With this, it is possible for the robot to go to the marked destination autonomously and on the shortest path.

The solution can integrate one or more of the following levels
of difficulty, as well as any other one that occurs to you:
* Reach the goal.
* Optimize the way to find the shortest path.
* Arrive as quickly as possible to the destination.

## How to run
A script has been prepared to launch the practice. To board
little powerful machines, does not launch the gazebo interface by default.
* Execution without seeing the world: `./run_it.sh`
* Execution watching the world: `./run_it.sh GUI`

If you have a very slow machine, or use the complete circuit, you should
increase gazebo waiting time on [line 11] (run_it.sh # L11)

In case you want to run it without a script, you should:
- Launch Gazebo with or without GUI:
    gazebo cityLarge.world #With GUI
    gzserver cityLarge.world #Sin GUI

- Execute the practice indicating what is the configuration file for the map:
    python main.py --mapConfig = taxiMap.conf --Ice.Config = teleTaxi.cfg

In addition, the practice can be launched without launching Gazebo. For this, you simply have to run it without the argument --Ice.Config = teleTaxi.cfg, although then you can not get information from the sensors of the vehicle, but you can work with the image to calculate the field without having Gazebo open.

## How to do the practice
To carry out the practice, you must edit the MyAlgorithms.py file and insert all the functionality in it.

### Where to insert the code
There are two parts in which the code should be inserted:

- The part related to finding the shortest path must be located inside the generatePath function, which is executed only when the button is pressed in the GUI [MyAlgorithm.py] (MyAlgorithm.py # L17)

```
    def generatePath (self):
        print "LOOKING FOR SHORTER PATH"
        mapIm = self.grid.getMap ()
        dest = self.grid.getDestiny ()
        gridPos = self.grid.getPose ()

        # ALL
```

- The relatic code to arrive at the destination, goes in the execute function, which is executed periodically. [MyAlgorithm.py] (MyAlgorithm.py # L29)

```
    def execute (self):
        # Add your code here
        print "GOING TO DESTINATION"
```

### API
* sensor.getRobotX () - to obtain the position of the robot
* sensor.getRobotY () - to obtain the position of the robot
* sensor.getRobotTheta () - to obtain the orientation of the robot with respect to the map
* motors.setV () - to set the linear speed
* motors.setW () - to set the angular velocity

### Own API
This component, in addition to relating to the world, has to be related to the map. To simplify this, we have a grid object with the following functions:
* grid.getMap () - returns the image of the map that is being displayed. The image returned will be a 3-channel image with values ​​0 and 255, where 0 represents the obstacles and 255 the road. Although the image has 3 channels, for practice it will be useful to use only one.
* grid.getDestiny () - returns the selected destination through the GUI as a tuple (x, y).
* grid.getPose () - returns the position with respect to the map, not with respect to the world, also as a tuple (x, y).
* gird.showGrid () - creates a window in which represents the values ​​of the field that have been assigned to the grid. The smaller values ​​will have a color closer to black, and will become clearer as larger values ​​are involved. For the representation a copy of the grid is made and its values ​​are normalized so that they are between 0 and 1, and it is represented later with cv2.imshow ().

The Grid class also provides a grid, on which to point the distance to the destination in it. The values ​​of this grid are of the float type. The size and positions of the grid match that of the image. To interact with her:
* grid.getVal (x, y) - returns the value in that grid position.
* grid.setVal (x, y, val) - sets the value val to the indicated position.

This class also offers another grid on which the path should be pointed once found. Points with value 0 will be ignored, higher values ​​will be considered path. The functions to interact are:
* grid.setPathVal (x, y, val) - sets the value val to the indicated position.
* grid.getPathVal (x, y) - returns the value of the indicated position.
* grid.setPathFinded () - establishes that the path has been found to start painting.

Finally, the grid object also offers functions to move from world coordinates to map coordinates and vice versa:
* gridToWorld (gridX, gridY) - receives the x and y components of the coordinates of the map and returns a tuple with the equivalent coordinates in the world: (worldX, worldY)
* worldToGrid (worldX, worldY) - receives the x and y components of the world coordinates and returns a tuple with the equivalent coordinates in the map: (gridX, gridY)

## Demonstration video


## Attributions
* * Copyright (C) 2016 CC-BY-4.0 Samuel Rey (@ reysam93)
Original text created by Victor Arribas (@varhub) adapted to cover the needs of this practice. Copyright (C) 2016 CC-BY-4.0 Victor Arribas *

* * Basic code made by Alberto Martín (@almartinflorido) *
* * Code of practice and world made by Samuel Rey *


