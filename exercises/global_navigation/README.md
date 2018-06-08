# GLOBAL NAVIGATION PRACTICE

The objective of this practice is to implement the logic of a Gradient Path 
Planning (GPP) algorithm. Global navigation through GPP, consists of:

- Selected a destination, the GPP algorithm is responsible for finding the 
shortest path to it, avoiding, in the case of this practice, everything that is 
not road.
- Once the path has been selected, the logic necessary to follow this path and 
reach the objective must be implemented in the robot.

With this, it is possible for the robot to go to the marked destination 
autonomously and following the shortest path.

The solution can integrate one or more of the following levels
of difficulty, as well as any other one that occurs to you:
* Reach the goal.
* Optimize the way to find the shortest path.
* Arrive as quickly as possible to the destination.

## E X E C U T I O N

Follow these simple steps to run the practice:
1. Launch Gazebo with roslaunch
```
$ roslaunch taxi-holo.launch
```

2. Execute the practice's component indicating the configuration file for the map:
```
$ python2 globalNavigation.py taxiMap.conf
```

3. Additionally, you can execute the referee component to check the efficiency 
and effectiveness of your algorithm (*NOT WORKING YET*):
```
$ python2 ./referee.py referee.yml
```
    
* REMEMBER: Once running, double click in any point of the map to set destination,
  then click on "Generate Path" and finally click "GO" to see the result.

## How to do the practice
To carry out the practice, you must edit the file MyAlgorithm.py and insert all 
the functionality in it.

### Where to insert the code
There are two parts in which the code should be inserted:

- The part related to finding the shortest path must be located inside the 
generatePath function, which is executed only when the button is pressed in 
the GUI [MyAlgorithm.py] (MyAlgorithm.py # L17)
```
        def generatePath(self):
            print "LOOKING FOR SHORTER PATH"
            mapIm = self.grid.getMap()      
            dest = self.grid.getDestiny()   
            gridPos = self.grid.getPose()

            # TODO
```

- The code that will allow the robot to reach the destination will be placed in 
the execute function, which is executed periodically. 
[MyAlgorithm.py] (MyAlgorithm.py # L29)

```
        def execute(self):
            # Add your code here
            print "GOING TO DESTINATION"
```

### API
* sensor.getRobotX () - to obtain the position of the robot
* sensor.getRobotY () - to obtain the position of the robot
* sensor.getRobotTheta () - to obtain the orientation of the robot with respect to the map
* vel.setV () - to set the linear speed
* vel.setW () - to set the angular velocity


### Own API
This component, is related both to the world and the map. To simplify this, we 
have a grid object with the following functions:
* grid.getMap () - returns the image of the map that is being displayed. 
The image returned will be a 3-channel image with values 0 or 255, 
where 0 represents the obstacles and 255 the road. Although the image has 3 
channels, for this practice it will be useful to use only one.
* grid.getDestiny () - returns the selected destination through the GUI as 
a tuple (x, y).
* grid.getPose () - returns the position with respect to the map, not with 
respect to the world, also as a tuple (x, y).
* gird.showGrid () - creates a window in which represents the values ​​of the 
field that have been assigned to the grid. The smaller values ​​will have a color 
closer to black, and will become clearer as larger values ​​are involved. For the 
representation, a copy of the grid is made and its values ​​are normalized so that 
they are between 0 and 1, and it is represented later with cv2.imshow().

The Grid class also provides a grid, on which to point the distance to the 
destination. The values ​​of this grid are float type. The size and positions of 
the grid match that of the image. To interact with it:
* grid.getVal (x, y) - returns the value in that grid position.
* grid.setVal (x, y, val) - sets the value val to the indicated position.

This class also offers another grid on which the path should be pointed once found. 
Points with value 0 will be ignored, higher values ​​will be considered path. 
The functions to interact are:
* grid.setPathVal (x, y, val) - sets the value val to the indicated position.
* grid.getPathVal (x, y) - returns the value of the indicated position.
* grid.setPathFinded () - establishes that the path has been found to start painting.

Finally, the grid object also offers functions to move from world coordinates to 
map coordinates and vice versa:
* gridToWorld (gridX, gridY) - receives the x and y components of the coordinates 
of the map and returns a tuple with the equivalent coordinates in the world: (worldX, worldY)
* worldToGrid (worldX, worldY) - receives the x and y components of the world 
coordinates and returns a tuple with the equivalent coordinates in the map: (gridX, gridY)


## Demonstration video


## Attributions
* * Copyright (C) 2016 CC-BY-4.0 Samuel Rey (@ reysam93)
Original text created by Victor Arribas (@varhub) adapted to cover the needs of 
this practice. Copyright (C) 2016 CC-BY-4.0 Victor Arribas *

* * Basic code made by Alberto Martín (@almartinflorido) *
* * Code of practice and world made by Samuel Rey *
