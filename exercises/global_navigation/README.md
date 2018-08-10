# NavigationOmpl Exercise

The objective of this practice is to learn the use of ompl to implement a local navigation in the taxi.

You can see this [repo](https://github.com/TheRoboticsClub/colab-gsoc2018-HanqingXie) and [web](https://jderobot.org/Club-hanqingxie) for more detials.

## Dependency
This project requires the follow dependency:
* jedrobot
* python2.7
* [OMPL](http://ompl.kavrakilab.org/)

### install ompl
[Download the OMPL installation script](http://ompl.kavrakilab.org/install-ompl-ubuntu.sh). First, make the script executable:

```chmod u+x install-ompl-ubuntu.sh```

Next, there are three ways to run this script:

```./install-ompl-ubuntu.sh --python will install OMPL with Python bindings```

## How to run
1. Launch Gazebo with (or without) GUI in one terminal:
 * With GUI

    `$ gazebo cityLarge.world` 

 * Without GUI

    `$ gzserver cityLarge.world`

2. Execute the practice's component indicating the configuration file for the map:

    `$ python2 globalNavigation.py taxiMap.conf teleTaxi_conf.yml`

* REMEMBER: Once running, double click in any point of the map to set destination,
  then click on "Generate Path" and finally click "GO" to see the result.

## How to do the practice
To carry out the practice, you must edit the MyAlgorithm.py file and insert the control logic into it.

## Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L70)
```
    def execute(self):

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.sendV(10)
        #self.motors.sendW(5)
        # TODO

```

## API
* sensor.getRobotX () - to obtain the position of the robot
* sensor.getRobotY () - to obtain the position of the robot
* sensor.getRobotTheta () - to obtain the orientation of the robot with respect to the map
* vel.setV () - to set the linear speed
* vel.setW () - to set the angular velocity


## Own API
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
