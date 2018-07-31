# AUTOLOC LASER PRACTICE                      

The goal of this practice is to implement the logic that allows a robot to self
locate in space. In particular, it is about getting the robot to carry out an 
algorithm of self-location, using only a laser sensor, to estimate its position 
in a given space (of which a map has been provided).

## EXECUTION

To launch the example, follow the steps below:

1. Launch Gazebo simulator with the world associated with this practice:

`$ roslaunch laser_loc.launch`

2. Once done it, run the practice and the user interface as follows:

`$ python2 laser_loc.py laser_loc_conf.yml`



## How to do the practice

To carry out the practice, you must edit the `MyAlgorithm.py` file and insert the 
control logic into it.

[MyAlgorithm.py](MyAlgorithm.py#L84)

```
    def execute(self):

        # TODO
        # Add your code here
```

If you wanted, you could use `__init__` method (MyAlgorithm.py#L21) to initiallize 
your own variables.


## API

Use:

* `self.sensors.motors.sendVelocities(vel)` - to send velocities where vel is:
    * use: `vel = CMDVel()` to buil a vel object
    * `vel.vx` - to set linear velocity
    * `vel.az` - to set angular velocity
* `self.sensors.laserdata or self.gui.getLaserData()` - To get Laser Data
* `self.parse_laser_data(laser)` - To parse de data captured by the laser sensor
* `self.pose3d.getPose3d().x`, `self.pose3d.getPose3d().y`, `self.pose3d.getPose3d().yaw` - Get position and orientation of the robot.


### Own API

This practice has some methods and variables that will be of great help to solve the problem:

* ||Class Particle|| -> `Particle(x, y, yaw, prob, self.map.robotAngle)`:
    Constructor class for single particles. USE IT AS SHOWN (Do not modify last argument).
* `self.setParticles([p1,p2,p3,...])` - To set particles (paint them on GUI).
* `self.setEstimation(particle)` - To set the estimated position of the robot (paint it on GUI).
* `img = self.map.pixmap.toImage()` - To get the image of the GUI.
* `self.paintTheoricalLaser(theoricalLaser)` - To paint the theorical laser on the GUI.
*  (variable) `self.particleClicked` - stores a particle if it has been clicked on the GUI.
* `self.map.map2pixel((x,y))` - Obtain the px corresponding to the coordinates x,y.
* `self.map.pixel2map((px,py))` - Obtain the correspnding coordinates to the pixel px,py.


## Demonstrative Video
[Video](https://www.youtube.com/watch?v=FmUN_tzM9MM)

## Attribution

* *Copyright (C) 2016 CC-BY-4.0 Carlos Awadallah (@cawadall)*

*Authors:*
* *Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>*
* *Carlos Awadallah Estévez <carlosawadallah@gmail.com>*

