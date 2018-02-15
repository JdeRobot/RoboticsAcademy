# Vacuum Cleaner

# Vacuum Cleaner practice

The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.


## How to run
To launch the example, follow the steps below:
* Execution without watching the world: gzserver Vacuum.world
* Execution watching the world: gazebo Vacuum.world
* Execution of the practice and the user interface: python2 vacuumCleaner.py vacuumCleaner_conf.yml
* Execution of the automatic evaluator: python2 referee.py referee.yml

To simplify the closure of the environment, simply close the VacuumCleaner window (s). Ctrl + C will give problems.

## How to do the practice
To carry out the practice, you must edit the MyAlgorithm.py file and insert the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L74)
```
    def execute(self):

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.sendV(10)
        #self.motors.sendW(5)
        # TODO

```


### API
* self.pose3d.getPose3d().yaw - to get the orientation of the robot
* bumper.getBumperData().state - to establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* laser.getLaserData() - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
* motors.sendV() - to set the linear speed
* motors.sendW() - to set the angular velocity

For this example, it is necessary to ensure that the vacuum cleaner covers the highest possible percentage of the house. The application of the automatic evaluator (referee) will measure the percentage traveled, and based on this percentage, will perform the qualification of the solution algorithm.

## Types conversion
### Laser
```
    laser_data = self.laser.getLaserData()

    def parse_laser_data(laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser += [(dist, angle)]
         return laser
```

```
        laser_vectorized = []
        for d,a in laser:
            # (4.2.1) laser into GUI reference system
            x = d * math.cos(a) * -1
            y = d * math.sin(a) * -1
            v = (x,y)
            laser_vectorized += [v]

        laser_mean = np.mean(laser_vectorized, axis=0)
```

## Demonstrative video
https://www.youtube.com/watch?v=ThTXrqTDJ_A
