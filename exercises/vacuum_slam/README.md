# Vacuum Slam

# Vacuum Slam Practice

The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum with autolocation. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

## How to run
To launch the example, follow the steps below:
1. Execution watching the world: 
`$ roslaunch vacuum_slam.launch`
2. Execution of the practice and the user interface: 
`$ python2 vacuum.py vacuum_conf.yml`
3. Execution of the automatic evaluator: 
`$ python2 referee.py vacuum_conf.yml`

To simplify the closure of the environment, simply close the VacuumCleaner window (s). Ctrl + C will give problems.


## How to do the practice
To carry out the practice, you must edit the MyAlgorithms.py file and insert all the functionality in it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L81)
```
        def execute(self):

            # Add your code here
            print "Runing"

            #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
            #self.motors.sendV(10)
            #self.motors.sendW(5)
```

### API
* self.pose3d.getPose3d().yaw - to get the orientation of the robot
* self.bumper.getBumperData().state - to establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
* self.bumper.getBumperData().bumper - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its left and 2 if the collision is at its right.
* laser.getLaserData() - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
* self.motors.sendVelocities(vel) - to set linear and angular velocity. Object 'vel' must be a CMDVel():
    - vel = CMDVel()
    - vel.vx = v - linear velocity
    - vel.az = w - angular velocity

For this example, it is necessary to ensure that the vacuum cleaner covers the highest possible percentage of the house. The application of the automatic evaluator (referee) will measure the percentage traveled, and based on this percentage, will perform the qualification of the solution algorithm.

## Types conversion
### Laser
```
        laser_data = self.laser.getLaserData()

        def parse_laser_data(laser_data):
            laser = []
            for i in range(180):
                dist = laser_data.values[i]
                angle = math.radians(i)
                laser += [(dist, angle)]
            return laser
```

```
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
