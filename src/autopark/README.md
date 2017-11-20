
# Autopark

# Autopark Practice

The goal of this practice is to implement the logic of a navigation algorithm for an automated vehicle. The vehicle must find a parking space and park properly.

## How to run
To launch the example, follow the steps below:
* Execution without seeing the world: `gzserver autopark.world`
* Execution watching the world: `gazebo autopark.world`
* Running the practice and the user interface: `python2 autopark.py autopark.cfg`
* Execution of the automatic evaluator: `python2 referee.py autopark.cfg`

To simplify the closure of the environment, just close the Autopark window (s). Ctrl + C will give problems.


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
* pose3d.getX() - to obtain the position of the robot
* pose3d.getY() - to obtain the position of the robot
* pose3d.getYaw() - to obtain the position of the robot
* laser.getLaserData() - It allows obtaining the data of the laser sensor, which consists of 180 pairs of values (0-180º, distance in millimeters).
* motors.sendW() - to set the angular velocity
* motors.sendV() - to set the linear velocity

For this example, you have to get taxi to park properly in the free parking space. The application of the referee will measure different parameters (time it takes the taxi to park, number of crashes with other cars, distance to vehicles), and based on these, will perform the qualification of the solution algorithm.
## Conversion de tipos
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

## Demonstrative video (in spanish)
https://www.youtube.com/watch?v=2SYEb3DyWEE

