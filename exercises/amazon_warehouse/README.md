# AMAZON WAREHOUSE EXERCISE

The goal of this practice is to implement the logic of a pick, navigate and place robot. The robot must generate shortest path to pallet, navigate to it, pick and deliver it to the delivery zone autonomously.

## How to run
To launch the example, follow the steps below:

0. Source the gazebo setups:

```
$ source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
$ source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

1. Run Gazebo simulator:

```
$ roslaunch /opt/jderobot/share/jderobot/gazebo/launch/amazonrobot_1_warehouse.launch
```

2. Navigate to exercise folder and run the practice and the user interface: 

```
$ python2 amazonWarehouse.py amazonMap.conf amazonConf.yml

```

To simplify the closure of the environment, just close the Amazon window (s). *`Ctrl + C` will give problems.*


## How to do the practice
To carry out the practice, you must edit the `MyAlgorithm.py` file and insert the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L89)
```
    def execute(self):

        # Add your code here
        print "Running"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        self.vel.setV(0.2)
        self.vel.setW(0.1)

        # self.sensor.getRobotTheta()
	# self.sensor.getRobotY()
        
        # TODO
```

### API
* `self.sensor.getRobotX()` - to obtain the position of the robot
* `self.sensor.getRobotY()` - to obtain the position of the robot
* `self.sensor.getRobotTheta()` - to obtain the position of the robot
* `self.laser.getLaserData()` - It allows obtaining the data of the laser sensor, which consists of 180 pairs of values (0-180º, distance in millimeters).
* `self.vel.setW()` - to set the angular velocity
* `self.vel.setV()` - to set the linear velocity

For this example, you have to get the robot autonomously navigate in warehouse environment.


## Conversion of types.

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

## Demonstrative video on initialization and running
[Video](https://www.youtube.com/watch?v=6wjy2oEKDD0)

