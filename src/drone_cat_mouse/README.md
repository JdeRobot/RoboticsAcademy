# Cat and mouse

# Drone_cat_mouse prctice

The objective of this practice is to program an autonomous behavior for a drone that simulates the game of the cat and the mouse

## How to execute
To launch the example, follow the steps below:

* Execution without watching the world: `gzserver ardrone-trees-simple.world`
* Execution watching the world: `gazebo ardrone-trees-simple.world`
* Cat execution: `python2 cat.py cat_conf.cfg`
* Mouse execution: `./qX_mouse qX.cfg` 
	Where X wiill be the execute mouse (q1, q2, etc)
* Referee execution: `python2 referee.py referee.cfg`

To simplify the closure of the environment, just close the (s)
window (s) of bump_and_go. * Ctrl + C will give problems *.

## How to do the practice
To carry out the practice, you must edit the file MyAlgorithms.py and insert the control logic.

### Where to inster the code
[MyAlgorithm.py](MyAlgorithm.py#L58)
```
    def execute(self):
        # Add your code here
        tmp = self.navdata.getNavdata()
        if tmp is not None:
            print ("State: " +str(tmp.state))
            print ("Altitude: " +str(tmp.altd))
            print ("Vehicle: " +str(tmp.vehicle))
            print ("Battery %: " +str(tmp.batteryPercent))
```

### API
* cmdvel.sendCMDVel(self,vx,vy,vz,ax,ay,az): sends linear and angular speed commands to the drone.
* pose.getX(), pose.getY(), pose.getZ(): returns the position values ​​of the drone in space.
* pose.getRoll(), pose.getPitch(), pose.getYaw(): returns the rotation values ​​of the drone in space.
* camera.getImage(): returns the image captured by the active camera of the drone (frontal or ventral).
* extra.toggleCam(): changes the active camera of the drone (ventral or frontal).
* extra.takeOff(): the drone takes off.
* extra.land(): The drone lands.

For this example, it is necessary to achieve that the black drone (cat) programmed by the student, follow the red drone (mouse) that is already 
preprogrammed and has a random trajectory as close as possible without crashing. The referee application (referee in English) will measure the 
distance between both drones and will assign a score. The more time spent near the mouse in time, the more score you will get.


## Demonstration video of the 2016 championship
https://youtu.be/Hd2nhOx1tqI?t=8m30s
