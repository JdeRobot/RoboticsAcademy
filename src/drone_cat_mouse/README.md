# CAT AND MOUSE EXCERCISE

The objective of this practice is to program an autonomous behavior
for a drone that simulates the game of the cat and the mouse.

This is to make the black drone (cat), programmed by the student,
follow the red drone (mouse, that is already preprogrammed and has a random path)
as close as possible without crashing. The referee application will measure
the distance between the two drones and assign a score based on it. The longer
time you spend close to the mouse, more score will be obtained.

## E X E C U T I O N 
Follow these steps:

1. First of all, launch Gazebo simulator:
`$ gazebo ardrone-trees-simple.world`
    
* In case your CPU does not support the load of the simulator, 
try the execution without seeing the world:
`$ gzserver ardrone-trees-simple.world`

2. Once done it, run the cat component:
`python2 ./cat.py cat_conf.yml`
and then, the mouse component:
`$ ./qX_mouse qX.cfg` 
* X will be the mouse we want to use (q1, q2, etc)

3. Run the referee: 
`$ python2 ./referee.py referee.yml`

To simplify the closure of the environment, just close the
window(s). * Ctrl + C will be problematic *.

NOTE: if you want to teleoperate the mouse, run:

```
cd ~/Academy/src/drone_cat_mouse/mice/
uav_viewer_py mouse_cfg.yml
```

## HOW TO DO THE PRACTICE
To carry out the practice, you must edit the file MyAlgorithms.py and
insert the control logic.

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
* cmdvel.sendCMDVel(vx,vy,vz,ax,ay,az): sends linear and angular speed commands to the drone.
* pose.getPose3d(): returns the position and rotation values of the drone in space. Content: x, y, z, h, yaw, pitch, roll, q (quaternion)
* pose.getRoll(), pose.getPitch(), pose.getYaw(): returns the rotation values of the drone in space.
* extra.toggleCam(): changes the drone's active camera (frontal or the one below).
* extra.takeoff(): Takeoff of the drone.
* extra.land(): landing of the drone.
* camera.getImage() - to get the image received from server. Content: height, width, format, data (opencv Image)

Image to be processed is in *camera.getImage().data*



## Demonstration video of the 2016 championship
https://youtu.be/Hd2nhOx1tqI?t=8m30s
