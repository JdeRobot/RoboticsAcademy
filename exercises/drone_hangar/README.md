# CAT AND MOUSE EXCERCISE

The objective of this practice is to program an autonomous behavior
for a drone that simulates the game of the cat and the mouse.

This is to make the black drone (cat), programmed by the student,
follow the red drone (mouse, that is already preprogrammed and has a random path)
as close as possible without crashing. However, the cat must
escape from an hangar with mobile obstacles without colliding first.
The referee application will measure the distance between the two drones and assign a score based on it. 
The longer time you spend close to the mouse, more score will be obtained.


## Downloading the mouse programs and their configuration

```
cd mice 
wget http://jderobot.org/store/jmplaza/uploads/jderobot-academy/drone-catmouse/mice.tgz 
tar -zxvf mice.tgz
```

## EXECUTION
Follow these steps:

1. First of all, launch Gazebo simulator:

`$ gazebo gymkhana.world`
    
* In case your CPU does not support the load of the simulator, 
try the execution without seeing the world:

`$ gzserver gymkhana-simple.world`

2. Once done it, run the cat component:

`python2 ./cat.py cat_conf.yml`

3. and then, the mouse component, one of the following:

* start teleoperating the mouse to refine your cat:

```
cd ~/Academy/src/drone_hangar/mice/
uav_viewer_py mouse_cfg.yml
```
* if you feel confident enough with your cat, try with an autonomous mouse (they are ordered in increasing difficulty):
`$ ./trainning_mouse trainning.cfg` 
* or `$ ./q1_mouse q1.cfg`
* or `$ ./q2_mouse q2.cfg`


4. Optionally you can run the referee too: 

`$ python2 ./referee.py referee.yml`

To simplify the closure of the environment, just close the
window(s). *Ctrl + C will be problematic*.



## HOW TO DO THE PRACTICE

To carry out the practice, you must edit the file `MyAlgorithms.py` and
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
* `cmdvel.sendCMDVel(vx,vy,vz,ax,ay,az)`: sends linear and angular speed commands to the drone.
* `pose.getPose3d()`: returns the position and rotation values of the drone in space. Content: x, y, z, h, yaw, pitch, roll, q (quaternion)
* `pose.getRoll()`, `pose.getPitch()`, `pose.getYaw()`: returns the rotation values of the drone in space.
* `extra.toggleCam()`: changes the drone's active camera (frontal or the one below).
* `extra.takeoff()`: Takeoff of the drone.
* `extra.land()`: landing of the drone.
* `camera.getImage()` - to get the image received from server. Content: height, width, format, data (opencv Image)
* `camera.setFilteredImage(filt_image)` - to set filtered image (Opencv Image)

Image to be processed is in *camera.getImage().data*



## Demonstration video of the 2016 championship
[Video](https://youtu.be/Hd2nhOx1tqI?t=8m30s)
