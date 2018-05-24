                       LABYRINTH ESCAPE EXCERSISE
                       ==========================

The objective of this practice is to implement the logic that allows a drone to 
escape from a labyrinth through visual signals placed on the ground.

////////////////////////////////////////////////////////////////////////////////
                           E X E C U T I O N 
////////////////////////////////////////////////////////////////////////////////

Follow these simple steps to launch this practice:

1. In a terminal launch gazebo:
`$ gazebo ArDrone_labyrinth.world`

2. Use another terminal to launch the labyrinth escape component:
`$ python2 ./labyrinth_escape.py labyrinth_escape_conf.yml`

////////////////////////////////////////////////////////////////////////////////

## How to do the practice
To carry out the practice, you must edit the MyAlgorithm.py file and insert 
the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L67)
```
         def execute(self):
             # Add your code here
             tmp = self.navdata.getNavData()
             if tmp is not None:
                 print ("State: " +str(tmp.state))
                 print ("Altitude: " +str(tmp.altd))
                 print ("Vehicle: " +str(tmp.vehicle))
                 print ("Battery %: " +str(tmp.batteryPercent))
        
```

### API
* self.camera.setThresholdImage(): If you want show a black and white image.
* self.camera.getImage().data: returns the image captured by the active camera of the drone (frontal or ventral).
* self.cmdvel.setVX(velx): set linear speed of the drone.
* self.cmdvel.setVY(vely): set linear speed of the drone.
* self.cmdvel.sendVelocities(): send set velocities to the drone.
* self.cmdvel.sendCMDVel(self,vx,vy,vz,ax,ay,az): sends linear and angular speed commands to the drone.
* self.pose.getPose3d().x, self.pose.getPose3d().y, self.pose.getPose3d().z: returns the position values ​​of the drone in space.
* self.pose.getPose3d().roll, self.pose.getPose3d().pitch, self.pose.getPose3d().yaw: returns the rotation values ​​of the drone in space.
* self.extra.toggleCam(): changes the drone's active camera (frontal or the one below).
* self.extra.takeOff(): Takeoff of the drone.
* self.extra.land(): landing of the drone.

## Demonstrative video
http://jderobot.org/store/jmplaza/uploads/teaching/curso-drones/labyrinth_escape.ogv



