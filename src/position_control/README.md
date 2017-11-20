		POSITION CONTROL EXCERSISE
		==========================

In this practice we will learn the use of PID controllers to implement a local navigation algorithm in the quadricopters.
For this practice a world has been designed for the Gazebo simulator. This world has a 3D model of the AR.Drone and 5 beacons arranged in cross mode. The intention is to make the drone do the following route: the first beacon to visit is the one to the left of the drone, the next will be the one in front, then you will have to go to the one that was located to the left of the initial position, then the one in the back, and finally to the one in front, in the most distant position. Finally we will make it return to the initial position and land.

///////////////////////////////////////////////////////////////////
 			E X E C U T I O N 
///////////////////////////////////////////////////////////////////

In a terminal launch the gazebo simulator:
$ gazebo ardrone-beacons.world

In other terminal launch the position_control component with your algorithm:
$ python2 ./position_control.py position_control_conf.yml


///////////////////////////////////////////////////////////////////
