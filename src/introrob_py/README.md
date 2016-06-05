Introrob_py
==============

For simulation
--------------
In a terminal launch the gazebo simulator:
gazebo ArDrone.world

In other terminal lauch the introrob component:
./introrob.py --Ice.Config=introrob_simulated_conf.cfg

For real Ar.Drone
--------------
In a terminal lauch ardrone_server:
ardrone_server --Ice.Config=ardrone_interfaces.cfg

In other terminal launch introrob component:
./introrob.py --Ice.Config=introrob_conf.cfg
