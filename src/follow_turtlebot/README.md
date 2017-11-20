		FOLLOW TURTLEBOT EXCERSISE
		==========================
In this practice, you will have to create an algorithm that makes the Drone follow a terrestrial robot (turtlebot). It is highly recommended to have previously done the color filter and position control practices, as we start off them to create a good algorithm that achieves the established objectives. 
You will have then to combine a color filter that processes the image of the drone's camera to segment the turtlebot (which has a green rectangle on it). Once done it, the next step will be analyze the image to determine the position of the terrestrial robot with respect to that of our drone, and correct its position with that information, in orther to follow it.

///////////////////////////////////////////////////////////////////
 			E X E C U T I O N 
///////////////////////////////////////////////////////////////////
We will use 3 terminal for the execution. Follow these instructions:

In the first one, launch the gazebo simulator:
$ gazebo ardrone-turtlebot.world

Use another terminal to launch the turtlebot's teleoperator:
$ kobukiViewer kobukiViewer.yml

Finally, lauch the follow_turtlebot component:
$ python2 ./follow_turtlebot.py follow_turtlebot_conf.cfg

If you want to find the optimum values for your filter (in order to segment turtlebot) you can launch in other terminal the colorTuner component as follows:

NOTE: Ensure Gazebo's world is running, so you will be able to see through ARDrone's camera the turtlebot. Once running, press "Take off" button in the GUI and place the Drone using the controller above the turtlebot. Now you are able to see it in colorTuner tool.

$ colorTuner color_tuner_conf.yml 

///////////////////////////////////////////////////////////////////
