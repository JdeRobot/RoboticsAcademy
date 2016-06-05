Follow turtlebot excercise
==============

In a terminal launch the gazebo simulator:
gazebo ardrone-turtlebot.world

In other terminal launch the turtlebot robot:
kobukiViewer --Ice.Config=turtlebot.cfg

In another terminal lauch the follow_turtlebot component:
./follow_turtlebot.py --Ice.Config=follow_turtlebot_conf.cfg

If you want to find the values of your color filter you can launch the colorTuner component:
colorTuner --Ice.Config=color_tuner_conf.cfg
