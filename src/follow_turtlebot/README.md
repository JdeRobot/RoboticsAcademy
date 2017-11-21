Follow turtlebot excercise
==============

In a terminal launch the gazebo simulator:
gazebo ardrone-turtlebot.world

In other terminal launch the turtlebot robot:
kobukiViewer turtlebot.yml

In another terminal lauch the follow_turtlebot component:
python2 ./follow_turtlebot.py follow_turtlebot_conf.yml

If you want to find the values of your color filter you can launch the colorTuner component:
colorTuner color_tuner_conf.yml
