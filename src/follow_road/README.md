Follow road excercise
==============

In a terminal launch the gazebo simulator:
gazebo road_drone_textures.world

In other terminal lauch the follow_road component:
./follow_road.py --Ice.Config=follow_road_conf.cfg

If you want to find the values of your color filter you can launch the colorTuner component:
colorTuner --Ice.Config=color_tuner_conf.cfg
