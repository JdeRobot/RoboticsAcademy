Color filter excercise
==============

In a terminal launch cameraserver component:
cameraserver --Ice.Config=cameraserver_conf.cfg

In other terminal launch the color_filter component:
./color_filter.py --Ice.Config=color_filter_conf.cfg

If you want to find the values for your filter you can launch in other terminal the colorTuner component:
colorTuner --Ice.Config=color_tuner_conf.cfg
