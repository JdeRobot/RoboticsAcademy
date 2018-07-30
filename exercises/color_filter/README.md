# COLOR FILTER EXCERCISE
		
In this practice the intention is to develop a color filter that allow us to segment some object in the image. You will have to get in contact with RGB and HSV color spaces, and `OpenCV` (`Python`) library.

For the realization of the practice, you are provided of a framework written in python that collects the images and allows its visualization. These images will be collected through several specific videos for this practice (pelota_roja.avi and pelotas_roja_azul.avi), although you could use your own videos instead or even a camera.


## EXECUTION

First of all, ensure you have put the correct path in the configuraton file (`cameraserver_conf.cfg`) to the video over you want to apply the filter, and comment the line that access your local camera. This is:

```
[EXAMPLE]
. . .

CameraSrv.Camera.0.Uri = /home/username/Desktop/pelota_roja.avi
# CameraSrv.Camera.0.Uri=0

. . .

```

Once done it, in a terminal launch cameraserver component:
```
$ cameraserver cameraserver_conf.cfg
```

In other terminal launch the color_filter component:
```
$ python2 ./color_filter.py color_filter_conf.yml
```

If you want to find the optimum values for your filter (in order to segment a concrete object) you can launch in other terminal the colorTuner component as follows (remember to run `cameraserver` tool as shown in line 20):

```
$ colorTuner color_tuner_conf.ym
```

-------

## API
* `camera.getImage()` - to get the image received from server
* `camera.setColorImage(input_image)` - to set color image
* `camera.setThresholdImage(bk_image)` - to set Threshold image
