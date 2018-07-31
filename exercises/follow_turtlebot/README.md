# Follow turtlebot excercise

The exercise consists of following a robot on the ground using a drone. The movements described by the ground robot will have to be followed by the air robot.

## How to execute?
In a terminal launch the gazebo simulator:
`gazebo ardrone-turtlebot.world`

In other terminal launch the turtlebot robot:
`kobukiViewer turtlebot.yml`

In another terminal lauch the follow_turtlebot component:
`python2 ./follow_turtlebot.py follow_turtlebot_conf.yml`

If you want to find the values of your color filter you can launch the colorTuner component:
`colorTuner color_tuner_conf.yml`

## How to do the practice?
To carry out the practice, you have to edit the file MyAlgorithms.py and insert in it your code, which gives intelligence to the turtlebot robot.

## Where to insert the code?
[MyAlgorithm.py](MyAlgorithm.py#L62)
```
    # Add your code here

    input_image = self.camera.getImage()
    if input_image is not None:
        self.camera.setColorImage(input_image)
        '''
        If you want show a thresold image (black and white image)
        self.camera.setThresoldImage(input_image)
        '''
```

## API
* `cameraL.getImage()` - to get the left image of the stereo pair.
* `self.camera.setThresholdImage()`: If you want show a black and white image.
* `self.cmdvel.sendCMDVel(self,vx,vy,vz,ax,ay,az)`: sends linear and angular speed commands to the drone.


## Demonstrative video
[Video](https://www.youtube.com/watch?v=uehDVlBzpmU)

* *Base code made by Alberto Martín (@almartinflorido)*
* *Code of practice performed by Francisco Rivas (@chanfr)*
* *Gazebo models and worlds made by Francisco Pérez (@fqez)*
* *Updated code made by Pablo Moreno (@PabloMorenoVera)*
