                            FOLLOW ROAD EXCERCISE
                            =====================

The goal of this practice is to implement the logic that allows a quadricopter
to follow a road. In order to do this, you will have to establish a color filter
to segment road lines, and then develop an algorithm to follow them until the 
end of the road.

////////////////////////////////////////////////////////////////////////////////
                           E X E C U T I O N 
////////////////////////////////////////////////////////////////////////////////

Follow these simple steps to launch this practice:

1. In a terminal launch the gazebo simulator:
`$ gazebo road_drone_textures.world`

2. In other terminal lauch the follow_road component:
`$ python2 ./follow_road.py follow_road_conf.yml`

* If you want to find the optimum values for your filter (in order to segment road lines) 
you can launch another terminal with the colorTuner component as follows 
(remember to run gazebo's world  as shown in line 11 so the image from the drone's
camera is available):
`$ colorTuner color_tuner_conf.yml`

////////////////////////////////////////////////////////////////////////////////

## How to do the practice
To carry out the practice, you must edit the MyAlgorithm.py file and insert 
the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L62)
```
         def execute(self):
           # Add your code here

            input_image = self.camera.getImage().data
            if input_image is not None:
                '''
                If you want show a thresold image
                self.setImageFiltered(input_image)
                '''
        
```

### API
* self.setImageFiltered(): If you want show a filtered image of the camera images.

## Demonstrative video (in spanish)
https://youtu.be/rIkTImMyoXw
