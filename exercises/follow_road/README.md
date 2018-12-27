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
`$ roslaunch /opt/jderobot/share/jderobot/gazebo/launch/follow_road.launch`

2. In other terminal lauch the follow_road component:
`$ python2 follow_road.py`


////////////////////////////////////////////////////////////////////////////////

## How to do the practice
To carry out the practice, you must edit the MyAlgorithm.py file and insert
the control logic into it.

### Where to insert the code
[MyAlgorithm.py](MyAlgorithm.py#L62)
```
         def execute(self):
           # Add your code here

           input_imageV = self.drone.getImageVentral().data
               input_imageF = self.drone.getImageFrontal().data

               if input_imageV is not None:
                '''
                If you want show a thresold image
                self.setImageFilteredVentral(input_imageV)
                '''

```

### API
* self.setImageFilteredVentral(image_HSV_filtered_Mask_V): : If you want show a filtered image of the ventral camera images.
* self.setImageFilteredFrontal(image_HSV_filtered_Mask_F): If you want show a filtered image of the frontals camera images.

## Demonstrative video (in spanish)
https://youtu.be/rIkTImMyoXw
