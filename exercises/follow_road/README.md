# Follow Road Exercise

## Goal

The goal of this exercise is to implement the logic that allows a quadcopter to follow a road. In order to do this, you will have to establish a color filter to segment road lines and then develop an algorithm to follow them until the end of the road.

## Requirements

As this is a drones exercise, you will need to additionally install the `jderobot-drones` package. This can be done easily by `sudo apt install jderobot-drones`.

## Execution

To launch the exercise, simply use the following command from this directory:

`roslaunch rqt_follow_road follow_road.launch solution_file_name:=$PWD/my_solution.py`

As an easy way to find the values for the color filtering, you can use the colorTuner tool provided in your jderobot installation. This is used after launching the previous command in a seperate terminal as follows:

`colorTuner colorTuner.conf`

## Solution

To solve the exercise, you must edit the my_solution.py file and insert the control logic into it. Your code is to be entered in the `execute` function between the `Insert your code here` comments.
[my_solution.py](my_solution.py#L49)

```python
def execute(event):
  global drone
  img_frontal = drone.get_frontal_image()
  img_ventral = drone.get_ventral_image()
  # Both the above images are cv2 images
  ################# Insert your code here #################################

  set_image_filtered(img_frontal)
  set_image_threshed(img_ventral)

#########################################################################
```

## API

* set_image_filtered(cv2_image): If you want to show a filtered image of the camera images in the GUI
* set_image_threshed(cv2_image): If you want to show a thresholded image in the GUI
* drone.get_frontal_image() : Returns the latest image from the frontal camera as a cv2_image
* drone.get_ventral_image() : Returns the latest image from the ventral camera as a cv2_image
* drone.get_position(): Returns the position of the drone as a numpy array [x, y, z]
* drone.get_orientation(): Returns the roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw]
* drone.get_roll(): Returns the roll of the drone
* drone.get_pitch(): Returns the pitch of the drone
* drone.get_yaw(): Returns the yaw of the drone
* drone.set_cmd_vel(vx, vy, vz, az): Commands the linear velocity of the drone in the x, y and z directions and the angular velocity in z in its body fixed frame

## Demonstrative video (in spanish)

https://youtu.be/rIkTImMyoXw
