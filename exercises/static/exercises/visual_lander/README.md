# Visual Lander Exercise

## Goal

The goal of this exercise is to implement the logic that allows a quadrotor to visualize a beacon and land on it.

![World](../../docs/visual_lander.jpg)

## Requirements

As this is a drones exercise, you will need to additionally install the `jderobot-assets`, `dronewrapper` and `rqt_drone_teleop` packages. These can be installed as

```bash
sudo apt-get install ros-kinetic-drone-wrapper ros-kinetic-rqt-drone-teleop ros-kinetic-jderobot-assets
```

There is an additional dependency on MAVROS and PX4 that we are in the process of simplifying, however at the moment just use the script provided [here](https://github.com/JdeRobot/drones/blob/master/mavros_px4_sitl_installation.sh)

Apart from these, as this exercise also requires a ground robot, you will need the `rqt_ground_robot_teleop` package. We are in the process of making it available through `apt-get` however, until that is available, the best method for it would be to [clone it](https://github.com/JdeRobot/ground_robots) into your catkin_ws.

## Execution

To launch the exercise, simply use the following command from this directory:

`roslaunch visual_lander.launch`

## Solution

To solve the exercise, you must edit the [my_solution.py](my_solution.py#L46) file and insert the control logic into it. Your code is to be entered into the `execute` function between the `Insert your code here` comments.

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

* `set_image_filtered(cv2_image)`: If you want to show a filtered image of the camera images in the GUI
* `set_image_threshed(cv2_image)`: If you want to show a threshold image in the GUI
* `drone.get_frontal_image()` : Returns the latest image from the frontal camera as a cv2_image
* `drone.get_ventral_image()` : Returns the latest image from the ventral camera as a cv2_image
* `drone.get_position()`: Returns the position of the drone as a numpy array [x, y, z]
* `drone.get_orientation()`: Returns the roll, pitch and yaw of the drone as a numpy array [roll, pitch, yaw]
* `drone.get_roll()`: Returns the roll of the drone
* `drone.get_pitch()`: Returns the pitch of the drone
* `drone.get_yaw()`: Returns the yaw of the drone
* `drone.set_cmd_vel(vx, vy, vz, az)`: Commands the linear velocity of the drone in the x, y and z directions and the angular velocity in z in its body fixed frame

## Demonstrative video

https://www.youtube.com/watch?v=i0PGusLHXQM
