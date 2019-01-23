# Follow_line_turtlebot exercise
The objective of this practice is to perform a PID reactive control capable of following the line painted on the ground.

## How to execute?
To launch the infrastructure of this practice, first launch the simulator with the appropriate scenario:
```
roslaunch /opt/jderobot/share/jderobot/gazebo/launch/turtlebot-xtion-followline.launch
```
Then you have to execute the academic application, which will incorporate your code:
```
python2 follow_line_turtlebot.py follow_line_turtlebot_conf.yml
```

## How to do the practice?
To carry out the practice, you have to edit the file `MyAlgorithm.py` and insert in it your code, which gives intelligence to the autonomous car.

## Where to insert the code?
[MyAlgorithm.py](MyAlgorithm.py#L109)
```
    def algorithm(self): # Add your code here
        #EXAMPLE: GETTING IMAGES
        image = self.getImage()
        image2 = self.get_threshold_image()

        #EXAMPLE: PRINTING MESSAGES ON THE TERMINAL
        #print "Running"

        #EXAMPLE: SENDING COMMANDS TO THE MOTORS
        #self.motors.setV(10)
        #self.motors.setW(5)

        #EXAMPLE: SHOWING IMAGES ON THE GUI
        self.set_color_image(image)
        #self.set_threshold_image (image2)

```

### API
* `self.getImage()` - to get the image 
* `self.motors.setV()` - to set the linear speed
* `self.motors.setW()` - to set the angular velocity
* `self.set_threshold_image()` - allows you to view a debug image or with relevant information. It must be an image in RGB format (Tip: np.dstack())

## Credits
* *Base code made by Alberto Martín (@almartinflorido)*
* *Code of practice performed by Julio Vega (@jmvega)*
* *Gazebo models and worlds made by Julio Vega (@jmvega)*
