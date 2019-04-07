# Follow_line practice
The objective of this practice is to perform a PID reactive control capable of following the line painted on the racing circuit.

## How to execute?
To launch the infrastructure of this practice, first set up the gazebo sources, then launch the simulator with the appropriate scenario:
```
source /opt/jderobot/share/jderobot/gazebo/gazebo-setup.sh
```

```
source /opt/jderobot/share/jderobot/gazebo/gazebo-assets-setup.sh
```

```
roslaunch /opt/jderobot/share/jderobot/gazebo/launch/f1_1_simplecircuit.launch world:=/opt/jderobot/share/jderobot/gazebo/worlds/f1_1_simplecircuit.world
```
Then you have to execute the academic application, which will incorporate your code:
```
python2 ./follow_line.py follow_line_conf.yml
```

## How to do the practice?
To carry out the practice, you have to edit the file `MyAlgorithms.py` and insert in it your code, which gives intelligence to the autonomous car.

## Where to insert the code?
[MyAlgorithm.py](MyAlgorithm.py#L87)
```
    def execute(self):
        #GETTING THE IMAGES
        image = self.getImage()

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.motors.sendV(10)
        #self.motors.sendW(5)

        #SHOW THE FILTERED IMAGE ON THE GUI
        self.set_threshold_image(image)
```

### API
* `self.getImage()` - to get the image 
* `self.motors.sendV()` - to set the linear speed
* `self.motors.sendW()` - to set the angular velocity
* `self.set_threshold_image()` - allows you to view a debug image or with relevant information. It must be an image in RGB format (Tip: np.dstack())


## Demonstrative video
[Video](https://www.youtube.com/watch?v=eNuSQN9egpA)

* *Base code made by Alberto Martín (@almartinflorido)*
* *Code of practice performed by Francisco Rivas (@chanfr)*
* *Gazebo models and worlds made by Francisco Pérez (@fqez)*
