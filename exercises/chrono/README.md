# Chrono practice
The objective of this practice is to perform a PID reactive control capable of following the line painted on the racing circuit as quickly as possible. It's a chorno test, so your car may be the fastest one on the track.

## How to execute?
To launch the infrastructure of this practice, first launch the simulator with the appropriate scenario:
roslaunch f1-chrono.launch

Then you have to execute the academic application, which will incorporate your code:
python2 chrono.py

## How to do the practice?
To carry out the practice, you have to edit the file MyAlgorithms.py and insert in it your code, which gives intelligence to the autonomous car.

### Where to insert the code?
[MyAlgorithm.py](MyAlgorithm.py#L74)
```
    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft().data
        imageRight = self.sensor.getImageRight().data

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        #self.sensor.setV(10)
        #self.sensor.setW(5)

        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(imageRight)
        self.setLeftImageFiltered(imageLeft)
```

### API
* cameraL.getImage() - to get the left image of the stereo pair
* motors.setV() - to set the linear speed
* motors.setW() - to set the angular velocity
* self.setRightImageFiltered() - allows you to view a debug image or with relevant information. It must be an image in RGB format (Tip: np.dstack())


## Demonstrative video
https://www.youtube.com/watch?v=eNuSQN9egpA

* *Base code made by Alberto Martín (@almartinflorido)*
* *Code of practice performed by Francisco Rivas (@chanfr)*
* *Gazebo models and worlds made by Francisco Pérez (@fqez)*
